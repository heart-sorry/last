#include "pid_demo.h"
#include "encoder.h"   // 编码器速度 g_encoderSpeed
#include "main.h"      // MOTOR_PWM_MAX

/* 内部功能函数 */
static void pid_formula_incremental(PID_T * _tpPID);
static void pid_formula_positional(PID_T * _tpPID);
static void pid_out_limit(PID_T * _tpPID);

/* 当前系统PID */
PID_T g_tAnglePID = {0};        /* 角度环 */
PID_T g_tLeftMotorPID = {0};    /* 左电机速度环 */
PID_T g_tRightMotorPID = {0};   /* 右电机速度环 */

/*******************************************************************************
 * @brief   PID参数初始化
 ******************************************************************************/
static void pid_InitVal(void)
{
    /* 角度环 */
    g_tAnglePID.kp = 1.0;
    g_tAnglePID.ki = 2.3;
    g_tAnglePID.kd = 21.61;
    g_tAnglePID.target = 0;
    g_tAnglePID.limit = 0;

    /* 左电机速度环（增量式：先调I再调P，I是主力，P阻尼，D一般不用）
     * 参数已按 PWM 分辨率 199→7199 放大 36 倍标定 */
    g_tLeftMotorPID.kp = 18.0;      // 比例（增量式中P抑制误差变化）
    g_tLeftMotorPID.ki = 10.8;      // 积分（增量式中I决定响应快慢）
    g_tLeftMotorPID.kd = 0.0;       // 微分
    g_tLeftMotorPID.target = 0;     // 目标值[可由外部设置]
    g_tLeftMotorPID.limit = MOTOR_PWM_MAX;    // 限幅值（PWM最大）

    /* 右电机速度环 */
    g_tRightMotorPID.kp = 18.0;
    g_tRightMotorPID.ki = 10.8;
    g_tRightMotorPID.kd = 0.0;
    g_tRightMotorPID.target = 0;
    g_tRightMotorPID.limit = MOTOR_PWM_MAX;
}


/*******************************************************************************
 * @brief   PID初始化
 ******************************************************************************/
void pid_Init(void)
{
    pid_InitVal();
}

/*******************************************************************************
 * @brief   角度闭环控制函数[角度环]
 * @note    位置式校，P-快速性，I-准确性，D-稳定性。一般按顺序调P,D再I
 ******************************************************************************/
void pid_closeloop_angle(float _angle_target)
{
    /* [设置目标值] */
    g_tAnglePID.target = _angle_target;

    /* [获取反馈值/当前值] - 读取当前角度 */
    // g_tAnglePID.current = MPUGETDATA(&);

    /* [通过PID公式计算PID输出] */
    pid_formula_positional(&g_tAnglePID);

    /* [限幅PID输出] */
    pid_out_limit(&g_tAnglePID);
}

/*******************************************************************************
 * @brief   电机闭环控制函数[速度环]
 * @note    增量式校，P-稳定性，I-准确性，D-快速性。一般按顺序调I,P再D
 *          输出out值可作用于电机驱动 Motor_SetSpeed(out);
 ******************************************************************************/
void pid_closeloop_motor(float _left_target, float _right_target)
{
    /* [设置目标值] */
    g_tLeftMotorPID.target = _left_target;
    g_tRightMotorPID.target = _right_target;

    /* [获取反馈值/当前值] - 读取编码器速度作为当前速度 */
    g_tLeftMotorPID.current  = g_encoderSpeed;
    g_tRightMotorPID.current = g_encoderSpeed;

    /* [通过PID公式计算PID输出] 电机速度环使用增量式PID */
    pid_formula_incremental(&g_tLeftMotorPID);
    pid_formula_incremental(&g_tRightMotorPID);

    /* [限幅PID输出] */
    pid_out_limit(&g_tLeftMotorPID);
    pid_out_limit(&g_tRightMotorPID);
}


/*******************************************************************************
 * @brief   PID输出限幅
 ******************************************************************************/
static void pid_out_limit(PID_T * _tpPID)
{
    if(_tpPID->out > _tpPID->limit)
        _tpPID->out = _tpPID->limit;
    else if(_tpPID->out < -_tpPID->limit)
        _tpPID->out = -_tpPID->limit;
}

/*******************************************************************************
 * @brief   增量式PID公式
 * @note    增量式校，P-稳定性，I-准确性，D-快速性。
 ******************************************************************************/
static void pid_formula_incremental(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;

    _tpPID->p_out = _tpPID->kp * (_tpPID->error - _tpPID->last_error);
    _tpPID->i_out = _tpPID->ki * _tpPID->error;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - 2 * _tpPID->last_error + _tpPID->last2_error);

    /* 增量累加 + 抗饱和：到达限幅后不再继续向限幅方向累加 */
    float new_out = _tpPID->out + _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;
    if (new_out > _tpPID->limit)       new_out =  _tpPID->limit;
    else if (new_out < -_tpPID->limit) new_out = -_tpPID->limit;
    _tpPID->out = new_out;

    _tpPID->last2_error = _tpPID->last_error;
    _tpPID->last_error = _tpPID->error;
}

/*******************************************************************************
 * @brief   位置式PID公式
 * @note    位置式校，P-快速性，I-准确性，D-稳定性。
 ******************************************************************************/
static void pid_formula_positional(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;
    _tpPID->integral += _tpPID->error;

    /* 积分限幅（抗积分饱和）：积分项本身不允许超过输出限幅 */
    if (_tpPID->ki != 0.0f)
    {
        float intMax = _tpPID->limit / _tpPID->ki;
        if (_tpPID->integral > intMax)  _tpPID->integral = intMax;
        if (_tpPID->integral < -intMax) _tpPID->integral = -intMax;
    }

    _tpPID->p_out = _tpPID->kp * _tpPID->error;
    _tpPID->i_out = _tpPID->ki * _tpPID->integral;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - _tpPID->last_error);

    _tpPID->out = _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;

    _tpPID->last_error = _tpPID->error;
}
