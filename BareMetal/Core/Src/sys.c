/* =============================头文件============================= */
#include "sys.h"
#include "main.h"
#include <stdio.h>
#include "encoder.h"
#include "pid_demo.h"
#include "uart_vofa.h"
#include "motor.h"
#include "OLED_IIC_Config.h"
#include "OLED_Function.h"
#include "MyDelay.h"

/* =============================状态机============================= */
SYS_FLAG_STA_E g_eSysFlagManage;  // 系统标志位管理

/* 电机速度目标值（可通过 VOFA+ 上位机修改） */
static float s_motorTargetSpeed = 0.0f;

/* 开环测试模式：open=1 时直接用 pwm 驱动电机，用于测定死区和最低稳定转速 */
static float s_openLoop = 0.0f;   /* 0=PID闭环(默认)  1=开环直控PWM */
static float s_openPwm  = 0.0f;   /* 开环PWM值，范围 -MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX */

/* 启动冲量(kick-start)参数：静止→转动瞬间先给大PWM克服静摩擦 */
#define KICK_TICKS     15     /* 冲量持续时间，单位=PID周期10ms，15=150ms */
static float    s_kickPwm = 1800.0f; /* 启动冲量PWM，可用VOFA命令 kick=xx 在线修改 */
static uint16_t s_kickCnt = 0; /* 冲量剩余节拍 */
static uint8_t  s_wasStop = 1; /* 上一时刻是否停车 */

/* VOFA 发送用的浮点变量（每轮任务更新） */
static float s_vofaEncoderSpeed = 0.0f;
static float s_vofaEncoderCount = 0.0f;


/* =============================任务函数声明============================= */
static void Encoder_Read_Task(void);   // 编码器读取任务
static void Motor_PID_Task(void);      // 电机速度PID任务
static void VOFA_Send_Task(void);      // VOFA数据发送任务
static void OLED_Display_Task(void);   // OLED显示任务


/******************************************************************
 * @brief   任务汇总表
 * @note    {所属状态, Run, 初始计时, 重载计时(ms), 任务函数}
 *          系统时基 1ms（TIM4），TimRload 单位 ms
 ******************************************************************/
static TaskComps_T g_tTaskComps[] =
{
    {SYS_STA_RUN, 0,   1,   1,   Encoder_Read_Task},   // 编码器读取   (1ms)
    {SYS_STA_RUN, 0,  10,  10,   Motor_PID_Task},      // 电机速度PID  (10ms)
    {SYS_STA_RUN, 0,  10,  10,   VOFA_Send_Task},      // VOFA发送     (10ms)
    {SYS_STA_RUN, 0, 100, 100,   OLED_Display_Task},   // OLED显示     (100ms)
};



/* =============================系统函数============================= */
static void TaskStartSchedule(void);

/******************************************************************
 * @brief   系统初始化（应用层）
 * @note    硬件初始化已由 main.c 中 CubeMX 生成的 MX_*_Init() 完成
 *          本函数只做应用层初始化：PID/VOFA/OLED
 ******************************************************************/
void SysInit(void)
{
    g_eSysFlagManage = SYS_STA_INIT;

    /* 应用初始化 */
    pid_Init();          // PID参数初始化

    OLED_Init();         // OLED
    OLED_ClearRAM();
    OLED_RefreshRAM();

    /* VOFA+ 初始化并注册数据通道 */
    vofa_init();
    vofa_register_float(&s_vofaEncoderSpeed);      // 通道1: 编码器速度
    vofa_register_float(&s_vofaEncoderCount);      // 通道2: 编码器累计值
    vofa_register_float(&g_tLeftMotorPID.out);     // 通道3: PID输出
    vofa_register_float(&s_motorTargetSpeed);      // 通道4: 目标速度
    vofa_bind_cmd("speed", &s_motorTargetSpeed);   // 上位机命令: speed=xxx
    vofa_bind_cmd("kp", &g_tLeftMotorPID.kp);      // 在线调P: kp=2.0
    vofa_bind_cmd("ki", &g_tLeftMotorPID.ki);      // 在线调I: ki=0.1
    vofa_bind_cmd("kd", &g_tLeftMotorPID.kd);      // 在线调D: kd=0.5
    vofa_bind_cmd("open", &s_openLoop);            // open=1进入开环测试, open=0回闭环
    vofa_bind_cmd("pwm",  &s_openPwm);             // 开环下直接设定PWM: pwm=900
    vofa_bind_cmd("kick", &s_kickPwm);             // 启动冲量PWM: kick=1800
}

/******************************************************************
 * @brief   运行函数
 ******************************************************************/
void SysRun(void)
{
    g_eSysFlagManage = SYS_STA_RUN;
    TaskStartSchedule();
}

/******************************************************************
 * @brief   任务调度主循环
 ******************************************************************/
static void TaskStartSchedule(void)
{
    while(1)
    {
        for (uint8_t i = 0; i < sizeof(g_tTaskComps) / sizeof(g_tTaskComps[0]); i++)
        {
            if (g_tTaskComps[i].Run)
            {
                g_tTaskComps[i].pTaskFunc();
                g_tTaskComps[i].Run = 0;
            }
        }
    }
}

/******************************************************************
 * @brief   时间片管理（1ms 中断调用）
 ******************************************************************/
void TaskTimeSliceManage(void)
{
    for (uint8_t i = 0; i < sizeof(g_tTaskComps) / sizeof(g_tTaskComps[0]); i++)
    {
        if (g_tTaskComps[i].SysStaID == g_eSysFlagManage && g_tTaskComps[i].TimCount)
        {
            g_tTaskComps[i].TimCount--;
            if (g_tTaskComps[i].TimCount == 0)
            {
                g_tTaskComps[i].TimCount = g_tTaskComps[i].TimRload;
                g_tTaskComps[i].Run = 1;
            }
        }
    }
}



/* =============================应用任务函数============================= */

/******************************************************************
 * @brief   编码器读取任务 (1ms)
 ******************************************************************/
static void Encoder_Read_Task(void)
{
    Encoder_Read();
    s_vofaEncoderSpeed = g_encoderSpeed;
    s_vofaEncoderCount = (float)g_encoderCount;
}

/******************************************************************
 * @brief   电机速度PID任务 (10ms)
 * @note    用编码器速度做反馈，输出 g_tLeftMotorPID.out
 ******************************************************************/
static void Motor_PID_Task(void)
{
    /* 开环测试模式：直接输出PWM，不做PID（用于测定死区/最低稳定转速） */
    if (s_openLoop >= 0.5f)
    {
        g_tLeftMotorPID.out = s_openPwm;   /* 让VOFA通道3显示当前PWM */
        s_wasStop = (s_openPwm == 0.0f);
        s_kickCnt = 0;
        Motor_SetRaw((int16_t)s_openPwm);
        return;
    }

    /* 目标为0：直接停车并复位增量式PID状态（out/误差历史清零） */
    if (s_motorTargetSpeed == 0.0f)
    {
        g_tLeftMotorPID.out = 0;
        g_tLeftMotorPID.integral = 0;
        g_tLeftMotorPID.p_out = 0;
        g_tLeftMotorPID.i_out = 0;
        g_tLeftMotorPID.d_out = 0;
        g_tLeftMotorPID.error = 0;
        g_tLeftMotorPID.last_error = 0;
        g_tLeftMotorPID.last2_error = 0;
        g_tRightMotorPID.out = 0;
        g_tRightMotorPID.last_error = 0;
        g_tRightMotorPID.last2_error = 0;
        s_wasStop = 1;
        s_kickCnt = 0;
        Motor_SetSpeed(0);
        return;
    }

    /* 启动沿：停车→转动，先给冲量克服静摩擦。
     * 增量式输出是递推累加的，冲量结束后让out从冲量值开始接续，
     * PID只需小幅增减即可平滑接管，不会再次卡住。 */
    if (s_wasStop)
    {
        s_wasStop = 0;
        s_kickCnt = KICK_TICKS;
        g_tLeftMotorPID.out = 0;
        g_tLeftMotorPID.last_error = 0;
        g_tLeftMotorPID.last2_error = 0;
    }

    /* 冲量持续阶段：强制大PWM，跳过PID */
    if (s_kickCnt > 0)
    {
        s_kickCnt--;
        /* 冲量幅值保护：取绝对值并限幅 0~MOTOR_PWM_MAX，方向跟随目标 */
        float kp_abs = (s_kickPwm >= 0.0f) ? s_kickPwm : -s_kickPwm;
        if (kp_abs > MOTOR_PWM_MAX) kp_abs = MOTOR_PWM_MAX;
        float kick = (s_motorTargetSpeed > 0.0f) ? kp_abs : -kp_abs;
        g_tLeftMotorPID.out = kick;

        /* 冲量最后一拍：预置误差历史，使下一拍P项增量为0，避免接管瞬间跳变 */
        if (s_kickCnt == 0)
        {
            float errNow = s_motorTargetSpeed - g_encoderSpeed;
            g_tLeftMotorPID.error = errNow;
            g_tLeftMotorPID.last_error = errNow;
            g_tLeftMotorPID.last2_error = errNow;
        }

        Motor_SetRaw((int16_t)kick);
        return;
    }

    /* 增量式PID计算（out内部递推累加） */
    pid_closeloop_motor(s_motorTargetSpeed, s_motorTargetSpeed);

    float out = g_tLeftMotorPID.out;

    /* 单向滑行策略：朝目标方向转动时，反向输出钳为0（滑行而非反接刹车）。
     * 增量式必须把钳位值写回out，作为下一拍累加的基准，否则会持续憋输出。 */
    if (s_motorTargetSpeed > 0.0f && out < 0.0f)
    {
        out = 0.0f;
    }
    else if (s_motorTargetSpeed < 0.0f && out > 0.0f)
    {
        out = 0.0f;
    }

    g_tLeftMotorPID.out = out;
    Motor_SetSpeed((int16_t)out);
}

/******************************************************************
 * @brief   VOFA数据发送任务 (10ms)
 ******************************************************************/
static void VOFA_Send_Task(void)
{
    vofa_run();
}

/******************************************************************
 * @brief   OLED显示任务 (100ms)
 ******************************************************************/
static void OLED_Display_Task(void)
{
    unsigned char buf[32];

    OLED_ClearRAM();

    OLED_ShowStr(0, 0, (unsigned char*)(s_openLoop >= 0.5f ? "OPEN" : "PID "), 1);
    OLED_ShowStr(28, 0, (unsigned char*)"==Encoder==", 1);

    long sp10, tg10, spI, tgI;

    sprintf((char*)buf, "Count:%ld", (long)g_encoderCount);
    OLED_ShowStr(0, 16, buf, 1);

    /* 浮点放大10倍后手工取整/小数，避免ARMCC V5的%f格式化问题 */
    sp10 = (long)(g_encoderSpeed * 10.0f);
    spI  = (sp10 >= 0) ? (sp10 / 10) : -((-sp10) / 10);
    sprintf((char*)buf, "Speed:%ld.%ld", spI,
            (sp10 >= 0) ? (sp10 % 10) : ((-sp10) % 10));
    OLED_ShowStr(0, 32, buf, 1);

    tg10 = (long)(s_motorTargetSpeed * 10.0f);
    tgI  = (tg10 >= 0) ? (tg10 / 10) : -((-tg10) / 10);
    sprintf((char*)buf, "Target:%ld.%ld", tgI,
            (tg10 >= 0) ? (tg10 % 10) : ((-tg10) % 10));
    OLED_ShowStr(0, 48, buf, 1);

    OLED_RefreshRAM();
}
