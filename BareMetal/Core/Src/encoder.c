/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   编码器读取 (TIM3 编码器接口模式, PA6=A相, PA7=B相)
  ******************************************************************************
  */
#include "encoder.h"

/* TIM3 句柄在 main.c 中定义 */
extern TIM_HandleTypeDef htim3;

/* 滑动测速窗口长度(ms)。窗口越长低速分辨率越高、越平滑；
 * 50ms 时速度分辨率 = 1脉冲/50ms = 0.2(脉冲/10ms) */
#define SPEED_WIN_MS   50u

/* 编码器全局数据 */
volatile int32_t g_encoderCount = 0;
volatile float   g_encoderSpeed = 0.0f;

/* 滑动窗口：保存最近 SPEED_WIN_MS 个1ms增量，每1ms滚动更新 */
static int16_t  s_ring[SPEED_WIN_MS];
static uint16_t s_ringIdx = 0;
static int32_t  s_ringSum = 0;

static int16_t  s_lastCnt = 0;

/**
  * @brief  读取编码器（每1ms调用一次）
  *         Count: 累计脉冲
  *         Speed: 最近50ms滑动窗口平均速度，折算为"脉冲/10ms"
  *                每1ms都刷新，PID(10ms)每次都能拿到新鲜平滑的反馈
  */
void Encoder_Read(void)
{
    int16_t curCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /* 1ms 增量，按方向宏取符号 */
    int16_t delta = (int16_t)((curCnt - s_lastCnt) * ENCODER_DIR);
    s_lastCnt = curCnt;
    g_encoderCount += delta;

    /* 滑动窗口：去掉最旧样本，加入最新样本 */
    s_ringSum -= s_ring[s_ringIdx];
    s_ring[s_ringIdx] = delta;
    s_ringSum += delta;
    s_ringIdx++;
    if (s_ringIdx >= SPEED_WIN_MS) s_ringIdx = 0;

    /* 折算为 脉冲/10ms（与目标速度同量纲） */
    g_encoderSpeed = (float)s_ringSum * (10.0f / (float)SPEED_WIN_MS);
}
