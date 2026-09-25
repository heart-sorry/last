/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   编码器读取 (TIM3 编码器接口模式, PA6=A相, PA7=B相)
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 编码器方向系数：
 *   +1 = 默认方向（电机正转时 Speed 为正）
 *   -1 = 方向取反（若上电后电机全速失控，或正转时 Speed 显示负数，改成 -1）
 */
#define ENCODER_DIR        -1

/* 编码器数据：由 Encoder_Read() 更新（任务中每1ms调用一次） */
extern volatile int32_t g_encoderCount;   /* 累计脉冲数（总行程） */
extern volatile float   g_encoderSpeed;   /* 50ms窗口平均速度（单位:脉冲/10ms，带小数，低速也稳定） */

/**
  * @brief  读取编码器（每1ms调用一次）
  * @note   TIM3 配置为 TI12 双边沿4倍频，ARR=65535
  *         读 CNT 差值得到增量，int16差值自动处理溢出回绕
  */
void Encoder_Read(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
