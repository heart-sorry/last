/**
  ******************************************************************************
  * @file    motor.h
  * @brief   直流电机驱动 (TB6612)
  *          PWM: PA0=TIM2_CH1(A通道), PA1=TIM2_CH2(B通道)
  *          方向: AIN1=PB0 AIN2=PB1 / BIN1=PA4 BIN2=PA5
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 电机起步最小PWM（死区补偿）：
 * PWM 小于此值时电机无法克服静摩擦。若发现低速一顿一顿就加大此值；
 * 若停车时仍缓慢蠕动就减小。当前占比约 MOTOR_PWM_MIN/MOTOR_PWM_MAX ≈ 12.5%。
 */
#define MOTOR_PWM_MIN     900

/**
  * @brief  电机原始输出（只限幅，无死区补偿），开环测试用
  * @param  speed: -MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX
  */
void Motor_SetRaw(int16_t speed);

/**
  * @brief  设置电机速度
  * @param  speed: 速度值，范围 -MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX
  *                正数正转，负数反转，0停止
  * @note   A/B两通道同步输出；只接一个电机时使用A通道(AO1/AO2)即可
  */
void Motor_SetSpeed(int16_t speed);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
