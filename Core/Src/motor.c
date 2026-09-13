/**
  ******************************************************************************
  * @file    motor.c
  * @brief   直流电机驱动 (TB6612)
  ******************************************************************************
  */
#include "motor.h"

/* TIM2 句柄在 main.c 中定义 */
extern TIM_HandleTypeDef htim2;

/**
  * @brief  电机原始输出（只做限幅，不做死区补偿）
  * @param  speed: -MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX，供开环测试使用，可输出任意小PWM
  */
void Motor_SetRaw(int16_t speed)
{
    uint16_t pwm;

    if (speed > MOTOR_PWM_MAX)  speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0)
    {
        pwm = (uint16_t)speed;
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        pwm = (uint16_t)(-speed);
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    else
    {
        pwm = 0;
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
}

/**
  * @brief  设置电机速度 (-MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX)，带死区补偿，闭环PID使用
  */
void Motor_SetSpeed(int16_t speed)
{
    /* 死区补偿：非0输出不小于起步PWM */
    if (speed > 0 && speed < MOTOR_PWM_MIN)
        speed = MOTOR_PWM_MIN;
    else if (speed < 0 && speed > -MOTOR_PWM_MIN)
        speed = -MOTOR_PWM_MIN;

    Motor_SetRaw(speed);
}
