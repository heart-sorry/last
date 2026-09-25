/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   引脚定义与外设句柄声明
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* ============================ 电机方向引脚 (TB6612) ============================ */
#define AIN1_Pin          GPIO_PIN_0
#define AIN1_GPIO_Port    GPIOB
#define AIN2_Pin          GPIO_PIN_1
#define AIN2_GPIO_Port    GPIOB
#define BIN1_Pin          GPIO_PIN_4
#define BIN1_GPIO_Port    GPIOA
#define BIN2_Pin          GPIO_PIN_5
#define BIN2_GPIO_Port    GPIOA

/* ============================ OLED 软件IIC引脚 ============================ */
#define OLED_SCL_Pin      GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin      GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* ============================ PWM 限幅 (TIM2 ARR=7199) ============================ */
#define MOTOR_PWM_MAX     7199

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* 外设句柄（定义在 main.c，供 msp.c / 各驱动模块使用） */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
