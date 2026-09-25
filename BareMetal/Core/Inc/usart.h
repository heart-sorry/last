#ifndef __USART_H
#define __USART_H

#include "main.h"

extern UART_HandleTypeDef huart1;   // 串口1句柄（供 uart_vofa 使用）

void USART1_Init(void);              // 串口1初始化（PA9=TX, PA10=RX, 115200）

#endif /* __USART_H */
