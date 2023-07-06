#ifndef __DRV_UART_H
#define __DRV_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);

void UART_SendData(uint8_t *Data,UART_HandleTypeDef huart);

void USART2_Init(void);
void USART4_Init(void);
void USART5_Init(void);
void USART1_Init(void);
void USART3_Init(void);

#endif
