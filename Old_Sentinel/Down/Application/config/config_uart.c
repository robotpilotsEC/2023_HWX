/**
  ******************************************************************************
  * @file           : config_uart.c\h
  * @brief          : 
  * @note           : finish 2022-2-12 12:24:28
  ******************************************************************************
  */
	
#include "config_uart.h"
#include "judge_protocol.h"

void USART1_rxDataHandler(uint8_t *rxBuf)
{
}

void USART2_rxDataHandler(uint8_t *rxBuf)
{
}

void USART3_rxDataHandler(uint8_t *rxBuf)
{
}

void USART4_rxDataHandler(uint8_t *rxBuf)
{
}

void USART5_rxDataHandler(uint8_t *rxBuf)
{
	judge_recive(rxBuf);
}
