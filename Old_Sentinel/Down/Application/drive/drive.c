/**
  ******************************************************************************
  * @file           : drive.c/h
  * @brief          : 
  * @note           : finish 2022-2-12 12:17:38
  ******************************************************************************
  */

#include "drive.h"
#include "bmi2_defs.h"
#include "bmi.h"
#include "drv_can.h"
#include "drv_uart.h"

#include "judge.h"


void drive_init(void)
{
  /* 外设初始化 */
  CAN1_Init();
  CAN2_Init();
  USART1_Init();
  USART2_Init();
  USART3_Init();
  USART4_Init();
  USART5_Init();
}

void module_init(void)
{
}
