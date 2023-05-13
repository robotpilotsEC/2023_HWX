/**
  ******************************************************************************
  * @file           : can_potocol.c/h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
#ifndef __CAN_POTOCOL_H
#define __CAN_POTOCOL_H

#include "stm32f4xx_hal.h"

typedef struct 
{
  CAN_RxHeaderTypeDef Header;
  uint8_t Data[8];
}can_rx_info_t;

typedef struct 
{
  CAN_TxHeaderTypeDef Header;
  uint8_t Data[8];
}can_tx_info_t;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
