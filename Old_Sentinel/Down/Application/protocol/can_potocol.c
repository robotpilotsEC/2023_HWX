/**
  ******************************************************************************
  * @file           : can_potocol.c/h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#include "can_potocol.h"
#include "drv_can.h"
#include "config_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

can_rx_info_t CAN_RxInfo;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{//can接受中断，在stm32f4xx_hal_can.c内弱定义
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxInfo.Header, CAN_RxInfo.Data);
  if(hcan == &hcan1)
  {
    CAN1_Get_Data(CAN_RxInfo.Header.StdId, CAN_RxInfo.Data);
  }
  else if(hcan == &hcan2)
  {
    CAN2_Get_Data(CAN_RxInfo.Header.StdId, CAN_RxInfo.Data);
  }
  else 
  {
    return;
  }
}
