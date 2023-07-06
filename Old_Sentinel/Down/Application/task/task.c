/**
  ******************************************************************************
  * @file           : task.c\h
  * @brief          : 
  * @note           : 2022-1-21 22:08:46
  ******************************************************************************
  */

#include "task.h"
#include "cmsis_os.h"
#include "drv_can.h"

extern uint8_t up_com_tx_buf_1[8]; //power_heat
extern uint8_t up_com_tx_buf_2[8]; //game_robot_status
extern uint8_t up_com_tx_buf_3[8]; //shoot_data
extern uint8_t up_com_tx_buf_4[8]; //game_robot_pos
extern uint8_t up_com_tx_buf_5[8]; //robot_hurt

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_RxFrameTypeDef hcan1RxFrame;
extern CAN_RxFrameTypeDef hcan2RxFrame;


/**
  * @brief  ��������
  * @param  
  * @retval 
  */
void Control_Task(void const * argument)
{
	for(;;)
	{
		
		osDelay(1);
	}
}

/**
  * @brief  ʵʱ����
  * @param  
  * @retval 
  */
void Realtime_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}

/**
  * @brief  usmart ����ר������
  * @param  
  * @retval 
  */
void Interaction_Task(void const * argument)
{
  for(;;)
  {
    osDelay(1000);
  }
}

