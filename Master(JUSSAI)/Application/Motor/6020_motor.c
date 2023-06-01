
#include "6020_motor.h"
#include "rp_math.h"
#include "string.h"





extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_RxFrameTypeDef hcan1RxFrame;



/**
  * @brief  ��ʼ��6020�ṹ��
  * @param  motor_6020_t*  motor_6020_base_info_t* motor_6020_info_t*
	* @note   �����Լ������˸��ṹ��ָ�븳ֵ
  */

void MOTOR_6020_INIT(motor_6020_t *motor,\
										 motor_6020_base_info_t* base,\
										 motor_6020_info_t* info)
{
	motor->base_info=base;
	memset(motor->base_info,0,sizeof(motor_6020_base_info_t));
	
	motor->info=info;
	motor->info->status=_DEV_OFFLINE;
	
	motor->info->offline_cnt=OFFLINE_TIME_MAX;
	motor->info->offline_cnt_max=OFFLINE_TIME_MAX;
	
	motor->output_current=0;
	
}


/**
	* @brief  �������ݸ�6020
  * @param  
  * @retval 
	* @note   ʹ��can1
  */
HAL_StatusTypeDef MOTOR_6020_CAN_SENT_DATA( int16_t data_1,int16_t data_2,int16_t data_3,int16_t data_4)//����ٸģ���ת����
{

	uint32_t txMailBox;//��������
	CAN_TxHeaderTypeDef txFrameHeader;
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint8_t data[8];
	/*���ݴ���*/
	data[0]      = data_1 >> 8;
	data[1]      = data_1;
	data[2]      = data_2 >> 8;
	data[3]      = data_2;
	data[4]      = data_3 >> 8;
	data[5]      = data_3;
	data[6]      = data_4 >> 8;
	data[7]      = data_4;
	/*����֡��ͷ*/
	txFrameHeader.StdId = MOTOR_6020_SENT_ID;
	txFrameHeader.IDE   = CAN_ID_STD;
	txFrameHeader.RTR   = CAN_RTR_DATA;
	txFrameHeader.DLC   = 0x08;
	
	
	  /*Ĭ��ʹ��CAN1*/
  ret = HAL_CAN_AddTxMessage(&hcan1, &txFrameHeader, data, &txMailBox);
    
	memset(data,0,sizeof(data));
	
	return ret;
		
		
}


/**
  * @brief  ���������е�����д��ṹ��
  * @param  motor_6020_t *
  * @retval NONE
  */
void MOTOR_6020_GET_DATA(motor_6020_t *motor)
{

	/*������Ϣ */
	motor->base_info->angle = hcan1RxFrame.data[0];
	motor->base_info->angle <<= 8;
	motor->base_info->angle |= hcan1RxFrame.data[1];
	motor->base_info->speed = hcan1RxFrame.data[2];
	motor->base_info->speed <<= 8;
	motor->base_info->speed |= hcan1RxFrame.data[3];
	motor->base_info->current = hcan1RxFrame.data[4];
	motor->base_info->current <<= 8;
	motor->base_info->current |= hcan1RxFrame.data[5];
	motor->base_info->temperature = hcan1RxFrame.data[6];
	
	/*����������*/
	motor->info->offline_cnt = 0;
}

/**
  * @brief  ������Ƿ����
  * @param  motor_6020_t *
  * @retval NONE
  */
void MOTOR_6020_HEART(motor_6020_t *motor)
{
	if(motor->info->offline_cnt++ >= motor->info->offline_cnt_max)
	{
		motor->info->status = _DEV_OFFLINE;
		motor->info->offline_cnt = motor->info->offline_cnt_max;
	}
	else
	{
		motor->info->status = _DEV_ONLINE;
	}
}

