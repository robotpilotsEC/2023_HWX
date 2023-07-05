/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v2.0		
  * @Author     : hwx			
  * @Date       : 2023-7-06         
  * @Description:    
  *
  *
  ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "2006_motor.h"
#include "rp_math.h"
#include "string.h"
#include "can_protocol.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define one(x)					((x)>0? (1):(-1))


/* Exported variables --------------------------------------------------------*/
extern Master_Head_t Master_Head_structure;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_RxFrameTypeDef hcan1RxFrame;
extern CAN_RxFrameTypeDef hcan2RxFrame;


/* Function  body --------------------------------------------------------*/
/**
  * @brief  ��ʼ��6020�ṹ��
  * @param  motor_6020_t*  motor_6020_base_info_t* motor_6020_info_t*
	* @note   �����Լ������˸��ṹ��ָ�븳ֵ
  */
void Motor_2006_Init(motor_2006_t *motor,\
										 motor_2006_base_info_t* base,\
										 motor_2006_info_t* info)
{
	motor->base_info=base;
	memset(motor->base_info,0,sizeof(motor_2006_base_info_t));
	
	motor->info=info;
	motor->info->status=_DEV_OFFLINE;
	
	motor->info->offline_cnt=OFFLINE_TIME_MAX;
	motor->info->offline_cnt_max=OFFLINE_TIME_MAX;
	
	motor->output_current=0;
	motor->base_info->circle_cnt = 0;
	
	motor->base_info->angle = 0 ;
	motor->base_info->target_angle_sum= 0 ; 
	motor->base_info->done_angle = 0;
}


/**
	* @brief  �������ݸ�2006
  * @param  
* @retval   ������ΪM2H����ģʽ����̨ģʽ�뷢�����ģʽ
	* @note   ʹ��can2
  */
HAL_StatusTypeDef Motor_2006_CanSentData( int16_t data_1,int16_t data_2,int16_t data_3,int16_t data_4)//����ٸģ���ת����
{
	
	
	uint32_t txMailBox;//��������
	CAN_TxHeaderTypeDef txFrameHeader;
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint8_t data[8];
	
	/*��������*/

	/*���ݴ���*/
	data[0]      = data_1 >> 8;
	data[1]      = data_1;      //��������
	data[2]      = data_2 >> 8;
	data[3]      = data_2;      //��������
	
	data[4]      = Master_Head_structure.Send_L_Head.gimbal_mode; //�� ��̨
	data[5]      = Master_Head_structure.Send_R_Head.gimbal_mode; //�� ��̨
	data[6]      = Master_Head_structure.Send_L_Head.shoot_mode; //�� ����
	data[7]      = Master_Head_structure.Send_R_Head.shoot_mode; //�� ����
	/*����֡��ͷ*/
	txFrameHeader.StdId = MOTOR_2006_SENT_ID;
	txFrameHeader.IDE   = CAN_ID_STD;
	txFrameHeader.RTR   = CAN_RTR_DATA;
	txFrameHeader.DLC   = 0x08;
	
	
	  /*Ĭ��ʹ��CAN2*/
  ret = HAL_CAN_AddTxMessage(&hcan2, &txFrameHeader, data, &txMailBox);
    
	memset(data,0,sizeof(data));
	
	
	return ret;
		
		
}


/**
  * @brief  ���������е�����д��ṹ��
  * @param  motor_2006_t *
  * @retval NONE
  */
void Motor_2006_GetData(motor_2006_t *motor)
{
	int16_t  angle_last = motor->base_info->angle;
	
	/*������Ϣ */
	motor->base_info->angle = hcan2RxFrame.data[0];
	motor->base_info->angle <<= 8;
	motor->base_info->angle |= hcan2RxFrame.data[1];
	motor->base_info->speed = hcan2RxFrame.data[2];
	motor->base_info->speed <<= 8;
	motor->base_info->speed |= hcan2RxFrame.data[3];
	motor->base_info->current = hcan2RxFrame.data[4];
	motor->base_info->current <<= 8;
	motor->base_info->current |= hcan2RxFrame.data[5];
	motor->base_info->temperature = hcan2RxFrame.data[6];	
	
	
	/* angle_add */
	motor->base_info->angle_add = motor->base_info->angle - angle_last;
	
	if(abs(motor->base_info->angle_add) > 4096)
	{
		motor->base_info->angle_add -= 8192 * one(motor->base_info->angle_add);
	}
	
}


/**
  * @brief  ������Ƿ����
  * @param  motor_2006_t *
  * @retval NONE
  */
void Motor_2006_HeartBeat(motor_2006_t *motor)
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

