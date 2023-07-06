
#include "2006_motor.h"
#include "rp_math.h"
#include "string.h"
#include "my_shoot.h"

#define one(x)					((x)>0? (1):(-1))

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_RxFrameTypeDef hcan1RxFrame;
extern CAN_RxFrameTypeDef hcan2RxFrame;

extern shoot_t shoot_structure;
extern int8_t  deal_done;



/**
  * @brief  ��ʼ��6020�ṹ��
  * @param  motor_6020_t*  motor_6020_base_info_t* motor_6020_info_t*
	* @note   �����Լ������˸��ṹ��ָ�븳ֵ
  */

void MOTOR_2006_INIT(motor_2006_t *motor,\
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
  * @retval 
	* @note   ʹ��can2
  */
HAL_StatusTypeDef MOTOR_2006_CAN_SENT_DATA( int16_t data_1,int16_t data_2,int16_t data_3,int16_t data_4)//����ٸģ���ת����
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
void MOTOR_2006_GET_DATA(motor_2006_t *motor)
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
	
	if(shoot_structure.work_sta == S_S_oneshot || shoot_structure.stri_mode == ONE_SHOOT )
	{
		#if 0 //������Ȧ��
		if(angle_last - motor->base_info->angle > 5000)
		{
			motor->base_info->circle_cnt++;
		}
		else if(angle_last - motor->base_info->angle < (-5000))
		{
			motor->base_info->circle_cnt--;
		}
		
		motor->base_info->angle_sum = motor->base_info->circle_cnt*8191 + motor->base_info->angle;
		#endif
		
		#if 1 //�ǶȻ���
		/* angle_add */
		motor->base_info->angle_add = motor->base_info->angle - angle_last;
		
		if(abs(motor->base_info->angle_add) > 4096)
		{
			motor->base_info->angle_add -= 8192 * one(motor->base_info->angle_add);
		}
		
		/* angle_sum and target_angle_sum */
		motor->base_info->angle_sum += motor->base_info->angle_add;
		
		#endif
	}
	
	if(deal_done)
	{
		/* angle_add */
		motor->base_info->angle_add = motor->base_info->angle - angle_last;
		
		if(abs(motor->base_info->angle_add) > 4096)
		{
			motor->base_info->angle_add -= 8192 * one(motor->base_info->angle_add);
		}
		
		/* angle_sum and target_angle_sum */
		motor->base_info->done_angle += motor->base_info->angle_add;
		
	}
	
}

