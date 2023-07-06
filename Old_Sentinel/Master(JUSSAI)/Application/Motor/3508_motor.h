/**
  ******************************************************************************
  * @file           : 3508_motor.c\h
  * @brief          : 
  * @update         : 2022��1��15��13:25:54
  ******************************************************************************
  */

	/**
  ******************************************************************************
  * 3508�������C620���������
	* ʹ�û����¶ȣ�0��C~50��C
	* ���ٱȣ�      3591/187 = 157312.68/8192
  ******************************************************************************
  */


#ifndef __3508_Motor_H
#define __3508_Motor_H

#include "stm32f4xx_hal.h"
//#include "pid.h"
#include "drv_can.h"


#define MOTOR_3508_SENT_ID 0x200
#define _DEV_OFFLINE        0
#define _DEV_ONLINE         1
#define OFFLINE_TIME_MAX   25




typedef struct
{
	int16_t   angle;            //����Ƕȣ�0~8191��
	int16_t   speed;            //���ת�٣�RPM����max = 7600��
	int16_t   current;          //ת�ص���-16384~16384��-20~20A��
	int16_t   temperature;      //��C
	int16_t   target_prm; 			//Ŀ��ת��
	int16_t   target_angle;
	int16_t   gap_angle;
}motor_3508_base_info_t;

typedef struct 
{
	uint8_t          offline_cnt;
	uint8_t          offline_cnt_max;
	uint8_t          status;
}motor_3508_info_t;

typedef struct motor_3508_t
{
	motor_3508_base_info_t *base_info;
	motor_3508_info_t      *info;
	int16_t                output_current;
	
}motor_3508_t;


void MOTOR_3508_INIT(motor_3508_t *motor,\
										 motor_3508_base_info_t *base,\
										 motor_3508_info_t *info);

void MOTOR_3508_GET_DATA1(motor_3508_t *motor);
void MOTOR_3508_GET_DATA2(motor_3508_t *motor);
HAL_StatusTypeDef MOTOR_3508_CAN1_SENT_DATA( uint16_t data_1,uint16_t data_2,uint16_t data_3,uint16_t data_4);
HAL_StatusTypeDef MOTOR_3508_CAN2_SENT_DATA( uint16_t data_1,uint16_t data_2,uint16_t data_3,uint16_t data_4);
void MOTOR_3508_HEART(motor_3508_t *motor);
#endif
