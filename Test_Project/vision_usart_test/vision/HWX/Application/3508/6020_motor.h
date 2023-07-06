/**
  ******************************************************************************
  * @file           : 3508_motor.c\h
  * @brief          : 
  * @update         : 2022年1月15日13:25:54
  ******************************************************************************
  */

	/**
  ******************************************************************************
  * 3508电机搭配C620电调参数：
	* 使用环境温度：0°C~50°C
	* 减速比：      3591/187 = 157312.68/8192
  ******************************************************************************
  */


#ifndef __6020_Motor_H
#define __6020_Motor_H

#include "stm32f4xx_hal.h"
//#include "pid.h"
#include "drv_can.h"

#define MOTOR_6020_SENT_ID 0x1FF
#define _DEV_OFFLINE        0
#define _DEV_ONLINE         1
#define OFFLINE_TIME_MAX   25





typedef struct
{
	int16_t   angle;            //电机角度（0~8191）
	int16_t   speed;            //电机转速（RPM）（max = 7600）
	int16_t   current;          //转矩电流-16384~16384（-20~20A）
	int16_t   temperature;      //°C
  short     bmi_speed;
	int16_t   angle_add;        //-4096~4096
	int32_t   angle_sum;        //-2147683647~2147683647
	int32_t   target_angle_sum; //目标角度，云台电机
	int32_t   target_speed;     //目标转速
}motor_6020_base_info_t; 

typedef struct 
{
	uint8_t          offline_cnt;
	uint8_t          offline_cnt_max;
	uint8_t          status;
}motor_6020_info_t;

typedef struct motor_6020_t
{
	motor_6020_base_info_t *base_info;
	motor_6020_info_t      *info;
	int16_t                output_current;
	drv_can_t              *can;
	void                  (*init)(struct motor_6020_t *motor);
	void                  (*update)(struct motor_6020_t *motor, uint8_t *rxBuf);
	void                  (*ctrl)(struct motor_6020_t *motor);
}motor_6020_t;


void MOTOR_6020_INIT(motor_6020_t *motor,\
										 motor_6020_base_info_t* base,\
										 motor_6020_info_t* info);

void MOTOR_6020_GET_DATA(motor_6020_t *motor);
HAL_StatusTypeDef MOTOR_6020_CAN_SENT_DATA( int16_t data_1,int16_t data_2,int16_t data_3,int16_t data_4);

#endif
