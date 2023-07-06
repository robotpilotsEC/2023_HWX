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




#ifndef __6020_Motor_H
#define __6020_Motor_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "drv_can.h"

/* Private macro -------------------------------------------------------------*/
#define MOTOR_6020_SENT_ID 0x1FF
#define _DEV_OFFLINE        0
#define _DEV_ONLINE         1
#define OFFLINE_TIME_MAX   25


/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
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
	void                  (*init)(struct motor_6020_t *motor);
	void                  (*update)(struct motor_6020_t *motor, uint8_t *rxBuf);
	void                  (*ctrl)(struct motor_6020_t *motor);
}motor_6020_t;

/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Function  body --------------------------------------------------------*/
void Motor_6020_Init(motor_6020_t *motor,motor_6020_base_info_t* base,motor_6020_info_t* info);
void Motor_6020_GetData(motor_6020_t *motor);
void Motor_6020_HeartBeat(motor_6020_t *motor);
HAL_StatusTypeDef Motor_6020_CanSentData( int16_t data_1,int16_t data_2,int16_t data_3,int16_t data_4);

#endif
