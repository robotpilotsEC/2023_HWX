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

/**
******************************************************************************
* 3508电机搭配C620电调参数：
* 使用环境温度：0°C~50°C
* 减速比：      3591/187 = 157312.68/8192
******************************************************************************
*/


#ifndef __3508_Motor_H
#define __3508_Motor_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "drv_can.h"

/* Private macro -------------------------------------------------------------*/
#define MOTOR_3508_SENT_ID 0x200
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
	int16_t   target_prm; 			//目标转速
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

/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Function  body --------------------------------------------------------*/
void Motor_3508_Init(motor_3508_t *motor,motor_3508_base_info_t *base,motor_3508_info_t *info);

void Motor_3508_GetData1(motor_3508_t *motor);
void Motor_3508_GetData2(motor_3508_t *motor);

HAL_StatusTypeDef Motor_3508_Can1SentData( uint16_t data_1,uint16_t data_2,uint16_t data_3,uint16_t data_4);
HAL_StatusTypeDef Motor_3508_Can2SentData( uint16_t data_1,uint16_t data_2,uint16_t data_3,uint16_t data_4);

void Motor_3508_HeartBeat(motor_3508_t *motor);

#endif

