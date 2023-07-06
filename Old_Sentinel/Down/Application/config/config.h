#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f4xx_hal.h"

/******************************全局使用枚举******************************/

typedef enum DEV_WORK_STATE
{
	DEV_OFFLINE,
	DEV_ONLINE,
}Dev_Work_State;

/* 状态灯枚举 */
typedef enum LED_NUM
{
	LED_ORANGE,            //
	LED_Blue,              //模式状态灯6失联5初始化4飞坡3吊射2陀螺仪1机械
	LED_RED,
	LED_GREEN,
}LED_Num;

typedef enum
{
	Light_t = 150,//250
	Interval_t = 200,//300
	Wait_t = 500,
}LED_time;

typedef enum
{
	LF,
	RF,
	LB,
	RB,
}motor_num;

#endif
