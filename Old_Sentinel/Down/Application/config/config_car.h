/**
  ******************************************************************************
  * @file           : config_car.c\h
  * @brief          : 
  * @note           : 2022-2-7 15:45:47
  ******************************************************************************
  */

#ifndef __CONFIG_CAR_H
#define __CONFIG_CAR_H

#include "stm32f4xx_hal.h"

//#define CHASSIS_OFF           //关闭底盘
//#define GIMBAL_OFF            //关闭云台
//#define LAUNCHER_OFF          //关闭发射机构
#define VISION_OFF            //关闭视觉
//#define ALL_MOTOR_CLOSE       //关闭所有电机（can不发送）

#define REMOTE_OFFLINE_CHECK  //开启遥控器失联检测

#define control_task_mode_init normal  //控制模式任务模式初始化值

#endif
