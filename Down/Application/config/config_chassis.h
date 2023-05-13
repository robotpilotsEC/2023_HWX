/**
  ******************************************************************************
  * @file           : config_chassis.c\h
  * @brief          : 
  * @note           : 
	*                   2022-3-17 增加小陀螺模式下转速模式项初始化值
	*                   2022-3-17 增加底盘速度target变化模式项初始化值
  ******************************************************************************
  */

#ifndef __CONFIG_CHASSIS_H
#define __CONFIG_CHASSIS_H

#define chassis_cycle_T 1000    //底盘小陀螺周期
#define chassis_speed_max 8000  //底盘最高速度
#define chassis_back_time 2000  //陀螺仪模式->机械模式的底盘最大归位时间

#define chassis_cycle_speed_mode_init 0
#define chassis_target_speed_mode_init 0

#endif
