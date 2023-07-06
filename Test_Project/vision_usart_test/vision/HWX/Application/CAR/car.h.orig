#ifndef __CAR_H
#define __CAR_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* 车行动模式枚举 */
typedef enum 
{
  offline_CAR,        //离线模式
  init_CAR,           //初始化模式
  machine_CAR,        //机械模式
  high_shoot_CAR,     //吊射模式
	test_CAR,
	follow_CAR,
	spin_CAR,           //小陀螺模式
}car_move_mode_e;

/* 车控制模式枚举 */
typedef enum 
{
  RC_CAR,             //遥控器控制
  KEY_CAR,            //键盘控制
}car_ctrl_mode_e;

/* 车配置枚举 */
typedef struct 
{
  uint8_t reserve;
}car_config_t;


typedef struct 
{
	car_config_t *config;  //整车配置
	
  uint8_t move_mode_commond;  //移动模式命令
  uint8_t move_mode_status;   //移动模式状态
//	uint8_t last_move_mode_status;
  uint8_t ctrl_mode;          //控制模式
	uint8_t last_mode;
}car_t;





void MODE_CHECK();



#define PI (3.1415926)



#endif
