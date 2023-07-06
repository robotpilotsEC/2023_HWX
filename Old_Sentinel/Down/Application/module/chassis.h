/**
  ******************************************************************************
  * @file           : chassis.c/h
  * @brief          : 
  * @note           : finish 2022-2-11 12:33:21
	*                   2022-3-17 增加底盘配置结构体，增加小陀螺模式下转速模式项
	*                             底盘配置结构体，增加底盘速度target变化模式项
  ******************************************************************************
  */

#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "3508_motor.h"

/* 底盘移动模式枚举 */
typedef enum 
{
	C_M_offline, //失联模式
	C_M_stop,    //停止模式
	C_M_normal,  //正常模式（以底盘朝向为正方向）
	C_M_special, //特殊模式（以云台朝向为正方向）
}chassis_move_mode_e;

/* 底盘旋转模式枚举 */
typedef enum 
{
	C_C_offline, //失联模式
	C_C_stop,    //停止模式
	C_C_normal,  //正常模式（可控制）
	C_C_follow,  //跟随模式（可前后颠倒）
	C_C_back,    //归位模式（不可前后颠倒）
	C_C_cycle,   //小陀螺模式
}chassis_cycle_mode_e;

typedef struct
{
	uint8_t chassis_cycle_speed_mode; //小陀螺模式下转速模式 //0匀速转动 //1匀加速转动 //2三角函数式
	uint8_t chassis_target_speed_mode; //底盘target速度变化模式 //0根据按键计数梯形变化 //1单值过微分跟踪器
}chassis_config_t;

/* 底盘信息 */
typedef struct 
{
	int16_t target_front_speed;  //目标前进速度
	int16_t target_right_speed;  //目标左移速度
	int16_t target_cycle_speed;  //目标旋转速度
	uint8_t move_mode;           //移动模式
	uint8_t cycle_mode;          //旋转模式
	int16_t back_cnt;            //归位计数
	int16_t speed_cnt;           //速度函数计数
}chassis_info_t;

/* 底盘 */
typedef struct 
{
	motor_3508_t *motor;
	chassis_config_t *config;
	chassis_info_t *info;
}chassis_t;

/* 外部变量 */
extern chassis_t chassis;

/* 初始化 */
void chassis_init(chassis_t *chassis);
void chassis_info_init(chassis_info_t *info);
void chassis_commond_init(chassis_t *chassis);

/* 控制任务 */
void chassis_ctrl_task(chassis_t *chassis);
void chassis_mode_update(chassis_t *chassis);
void chassis_commond_respond(chassis_t *chassis);
void chassis_work(chassis_t *chassis);
void chassis_judge_limit(chassis_t *chassis);

void motor_chassis_update(chassis_t *chassis);
void motor_speed_limit(chassis_t *chassis, int16_t *front, int16_t *right, int16_t *round);

#endif
