/**
  ******************************************************************************
  * @file           : car.c/h
  * @brief          : 
  * @note           : finish 2022-1-23 14:24:04
  ******************************************************************************
  */

#ifndef __CAR_H
#define __CAR_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "remote.h"

/* 车行动模式枚举 */
typedef enum 
{
  offline_CAR,        //离线模式
  init_CAR,           //初始化模式
  machine_CAR,        //机械模式
  gyro_CAR,           //陀螺仪模式
  fly_slope_CAR,      //飞坡模式
  high_shoot_CAR,     //吊射模式
}car_move_mode_e;

/* 车控制模式枚举 */
typedef enum 
{
  RC_CAR,             //遥控器控制
  KEY_CAR,            //键盘控制
}car_ctrl_mode_e;

typedef enum
{
	normal,  //0
	yaw_motor_angle_move_debug,  //1
	yaw_imu_angle_move_debug,  //2
	yaw_motor_angle_static_debug,  //3
	yaw_imu_angle_static_debug,  //4
	yaw_motor_speed_static_debug,  //5
	yaw_imu_speed_static_debug,  //6
	pitch_motor_angle_move_debug,  //7
	pitch_imu_angle_move_debug,  //8
	pitch_motor_angle_static_debug,  //9
	pitch_imu_angle_static_debug,  //10
	pitch_motor_speed_static_debug,  //11
	pitch_imu_speed_static_debug,  //12
}control_task_mode_e;

typedef struct 
{
	uint8_t control_task_mode;  //控制任务模式（设置控制任务内容）
}car_config_t;

typedef struct 
{
	car_config_t *config;  //整车配置
	
  uint8_t move_mode_commond;  //移动模式命令
  uint8_t move_mode_status;   //移动模式状态
  uint8_t ctrl_mode;          //控制模式
}car_t;

/* 外部变量 */
extern bool car_mode_change;
extern bool chassis_cycle_off;
extern bool chassis_cycle_on;
extern bool chassis_cycle_change;
extern bool gimbal_l_90;
extern bool gimbal_l_shoot;
extern bool gimbal_r_90;
extern bool gimbal_r_shoot;
extern bool gimbal_r_180;
extern bool gimbal_u_shoot;
extern bool gimbal_d_shoot;
extern bool Launcher_Shoot;
extern bool Launcher_Relax;
extern bool Launcher_Stop;
extern bool dial_reload;
extern bool dial_unload;
extern bool auto_on;
extern bool auto_off;
extern bool judge_close;

extern car_t car;

/* 初始化 */
void car_init(car_t *car);
void car_commond_init(car_t *car);

/* 控制任务 */
void car_ctrl(car_t *car);
void car_init_judge(void);
void car_mode_commond_update(car_t *car);
void car_mode_status_update(car_t *car);

/* 遥控器键位扫描 */
void RC_status_scan(car_t *car);
/* 遥控器键位状态任务 */
void RC_s1_status_check(car_t *car);
void RC_s2_status_check(car_t *car);
void RC_thumbwheel_status_check(car_t *car);

/* 键盘按键扫描 */
void KEY_status_scan(car_t *car);
/* 键盘按键状态任务 */
void KEY_mouse_l_status_check(car_t *car);
void KEY_mouse_r_status_check(car_t *car);
void KEY_Q_status_check(car_t *car);
void KEY_W_status_check(car_t *car);
void KEY_E_status_check(car_t *car);
void KEY_R_status_check(car_t *car);
void KEY_A_status_check(car_t *car);
void KEY_S_status_check(car_t *car);
void KEY_D_status_check(car_t *car);
void KEY_F_status_check(car_t *car);
void KEY_G_status_check(car_t *car);
void KEY_Z_status_check(car_t *car);
void KEY_X_status_check(car_t *car);
void KEY_C_status_check(car_t *car);
void KEY_V_status_check(car_t *car);
void KEY_B_status_check(car_t *car);
void KEY_Shift_status_check(car_t *car);
void KEY_Ctrl_status_check(car_t *car);

#endif
