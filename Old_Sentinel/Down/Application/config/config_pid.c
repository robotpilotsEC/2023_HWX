/**
  ******************************************************************************
  * @file           : config_pid.c\h
  * @brief          : 
  * @note           : 2022年1月15日13:32:03
  ******************************************************************************
  */

#include "config_pid.h"
#include "config.h"

/* 
pid_info_t example =
{
	.kp = 0,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.out_max = 0,
};
*/

/* 底盘电机pid信息 */
pid_info_t chassis_motor_pid_info[4] = 
{
	[LF] = 
	{
		.kp = 8,
		.ki = 0.33,
		.out_max = 12500,
		.integral_max = 6000,
	},
	[RF] = 
	{
		.kp = 8,
		.ki = 0.33,
		.out_max = 12500,
		.integral_max = 6000,
	},
	[LB] = 
	{
		.kp = 8,
		.ki = 0.33,
		.out_max = 12500,
		.integral_max = 6000,
	},
	[RB] = 
	{
		.kp = 8,
		.ki = 0.33,
		.out_max = 12500,
		.integral_max = 6000,
	},
};

/* 云台Yaw电机pid速度环信息 */
pid_info_t gimbal_motor_yaw_pid_speed_info = 
{
	.kp = -500,
	.ki = -7,
	.kd = 0,
	.out_max = 28000,
	.integral_max = 4000,
};
	
/* 云台Yaw陀螺仪pid速度环信息 */
pid_info_t gimbal_imu_yaw_pid_speed_info = 
{
	.kp = -550,
	.ki = -3,
	.kd = 0,
	.out_max = 28000,
	.integral_max = 4000,
};

/* 云台Yaw电机pid位置环信息 */
pid_info_t gimbal_motor_yaw_pid_angle_info = 
{
	.kp = 0.75,
	.ki = 0,
	.kd = 0,
	.out_max = 700,
	.integral_max = 0,
};

/* 云台Yaw陀螺仪pid位置环信息 */
pid_info_t gimbal_imu_yaw_pid_angle_info = 
{
	.kp = 13,
	.ki = 0,
	.kd = 0,
	.out_max = 500,
	.integral_max = 0,
};

/* 云台Pitch电机pid速度环信息 */
pid_info_t gimbal_motor_pitch_pid_speed_info = 
{
	.kp = -500,//-500
	.ki = -5,//-5
	.kd = 0,
	.out_max = 28000,
	.integral_bias = -2800,//-2800
	.integral_max = 5000,//3000
};

/* 云台Pitch陀螺仪pid速度环信息 */
pid_info_t gimbal_imu_pitch_pid_speed_info = 
{
	.kp = -450,//
	.ki = -1,//
	.kd = -1500,//
	.out_max = 28000,
	.integral_bias = -3000,//
	.integral_max = 5000,//
};

/* 云台Pitch电机pid位置环信息 */
pid_info_t gimbal_motor_pitch_pid_angle_info = 
{
	.kp = 1,//16
	.ki = 0,
	.out_max = 500,
	.integral_max = 0,
};

/* 云台Pitch陀螺仪pid位置环信息 */
pid_info_t gimbal_imu_pitch_pid_angle_info = 
{
	.kp = 15,//16
	.ki = 0,
	.out_max = 500,
	.integral_max = 0,
};

/* 拨盘电机pid位置环信息 */
pid_info_t dial_motor_pid_angle_info = 
{
	.kp = 0.2,
	.out_max = 3000,  //6000
};

/* 拨盘电机pid速度环信息 */
pid_info_t dial_motor_pid_speed_info = 
{
	.kp = 10,
	.ki = 0.4,
	.integral_max = 10000,
	.out_max = 0,
};

/* 摩擦轮左电机pid速度环信息 */
pid_info_t friction_left_motor_pid_speed_info = 
{
	.kp = 8,
	.ki = 0.33,
	.integral_max = 6000,
	.out_max = 10000,
};

/* 摩擦轮右电机pid速度环信息 */
pid_info_t friction_right_motor_pid_speed_info = 
{
	.kp = 8,
	.ki = 0.33,
	.integral_max = 6000,
	.out_max = 10000,
};

/* 限位轮电机pid速度环信息 */
pid_info_t position_motor_pid_speed_info = 
{
	.kp = 15,
	.ki = 0.4,
	.integral_max = 10000,
	.out_max = 9000,
};
