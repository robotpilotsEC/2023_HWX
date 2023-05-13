/**
  ******************************************************************************
  * @file           : debug.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
	
#include "debug.h"
#include "math.h"
#include "math_support.h"
#include "6020_motor.h"
#include "math_support.h"

extern uint8_t can1_tx_buf[16];
extern CAN_HandleTypeDef hcan1;
motor_6020_base_info_t gimbal_motor_yaw_base_info;
motor_6020_info_t gimbal_motor_yaw_info;
pid_info_t gimbal_motor_yaw_pid_speed_info = 
{
	.kp = 0,//-500,
	.ki = 0,//-7,
	.kd = 0,
	.out_max = 28000,
	.integral_max = 4000,
};
pid_t gimbal_motor_yaw_speed_pid = 
{
	.info = &gimbal_motor_yaw_pid_speed_info,
};
pid_info_t gimbal_motor_yaw_pid_angle_info = 
{
	.kp = 0,//0.75,
	.ki = 0,
	.kd = 0,
	.out_max = 700,
	.integral_max = 0,
};
pid_t gimbal_motor_yaw_angle_pid = 
{
	.info = &gimbal_motor_yaw_pid_angle_info,
};
drv_can_t gimbal_motor_yaw_can = 
{
	.hcan = &hcan1,
	.rx_id = 0x20A,
};
motor_6020_t gimbal_yaw_motor = 
{
	.base_info = &gimbal_motor_yaw_base_info,
	.info = &gimbal_motor_yaw_info,
	.motor_pid_speed = &gimbal_motor_yaw_speed_pid,
	.motor_pid_angle = &gimbal_motor_yaw_angle_pid,
	.can = &gimbal_motor_yaw_can,
	.init = motor_6020_init,
	.update = motor_6020_update,
};

void gimbal_yaw_motor_pid_angle_ctrl(void)
{
	pid_info_t *pid = gimbal_yaw_motor.motor_pid_angle->info;
	
	float target = 0;
	float measure = gimbal_yaw_motor.base_info->angle;
	float err = target - measure;
	
	//+-180¡ãÌø±ä¼ì²â
	if(abs(err) > 4096)
	{
		measure -= 8192 * sgn(measure);
	}
	
	pid->target = target;
	pid->measure = measure;
	single_pid_cal(pid);
	
	gimbal_yaw_motor.motor_pid_speed->info->target = pid->out;
	gimbal_yaw_motor_pid_speed_ctrl();
}

void gimbal_yaw_motor_pid_speed_ctrl(void)
{
	pid_info_t *pid = gimbal_yaw_motor.motor_pid_speed->info;
	
	int16_t output;
	
	pid->measure = gimbal_yaw_motor.base_info->speed;
	single_pid_cal(pid);
	output = pid->out;
	can1_tx_buf[(0x205 - 0x201) * 2] = (output >> 8) & 0xFF;
	can1_tx_buf[(0x205 - 0x201) * 2 + 1] = (output & 0xFF);
}
