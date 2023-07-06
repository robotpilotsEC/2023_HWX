/**
  ******************************************************************************
  * @file           : 6020_motor.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#include "6020_motor.h"
#include "rp_math.h"
#include "string.h"

/*
motor_6020_base_info_t example;
motor_6020_info_t example;
motor_6020_t example = 
{
	.base_info = &example,
	.info = &example,
	.pid_speed = &example,
	.motor_pid_angle = &example,
	.imu_pid_angle = &example,
	.can = &example,
	.init = motor_6020_init,
	.update = motor_6020_update,
};
*/

extern uint8_t can1_tx_buf[16];
extern uint8_t can2_tx_buf[16];
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void motor_6020_init(motor_6020_t *motor)
{
	memset(motor->base_info,0,sizeof(motor_6020_base_info_t));
	motor_6020_info_init(motor->info);
	if(motor->motor_pid_speed != NULL)
	{
		pid_init(motor->motor_pid_speed);
	}
	if(motor->imu_pid_speed != NULL)
	{
		pid_init(motor->imu_pid_speed);
	}
	if(motor->motor_pid_angle != NULL)
	{
		pid_init(motor->motor_pid_angle);
	}
	if(motor->imu_pid_angle != NULL)
	{
		pid_init(motor->imu_pid_angle);
	}
}

void motor_6020_info_init(motor_6020_info_t *info)
{
	info->offline_cnt = 5;
	info->offline_cnt_max = 5;
	info->status = DEV_OFFLINE;
	info->target_angle_sum = 0;
}

void motor_6020_update(motor_6020_t *motor, uint8_t *rxBuf)
{
	motor_6020_base_info_t *base_info = motor->base_info;
	uint16_t angle_last = base_info->angle;
	
	base_info->angle = rxBuf[0];
	base_info->angle <<= 8;
	base_info->angle |= rxBuf[1];
	base_info->speed = rxBuf[2];
	base_info->speed <<= 8;
	base_info->speed |= rxBuf[3];
	base_info->current = rxBuf[4];
	base_info->current <<= 8;
	base_info->current |= rxBuf[5];
	base_info->temperature = rxBuf[6];
	
	/* calculate anglar difference */
	base_info->angle_add = base_info->angle - angle_last;
	if(abs(base_info->angle_add) > 4096)
	{
		base_info->angle_add -= 8192 * one(base_info->angle_add);
	}
	
	/* angle_sum and target_angle_sum */
	base_info->angle_sum += base_info->angle_add;
	if(abs(base_info->angle_sum) > 0x0FFF)
	{
		base_info->angle_sum -= 0x0FFF * one(base_info->angle_sum);
		motor->info->target_angle_sum -= 0x0FFF * one(base_info->angle_sum);
	}
	
	motor->info->offline_cnt = 0;
	motor->info->status = DEV_ONLINE;
}
