/**
  ******************************************************************************
  * @file           : 2006_motor.c\h
  * @brief          : 
  * @update         : 2022年1月15日12:34:34
  ******************************************************************************
  */

#include "2006_motor.h"
#include "rp_math.h"
#include "config.h"
#include "string.h"

/*
motor_2006_base_info_t example;
motor_2006_info_t example;
motor_2006_t example = 
{
	.base_info = &example,
	.info = &example,
	.pid_speed = &example,
	.pid_angle = &example,
	.can = &example,
	.init = motor_2006_init,
	.update = motor_2006_update,
	.ctrl = motor_2006_angle_ctrl,
};
*/

extern uint8_t can1_tx_buf[16];
extern uint8_t can2_tx_buf[16];
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_init(motor_2006_t *motor)
{
	memset(motor->base_info,0,sizeof(motor_2006_base_info_t));
	motor_2006_info_init(motor->info);
	if(motor->pid_speed != NULL)
	{
		pid_init(motor->pid_speed);
	}
	if(motor->pid_angle != NULL)
	{
		pid_init(motor->pid_angle);
	}
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_info_init(motor_2006_info_t *info)
{
	info->offline_cnt_max = 5;
	info->target_angle_sum = 0;
	info->offline_cnt = 5;
	info->status = DEV_OFFLINE;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_update(motor_2006_t *motor, uint8_t *rxBuf)
{
	motor_2006_base_info_t *base_info = motor->base_info;
	int16_t last_angle = base_info->angle;
	
	/* 电调返回信息 */
	base_info->angle = rxBuf[0];
	base_info->angle <<= 8;
	base_info->angle |= rxBuf[1];
	base_info->speed = rxBuf[2];
	base_info->speed <<= 8;
	base_info->speed |= rxBuf[3];
	base_info->torque = rxBuf[4];
	base_info->torque <<= 8;
	base_info->torque |= rxBuf[5];
	
	/* angle_add */
	base_info->angle_add = base_info->angle - last_angle;
	if(abs(base_info->angle_add) > 4096)
	{
		base_info->angle_add -= 8192 * one(base_info->angle_add);
	}
	
	/* angle_sum and target_angle_sum */
	base_info->angle_sum += base_info->angle_add;
	if(abs(base_info->angle_sum) > 0x0FFFFFFF)
	{
		base_info->angle_sum -= 0x0FFFFFFF * one(base_info->angle_sum);
		motor->info->target_angle_sum -= 0x0FFFFFFF * one(base_info->angle_sum);
	}

	/* flag */
	motor->info->offline_cnt = 0;
	motor->info->status = DEV_ONLINE;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_speed_ctrl(motor_2006_t *motor)
{
	int16_t output;
	motor->pid_speed->info->measure = motor->base_info->speed;
	single_pid_cal(motor->pid_speed->info);
	output = motor->pid_speed->info->out;
	if(motor->can->hcan == &hcan1){
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(motor->can->hcan == &hcan2){
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_angle_ctrl(motor_2006_t *motor)
{
	motor->pid_angle->info->target = motor->info->target_angle_sum;
	motor->pid_angle->info->measure = motor->base_info->angle_sum;
	single_pid_cal(motor->pid_angle->info);
	motor->pid_speed->info->target = motor->pid_angle->info->out;
	motor_2006_speed_ctrl(motor);
}
