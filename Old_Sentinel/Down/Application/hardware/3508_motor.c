/**
  ******************************************************************************
  * @file           : 3508_motor.c\h
  * @brief          : 
  * @update         : 2022Äê1ÔÂ15ÈÕ13:25:49
  ******************************************************************************
  */

#include "3508_motor.h"
#include "config.h"
#include "rp_math.h"
#include "string.h"

/*
motor_3508_base_info_t example;
motor_3508_info_t example;
motor_3508_t example = 
{
	.base_info = &example,
	.info = &example,
	.pid_speed = &example,
	.pid_angle = &example,
	.can = &example,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_angle_ctrl,
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
void motor_3508_init(motor_3508_t *motor)
{
	memset(motor->base_info,0,sizeof(motor_3508_base_info_t));
	motor_3508_info_init(motor->info);
	if(motor->pid_angle != NULL)
	{
		pid_init(motor->pid_angle);
	}
	if(motor->pid_speed != NULL)
	{
		pid_init(motor->pid_speed);
	}
	
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_3508_info_init(motor_3508_info_t *info)
{
	info->offline_cnt_max = 5;
	info->offline_cnt = 5;
	info->status = DEV_OFFLINE;
	info->target_angle_sum = 0;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_3508_update(motor_3508_t *motor, uint8_t *rxBuf)
{
	motor_3508_base_info_t *base_info = motor->base_info;
	int16_t last_angle = base_info->angle;
	
	/* information */
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
	
	/* angle_add */
	base_info->angle_add = base_info->angle - last_angle;
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

	/* flag */
	motor->info->offline_cnt = 0;
	motor->info->status = DEV_ONLINE;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_3508_speed_ctrl(motor_3508_t *motor)
{
	int16_t output;
	motor->pid_speed->info->measure = motor->base_info->speed;
	single_pid_cal(motor->pid_speed->info);
	output = motor->pid_speed->info->out;
	if(motor->can->hcan == &hcan1)
	{
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
void motor_3508_angle_ctrl(motor_3508_t *motor)
{
	motor->pid_angle->info->target = motor->info->target_angle_sum;
	motor->pid_angle->info->measure = motor->base_info->angle_sum;
	single_pid_cal(motor->pid_angle->info);
	motor->pid_speed->info->target = motor->pid_angle->info->out;
	motor_3508_speed_ctrl(motor);
}
