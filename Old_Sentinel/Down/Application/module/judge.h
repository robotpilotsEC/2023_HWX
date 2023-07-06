/**
  ******************************************************************************
  * @file           : judge.c\h
	* @author         : czf
	* @date           : 2022-4-22 14:39:55
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#ifndef __JUDGE_H
#define __JUDGE_H

#include "stm32f4xx_hal.h"
#include "judge_protocol.h"

typedef struct 
{
	ext_rfid_status_t       rfid_status;
	ext_game_status_t       game_status;
	ext_game_robot_status_t game_robot_status;
	ext_power_heat_data_t   power_heat_data;
	ext_shoot_data_t        shoot_data;
	ext_game_robot_pos_t    game_robot_pos;
	ext_robot_hurt_t        ext_robot_hurt;
	ext_aerial_data_t       ext_aerial_data;
	ext_robot_command_t     ext_robot_command;
	ext_game_robot_HP_t     game_robot_HP;
	ext_bullet_remaining_t  bullet_remaining;
	
}judge_info_t;

typedef struct 
{
	judge_info_t *info;
	
}judge_t;

extern judge_t judge;

void Send_Can(void);

#endif
