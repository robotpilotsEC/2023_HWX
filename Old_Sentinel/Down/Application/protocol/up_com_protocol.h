/**
  ******************************************************************************
  * @file           : up_com_protocol.c\h
	* @author         : czf
	* @date           : 2022-4-22 20:16:29
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "stm32f4xx_hal.h"

void up_send_power_heat(void);
void up_send_game_robot_state(void);
void up_send_shoot(void);
void up_send_robot_pos(void);
void up_send_robot_hurt(void);
void up_send_robot_HP(void);