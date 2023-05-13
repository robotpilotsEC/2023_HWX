/**
  ******************************************************************************
  * @file           : config_judge.c\h
	* @author         : czf
	* @date           : 2022-4-22 15:13:20
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#ifndef __CONFG_JUDGE_H
#define __CONFG_JUDGE_H

#include "stm32f4xx_hal.h"

void judge_update(uint16_t id, uint8_t *rxBuf);

#endif
