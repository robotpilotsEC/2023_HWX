/**
  ******************************************************************************
  * @file           : cap.c\h
	* @author         : czf
	* @date           : 2022.4.28
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#ifndef __CAP_H
#define __CAP_H

#include "stm32f4xx_hal.h"

typedef struct
{
	uint8_t Y_O_N;
	uint8_t record_Y_O_N;
}cap_t;

extern cap_t cap;

#endif
