/**
  ******************************************************************************
  * @file           : frequency.c\h
  * @brief          : 
  * @note           : 2022-1-21 22:04:05
  ******************************************************************************
  */

#ifndef __FREQUENCY_H
#define __FREQUENCY_H

#include "stm32f4xx_hal.h"

typedef enum
{
	up_F,
	down_F,
}frequency_status_e;

typedef struct
{
	uint8_t    status_last;
	uint8_t    status_new;
	uint8_t    jump_times;
	uint16_t   cnt;
	uint16_t   cnt_sum;
	float      Hz;
	uint16_t   index;
	float      Hz_sum[100];
	float      Hz_ave;
	float      value_last;
	float      value_new;
}frequency_t;

void frequency_init(frequency_t *fre);
void frequency_update(frequency_t *fre, float value);
void frequency_judge(frequency_t *fre);
void frequency_calculate(frequency_t *fre);

#endif
