/**
  ******************************************************************************
  * @file           : lowpass_observation.c\h
  * @brief          : 
  * @note           : 2022-1-21 22:04:05
  ******************************************************************************
  */

#ifndef __LOWPASS_OBSERVATION
#define __LOWPASS_OBSERVATION

#include "stm32f4xx_hal.h"

typedef struct 
{
	uint16_t index;
	float err[5000];
	float err_lp[5000];
	float value_last;
	float value_lp_last;
	float value_sum;
	float value_sum_abs;
	float value_lp_sum;
	float value_lp_sum_abs;
	float diff_abs;
	float diff;
}lp_obs_t;

void lp_obs_init(lp_obs_t *obs);
void lp_obs_update(lp_obs_t *obs, float value, float value_lp);

#endif
