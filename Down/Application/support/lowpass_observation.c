/**
  ******************************************************************************
  * @file           : lowpass_observation.c\h
  * @brief          : 
  * @note           : 2022-1-21 22:04:05
  ******************************************************************************
  */

#include "lowpass_observation.h"
#include "rp_math.h"

void lp_obs_init(lp_obs_t *obs)
{
	obs->value_sum = 0;
	obs->value_sum_abs = 0;
	obs->value_lp_sum = 0;
	obs->value_lp_sum_abs = 0;
	obs->diff_abs = 0;
	obs->diff = 0;
}

void lp_obs_update(lp_obs_t *obs, float value, float value_lp)
{
	obs->value_sum -= obs->err[obs->index];
	obs->value_sum_abs -= abs(obs->err[obs->index]);
	obs->value_lp_sum -= obs->err_lp[obs->index];
	obs->value_lp_sum_abs -= abs(obs->err_lp[obs->index]);
	
	obs->err[obs->index] = value - obs->value_last;
	obs->err_lp[obs->index] = value_lp - obs->value_lp_last;
	obs->value_last = value;
	obs->value_lp_last = value_lp;

	obs->value_sum += obs->err[obs->index];
	obs->value_sum_abs += abs(obs->err[obs->index]);
	obs->value_lp_sum += obs->err_lp[obs->index];
	obs->value_lp_sum_abs += abs(obs->err_lp[obs->index]);
	
	obs->diff_abs = obs->value_sum_abs / obs->value_lp_sum_abs;
	obs->diff = (obs->value_sum - obs->value_lp_sum)/5000.f;

	obs->index++;
	obs->index %= 5000;
}
