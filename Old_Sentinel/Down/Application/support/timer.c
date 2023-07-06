/**
  ******************************************************************************
  * @file           : timer.c/h
  * @brief          : 
  * @note           : 2022-1-17 22:23:08
  ******************************************************************************
  */

#include "timer.h"
#include "string.h"

/*
timer_info_t example_timer_info;
timer_t      example_task_timer = 
{
	.info = &example_timer_info,
};
*/

extern TIM_HandleTypeDef htim2;

void timer_init(timer_t *timer)
{
	memset(timer->info,0,sizeof(timer_info_t));
}

void timer_start(timer_t *timer)
{
	timer_info_t *info = timer->info;
	info->start_time_ms = HAL_GetTick();
	info->start_time_us = __HAL_TIM_GET_COUNTER(&htim2);
}

void timer_end(timer_t *timer)
{
	timer_info_t *info = timer->info;
	info->end_time_ms = HAL_GetTick();
	info->end_time_us = __HAL_TIM_GET_COUNTER(&htim2);
	info->duration_ms = info->end_time_ms - info->start_time_ms;
	info->duration_us = info->duration_ms * 1000 + info->end_time_us - info->start_time_us;
}

void timer_cycle(timer_t *timer)
{
	timer_info_t *info = timer->info;
	timer_end(timer);
	info->start_time_ms = info->end_time_ms;
	info->start_time_us = info->end_time_us;
}
