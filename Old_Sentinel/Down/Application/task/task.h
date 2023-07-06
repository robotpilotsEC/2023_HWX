/**
  ******************************************************************************
  * @file           : task.c\h
  * @brief          : 
  * @note           : 2022-1-18 12:51:24
  ******************************************************************************
  */

#ifndef __TASK_H
#define __TASK_H

#include "timer.h"

timer_info_t control_task_normal_timer_info;
timer_t      control_task_normal_timer = 
{
	.info = &control_task_normal_timer_info,
};

timer_info_t realtime_task_normal_timer_info;
timer_t      realtime_task_normal_timer = 
{
	.info = &realtime_task_normal_timer_info,
};

timer_info_t control_task_cycle_timer_info;
timer_t      control_task_cycle_timer = 
{
	.info = &control_task_cycle_timer_info,
};

timer_info_t realtime_task_cycle_timer_info;
timer_t      realtime_task_cycle_timer = 
{
	.info = &realtime_task_cycle_timer_info,
};

#endif
