/**
  ******************************************************************************
  * @file           : timer.c/h
  * @brief          : 
  * @note           : 2022-1-17 22:23:13
  ******************************************************************************
  */

#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx_hal.h"

typedef struct 
{
	uint32_t start_time_us;  //开始时间(us)
	uint32_t start_time_ms;  //开始时间(ms)
	uint32_t end_time_us;    //结束时间(us)
	uint32_t end_time_ms;    //结束时间(ms)
	uint32_t duration_us;    //持续时间(us)
	uint32_t duration_ms;    //持续时间(ms)
}timer_info_t;

typedef struct 
{
	timer_info_t *info;
}timer_t;

void timer_init(timer_t *timer);
void timer_start(timer_t *timer);
void timer_end(timer_t *timer);
void timer_cycle(timer_t *timer);

#endif
