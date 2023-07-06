/**
  ******************************************************************************
  * @file           : pid_feedback_system.c/h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
	
#include "pid_feedback_system.h"
#include "math_support.h"

/* 
pid_feedback_system_config_t example_pid_feedback_system_config = 
{
	.target = ,
};
pid_feedback_system_base_info_t example_pid_feedback_system_base_info;
pid_feedback_system_info_t example_pid_feedback_system_info;
pid_feedback_system_t example_pid_feedback_system = 
{
	.config = &example_pid_feedback_system_config,
	.base_info = &example_pid_feedback_system_base_info,
	.info = &example_pid_feedback_system_info,
};
*/

void pid_feedback_system_init(pid_feedback_system_t *sys)
{
	pid_feedback_system_base_info_init(sys->base_info);
	sys->base_info->value_status_last = sys->config->measure;
	sys->info->jump_times = 0;
	sys->info->max = 0;
}

void pid_feedback_system_base_info_init(pid_feedback_system_base_info_t *info)
{
	info->cnt = 0;
	info->finish_time = 0;
}

void pid_feedback_system_ready(pid_feedback_system_t *sys,int16_t target,int16_t measure)
{
	sys->config->target = target;
	sys->config->measure = measure;
	pid_feedback_system_init(sys);
	sys->info->status = work_P;
	
}

void pid_feedback_system_update(pid_feedback_system_t *sys, float value)
{
	pid_feedback_system_base_info_t *info = sys->base_info;
	info->cnt++;
	switch(sys->info->status)
	{
		case init_P:
			info->cnt = 0;
			break;
		case work_P:
			if(info->cnt == 1)
			{
				info->value_last = value;
			}
			else if(info->cnt == 2)
			{
				info->value_last = info->value_new;
				info->value_new = value;
				pid_feedback_system_status_update(sys);
			}
			else 
			{
				info->value_last = info->value_new;
				info->value_new = value;
				pid_feedback_system_status_update(sys);
				pid_feedback_system_jump_update(sys);
			}
			break;
		case finish_P:
			sys->info->status = init_P;
			break;
		default:
			break;
	}
}

void pid_feedback_system_status_update(pid_feedback_system_t *sys)
{
	pid_feedback_system_base_info_t *info = sys->base_info;
	info->status_last = info->status_new;
	if(info->value_new < info->value_last)
	{
		info->status_new = down_P;
	}
	else if(info->value_new > info->value_last)
	{
		info->status_new = up_P;
	}
}

void pid_feedback_system_jump_update(pid_feedback_system_t *sys)
{
	pid_feedback_system_info_t *info = sys->info;
	pid_feedback_system_base_info_t *base_info = sys->base_info;
	pid_feedback_system_config_t *config = sys->config;
	
	/* 调节时间重置 */
	if(abs(base_info->value_new - config->target) > abs(config->target - config->measure) * 0.02)
	{
		info->jump_times += sys->base_info->finish_time;
		sys->base_info->finish_time = 0;
	}
	
	/* 极值点 */
	if(base_info->status_new != base_info->status_last)
	{
		/* 超调量 */
		if(abs(base_info->value_new - config->target) > info->max)
		{
			info->max = abs(base_info->value_new - config->target);
		}
		/* 调节时间 */
		if(abs(base_info->value_new - config->target) < abs(config->target - config->measure) * 0.02)
		{
			sys->base_info->finish_time++;
			if(sys->base_info->finish_time >= 2)
			{
				sys->info->status = finish_P;
				info->cnt_sum = base_info->cnt;
				info->max_per = info->max * 100.f / (config->target - config->measure);
			}
		}
		else if(abs(base_info->value_new - base_info->value_status_last) > abs(config->target - config->measure) * 0.02)
		{
			info->jump_times++;
			base_info->value_status_last = base_info->value_new;
		}
	}
}
