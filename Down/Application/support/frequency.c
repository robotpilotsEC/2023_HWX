/**
  ******************************************************************************
  * @file           : frequency.c\h
  * @brief          : 
  * @note           : 2022-1-21 22:04:05
  ******************************************************************************
  */

#include "frequency.h"

void frequency_init(frequency_t *fre)
{
	fre->jump_times = 0;
	fre->cnt = 0;
	fre->Hz = 0;
	fre->value_last = 0;
	fre->value_new = 0;
}

void frequency_update(frequency_t *fre, float value)
{
	if(fre->cnt == 0)
	{
		fre->value_new = value;
	}
	else 
	{
		fre->value_last = fre->value_new;
		fre->value_new = value;
		if(fre->cnt == 1)
		{
			frequency_judge(fre);
		}
		else
		{
			fre->status_last = fre->status_new;
			frequency_judge(fre);
			frequency_calculate(fre);
		}
	}
	fre->cnt++;
}

void frequency_judge(frequency_t *fre)
{
	if(fre->value_new > fre->value_last)
	{
		fre->status_new = up_F;
	}
	else if(fre->value_new < fre->value_last)
	{
		fre->status_new = down_F;
	}
	else 
	{
		fre->status_new = fre->status_last;
	}
}

void frequency_calculate(frequency_t *fre)
{
	if(fre->status_new != fre->status_last)
	{
		fre->jump_times ++;
		if(fre->jump_times == 1)
		{
			fre->cnt_sum = fre->cnt;
			fre->cnt = 0;
		}
		else if(fre->jump_times == 2)
		{
			fre->cnt_sum += fre->cnt;
			fre->jump_times = 0;
			fre->Hz = 1000.f / (float)(fre->cnt_sum);
			fre->Hz_ave -= fre->Hz_sum[fre->index] / 100.f;
			fre->Hz_sum[fre->index] = fre->Hz;
			fre->Hz_ave += fre->Hz_sum[fre->index] / 100.f;
			fre->index++;
			fre->index %= 100;
		}
	}
}
