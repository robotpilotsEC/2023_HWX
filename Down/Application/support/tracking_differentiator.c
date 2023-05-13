/**
  ******************************************************************************
  * @file           : tracking_differentiator.c\h
  * @brief          : 
  * @note           : 2022-2-7 22:53:39
  ******************************************************************************
  */

#include "tracking_differentiator.h"
#include "math_support.h"
#include "rp_math.h"
#include "math.h"

/*
tracking_differentiator_t example_track_tor = 
{
	.info = &example_track_tor_info,
};
*/

void tracking_differentiator_init(tracking_differentiator_t *tor)
{
	tor->info->output_value = 0;
	tor->info->output_differential_value = 0;
}

void tracking_differentiator_update(tracking_differentiator_t *tor, float v)
{
	float x1 = tor->info->output_value;
	float x2 = tor->info->output_differential_value;
	float r = tor->info->r;
	float h0 = tor->info->h0;
	float fh = fhan(x1 - v, x2, r, h0);
	tor->info->output_value = x1 + x2;
	if(tor->info->output_differential_value_max >= 0)
	{
		tor->info->output_differential_value = constrain(x2 + fh, -tor->info->output_differential_value_max, tor->info->output_differential_value_max);
	}
	else 
	{
		tor->info->output_differential_value = x2 + fh;
	}
}

float tracking_differentiator_cal(tracking_differentiator_t *tor, float x1, float x2, float v)
{
	float r = tor->info->r;
	float h0 = tor->info->h0;
	float fh = fhan(x1 - v, x2, r, h0);
	if(tor->info->output_differential_value_max >= 0)
	{
		tor->info->output_differential_value = constrain(x2 + fh, -tor->info->output_differential_value_max, tor->info->output_differential_value_max);
	}
	else
	{
		tor->info->output_differential_value = x2 + fh;
	}
	tor->info->output_value = x1 + tor->info->output_differential_value;
	return tor->info->output_value;
}

float fhan(float x1, float x2, float r, float h)
{
	float d = r * h * h;
	float a0 = h * x2;
	float y = x1 + a0;
	float a1 = sqrt(d * (d + 8 * abs(y)));
	float a2 = a0 + sgn(y) * (a1 - d) / 2.f;
	float a = (a0 + y) * fsg(y,d) + a2 * (1 - fsg(y,d));
	float output = -r * a / d * fsg(a,d) - r * sgn(a) * (1 - fsg(a,d));
	return output;
}

float fsg(float x, float d)
{
	float y = (sgn(x + d) - sgn(x - d)) / 2.f;
	return y;
}
