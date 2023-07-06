/**
  ******************************************************************************
  * @file           : config_tracking_differentiator.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#include "config_tracking_differentiator.h"
#include "tracking_differentiator.h"

/*
tracking_differentiator_info_t example_track_tor_info =
{
	.r = , //加加速度
	.h0 = , //滤波系数
	.output_differential_value_max = , //加速度最大值
};
*/

tracking_differentiator_info_t chassis_front_speed_track_tor_info =
{
	.r = 0.1, //加加速度
	.h0 = 1, //滤波系数
	.output_differential_value_max = 20, //加速度最大值
};

tracking_differentiator_info_t chassis_right_speed_track_tor_info =
{
	.r = 0.1, //加加速度
	.h0 = 1, //滤波系数
	.output_differential_value_max = 20, //加速度最大值
};

tracking_differentiator_info_t pitch_speed_track_tor_info =
{
	.r = 1, //加加速度
	.h0 = 1, //滤波系数
	.output_differential_value_max = -1, //加速度最大值
};
