/**
  ******************************************************************************
  * @file           : config_usmart.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#include "config_usmart.h"
#include "usmart.h"
#include "launcher.h"

usmart_cmd_list_t usmart_cmd_list[] = 
{
	(void*)launcher_motor_speed_change,"launcher_speed_change(speed)",
};
