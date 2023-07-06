/**
  ******************************************************************************
  * @file           : RP_ui.h
	* @author         : czf
	* @date           : 2022Äê6ÔÂ25ÈÕ14:57:42
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "stm32f4xx_hal.h"

typedef struct
{
	
}RP_ui_info_t;

typedef struct 
{
	RP_ui_info_t *info;
}RP_ui_t;

void Client_task(void);
void Client_info_update(void);
