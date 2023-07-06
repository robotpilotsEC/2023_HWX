#ifndef __POWER_LIMIT_H
#define __POWER_LIMIT_H

#include "stm32f4xx_hal.h"

#define POWER_PID_O_MAX 8000


void Chassis_Motor_Power_Limit(int16_t *data);
#endif
