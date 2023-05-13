#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "stm32f4xx_hal.h"


/* 数值函数 */
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define abs(x) 					((x)>0? (x):(-(x)))

int16_t RampInt(int16_t final, int16_t now, int16_t ramp);
float RampFloat(float final, float now, float ramp);
float DeathZoom(float input, float center, float death);
float Low_Pass_Fliter(float data , float last_data , float a);
#endif

