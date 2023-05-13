#include "led.h"
#include "config.h"

Led led[4];

void LED_LIGHT( uint8_t LED, uint16_t times)
{
	led[LED].config_times = times;
}

void led_work(void)
{
	led[LED_Blue].cnt++;
	led[LED_ORANGE].cnt++;
	led[LED_RED].cnt++;
	led[LED_GREEN].cnt++;
	switch(led[LED_Blue].statue)
	{
		case OFF:
			if(led[LED_Blue].config_times != 0)
			{
				led[LED_Blue].target_times = led[LED_Blue].config_times;
				led[LED_Blue].config_times = 0;
				led[LED_Blue].cnt = 0;
				led[LED_Blue].statue = LIGHT;
				LED_Blue_On;
				led[LED_Blue].measure_times = 1;
			}
			break;
		case LIGHT:
			if(led[LED_Blue].cnt >= Light_t)
			{
				led[LED_Blue].cnt = 0;
				led[LED_Blue].statue = INTERVAL;
				LED_Blue_Off;
			}
			break;
		case INTERVAL:
			if(led[LED_Blue].cnt >= Interval_t)
			{
				if(led[LED_Blue].measure_times >= led[LED_Blue].target_times)
				{
					led[LED_Blue].cnt = 0;
					led[LED_Blue].statue = WAIT;
					led[LED_Blue].target_times = 0;
				}
				else 
				{
					led[LED_Blue].cnt = 0;
					led[LED_Blue].statue = LIGHT;
					led[LED_Blue].measure_times++;
					LED_Blue_On;
				}
			}
			break;
		case WAIT:
			if(led[LED_Blue].cnt >= Wait_t)
			{
				led[LED_Blue].cnt = 0;
				led[LED_Blue].statue = OFF;
			}
			break;
	}
	switch(led[LED_ORANGE].statue)
	{
		case OFF:
			if(led[LED_ORANGE].config_times != 0)
			{
				led[LED_ORANGE].target_times = led[LED_ORANGE].config_times;
				led[LED_ORANGE].config_times = 0;
				led[LED_ORANGE].cnt = 0;
				led[LED_ORANGE].statue = LIGHT;
				LED_ORANGE_On;
				led[LED_ORANGE].measure_times = 1;
			}
			break;
		case LIGHT:
			if(led[LED_ORANGE].cnt >= Light_t)
			{
				led[LED_ORANGE].cnt = 0;
				led[LED_ORANGE].statue = INTERVAL;
				LED_ORANGE_Off;
			}
			break;
		case INTERVAL:
			if(led[LED_ORANGE].cnt >= Interval_t)
			{
				if(led[LED_ORANGE].measure_times >= led[LED_ORANGE].target_times)
				{
					led[LED_ORANGE].cnt = 0;
					led[LED_ORANGE].statue = WAIT;
					led[LED_ORANGE].target_times = 0;
				}
				else 
				{
					led[LED_ORANGE].cnt = 0;
					led[LED_ORANGE].statue = LIGHT;
					led[LED_ORANGE].measure_times++;
					LED_ORANGE_On;
				}
			}
			break;
		case WAIT:
			if(led[LED_ORANGE].cnt >= Wait_t)
			{
				led[LED_ORANGE].cnt = 0;
				led[LED_ORANGE].statue = OFF;
			}
			break;
	}
	switch(led[LED_RED].statue)
	{
		case OFF:
			if(led[LED_RED].config_times != 0)
			{
				led[LED_RED].target_times = led[LED_RED].config_times;
				led[LED_RED].config_times = 0;
				led[LED_RED].cnt = 0;
				led[LED_RED].statue = LIGHT;
				LED_RED_On;
				led[LED_RED].measure_times = 1;
			}
			break;
		case LIGHT:
			if(led[LED_RED].cnt >= Light_t)
			{
				led[LED_RED].cnt = 0;
				led[LED_RED].statue = INTERVAL;
				LED_RED_Off;
			}
			break;
		case INTERVAL:
			if(led[LED_RED].cnt >= Interval_t)
			{
				if(led[LED_RED].measure_times >= led[LED_RED].target_times)
				{
					led[LED_RED].cnt = 0;
					led[LED_RED].statue = WAIT;
					led[LED_RED].target_times = 0;
				}
				else 
				{
					led[LED_RED].cnt = 0;
					led[LED_RED].statue = LIGHT;
					led[LED_RED].measure_times++;
					LED_RED_On;
				}
			}
			break;
		case WAIT:
			if(led[LED_RED].cnt >= Wait_t)
			{
				led[LED_RED].cnt = 0;
				led[LED_RED].statue = OFF;
			}
			break;
	}
	switch(led[LED_GREEN].statue)
	{
		case OFF:
			if(led[LED_GREEN].config_times != 0)
			{
				led[LED_GREEN].target_times = led[LED_GREEN].config_times;
				led[LED_GREEN].config_times = 0;
				led[LED_GREEN].cnt = 0;
				led[LED_GREEN].statue = LIGHT;
				LED_GREEN_On;
				led[LED_GREEN].measure_times = 1;
			}
			break;
		case LIGHT:
			if(led[LED_GREEN].cnt >= Light_t)
			{
				led[LED_GREEN].cnt = 0;
				led[LED_GREEN].statue = INTERVAL;
				LED_GREEN_Off;
			}
			break;
		case INTERVAL:
			if(led[LED_GREEN].cnt >= Interval_t)
			{
				if(led[LED_GREEN].measure_times >= led[LED_GREEN].target_times)
				{
					led[LED_GREEN].cnt = 0;
					led[LED_GREEN].statue = WAIT;
					led[LED_GREEN].target_times = 0;
				}
				else 
				{
					led[LED_GREEN].cnt = 0;
					led[LED_GREEN].statue = LIGHT;
					led[LED_GREEN].measure_times++;
					LED_GREEN_On;
				}
			}
			break;
		case WAIT:
			if(led[LED_GREEN].cnt >= Wait_t)
			{
				led[LED_GREEN].cnt = 0;
				led[LED_GREEN].statue = OFF;
			}
			break;
	}
}
