/**
  ******************************************************************************
  * @file           : config_can.c\h
  * @brief          : 
  * @note           : 2022-1-18 13:03:50
  ******************************************************************************
  */

#include "config_can.h"
#include "config.h"
#include "string.h"
#include "cap_protocol.h"
#include "judge.h"
#include "debug.h"
#include "my_judge.h"
#include "cap.h"

//处理CAN1接收的数据
void CAN1_Get_Data(uint32_t id, uint8_t *data)
{
	switch (id)
	{
		case 0x0FF:
			if(data[0] == 1)
			{
				cap.Y_O_N = 1;
			}
			else 
			{
				cap.Y_O_N = 0;
			}
			break;
		case 0x300:
		default:
			break;
	}
}

//处理CAN2接收的数据
void CAN2_Get_Data(uint32_t id, uint8_t *data)
{
	switch (id)
	{
		case 0x030:
			cap_update(data);
		default:
			break;
	}
}
