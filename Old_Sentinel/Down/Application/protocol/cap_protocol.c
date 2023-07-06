/**
  ******************************************************************************
  * @file           : cap_protocol.c\h
	* @author         : czf
	* @date           : 2022-4-22 15:41:14
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "cap_protocol.h"
#include "string.h"
#include "judge.h"
#include "drv_can.h"
#include "cap.h"

cap_receive_data_t cap_receive_data;

uint8_t cap_tx_buf_1[8]; //0x2E
uint8_t cap_tx_buf_2[8]; //0x2F

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void cap_send_2E(void)
{
	cap_tx_buf_1[0] = judge.info->power_heat_data.chassis_power_buffer >> 8;
	cap_tx_buf_1[1] = judge.info->power_heat_data.chassis_power_buffer;
	cap_tx_buf_1[2] = judge.info->power_heat_data.chassis_volt >> 8;
	cap_tx_buf_1[3] = judge.info->power_heat_data.chassis_volt;
	cap_tx_buf_1[4] = judge.info->power_heat_data.chassis_current >> 8;
	cap_tx_buf_1[5] = judge.info->power_heat_data.chassis_current;
	CAN_u8_SendData(&hcan2, 0x2E, cap_tx_buf_1);
}

void cap_send_2F(void)
{
	uint16_t temp;
	cap_tx_buf_2[0] = judge.info->game_robot_status.chassis_power_limit >> 8;
	cap_tx_buf_2[1] = judge.info->game_robot_status.chassis_power_limit;
	temp = 300;
	cap_tx_buf_2[2] = temp >> 8;
	cap_tx_buf_2[3] = temp;
	temp = 150;
	cap_tx_buf_2[4] = temp >> 8;
	cap_tx_buf_2[5] = temp;
	if(cap.Y_O_N == 1)
	{
		if(cap.record_Y_O_N == 1)
		{
			temp = 0x0700;
		}
		else 
		{
			temp = 0x0300;
		}
	}
	else 
	{
		temp = 0x0000;
	}
	memcpy(&cap_tx_buf_2[6], &temp, 2);
	CAN_u8_SendData(&hcan2, 0x2F, cap_tx_buf_2);
}

void cap_update(uint8_t *rxBuf)
{
	cap_receive_data.cap_Ucr = int16_to_float(((uint16_t)rxBuf[0] << 8| rxBuf[1]), 32000, -32000, 30, 0);
  cap_receive_data.cap_I = int16_to_float(((uint16_t)rxBuf[2] << 8| rxBuf[3]), 32000, -32000, 20, -20);
  cap_receive_data.cap_state.state = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);        
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;   //加0.5使向下取整变成四舍五入
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}
