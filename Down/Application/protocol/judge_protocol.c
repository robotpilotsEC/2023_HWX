/**
  ******************************************************************************
  * @file           : judge_potocol.c
	* @author         : czf
	* @date           : 2022-4-22 14:38:38
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "judge_protocol.h"
#include "config_judge.h"
#include "string.h"
#include "CRC8.h"
#include "CRC16.h"

judge_frame_header_t judge_frame_header;
drv_judge_info_t drv_judge_info = {
	.frame_header = &judge_frame_header,
};

void judge_recive(uint8_t *rxBuf)
{
	uint16_t frame_length;
	if( rxBuf == NULL )
	{
		return;
	}
	drv_judge_info.frame_header->SOF = rxBuf[0];
	if(drv_judge_info.frame_header->SOF == 0xA5)
	{
		memcpy(&drv_judge_info.frame_header->data_length, rxBuf + 1, 4);
		if(Verify_CRC8_Check_Sum(rxBuf, 5) == 1)
		{
			frame_length = 5 + 2 + drv_judge_info.frame_header->data_length + 2;
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == 1)
			{
				memcpy(&drv_judge_info.cmd_id, rxBuf + 5, 2);
				judge_update(drv_judge_info.cmd_id, rxBuf + 7);
			}
			memcpy(&drv_judge_info.frame_tail, rxBuf + 5 + 2 + drv_judge_info.frame_header->data_length, 2);
		}
	}

	/* 如果一个数据包出现了多帧数据就再次读取 */
	if(rxBuf[frame_length] == 0xA5)
	{
		judge_recive( &rxBuf[frame_length] );
	}
}
