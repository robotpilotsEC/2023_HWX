/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v1.1		
  * @Author     : hwx			
  * @Date       : 2023年2月28日         
  * @Description:    
  *
  *
  ******************************************************************************
 */
 
#include "judge_protocol.h"
#include "cap_protocol.h"
#include "judge.h"
#include "drv_can.h"
#include "remote.h"
#include "Car.h"
#include "crc.h"
#include "cap.h"

extern rc_t                   rc_structure;

extern CAN_HandleTypeDef hcan1;

judge_frame_header_t judge_frame_header;

drv_judge_info_t drv_judge_info = {
	.frame_header = &judge_frame_header,
};
shoot_data_t shoot_statistics;

judge_config_t judge_config = 
{
	.buffer_max = 200,
};

judge_base_info_t judge_base_info =
{
	.remain_bullte = 150,
};
judge_rawdata_t judge_rawdata;

judge_info_t judge_info = 
{
	.offline_cnt_max = 1000,
	
};

judge_t judge = 
{
	.config    = &judge_config,
	.base_info = &judge_base_info,
	.info      = &judge_info,
	.data      = &judge_rawdata,
};



uint8_t shoot_1_over_speed_flag = 64;
uint8_t shoot_2_over_speed_flag = 64;

void judge_heart()
{
	if(judge.info->offline_cnt ++ >= judge.info->offline_cnt_max)
	{
		judge.info->offline_cnt = judge.info->offline_cnt_max;
		judge.info->status = JUDGE_OFFLINE;
	}
	else
	{
		judge.info->status = JUDGE_ONLINE;
	}
}

void judge_update(uint16_t id, uint8_t *rxBuf)
{
	switch(id)
	{
		case ID_rfid_status:
			memcpy(&judge.data->rfid_status, rxBuf, LEN_rfid_status);
			break;
		case ID_game_state:
			memcpy(&judge.data->game_status, rxBuf, LEN_game_state);
			if(judge.data->game_status.game_progress == 0x04)
			{
				cap.record_Y_O_N = 1;
			}
			else 
			{
				/*修改电容默认永远开启，外置开关*/
				cap.record_Y_O_N = 1;
			}
			break;
		case ID_power_heat_data:
			memcpy(&judge.data->power_heat_data, rxBuf, LEN_power_heat_data);
			cap_send_2E();
			break;
		case ID_game_robot_state:	
			memcpy(&judge.data->game_robot_status, rxBuf, LEN_game_robot_state);
			cap_send_2F();
			break;
		case ID_shoot_data:
			memcpy(&judge.data->shoot_data, rxBuf, LEN_shoot_data);
			break;
		case ID_game_robot_pos:
			memcpy(&judge.data->game_robot_pos, rxBuf, LEN_game_robot_pos);
			break;
		case ID_robot_hurt:
			memcpy(&judge.data->ext_robot_hurt, rxBuf, LEN_robot_hurt);
			break;
		case ID_aerial_data:
			memcpy(&judge.data->ext_robot_command, rxBuf, LEN_aerial_data);
		case ID_game_robot_HP:
			memcpy(&judge.data->game_robot_HP, rxBuf, LEN_game_robot_HP);
			break;
		case ID_bullet_remaining:
			memcpy(&judge.data->bullet_remaining, rxBuf, LEN_bullet_remaining);
		
			break;
		default:
			break;
	}
}

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
