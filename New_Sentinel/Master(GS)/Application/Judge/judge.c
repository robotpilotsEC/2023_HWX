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

uint32_t shoot_1_time = 0;
uint32_t shoot_2_time = 0;
uint32_t elapsed_time = 0;
float elapsed_seconds = 0;

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
	/*掉线计数器清零*/
	judge.info->offline_cnt = 0;
	
	switch(id)
	{
		case ID_rfid_status:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->rfid_status, rxBuf, LEN_rfid_status);
			
			/*数据处理-----------------------------------------------------------------------------------------*/
			break;
		case ID_game_state:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->game_status, rxBuf, LEN_game_state);
			/*数据处理-----------------------------------------------------------------------------------------*/
			judge.base_info->game_progress = judge.data->game_status.game_progress;
			if(S1_UP && S2_UP)
			{
				judge.base_info->game_progress = 4; 
			}
			break;
		
		case ID_power_heat_data:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->power_heat_data, rxBuf, LEN_power_heat_data);
			cap_send_2E();
			/*数据处理-----------------------------------------------------------------------------------------*/
			judge.base_info->shooter_cooling_limit    = judge.data->power_heat_data.shooter_id1_17mm_cooling_heat;
			judge.base_info->chassis_power_buffer     = judge.data->power_heat_data.chassis_power_buffer;
			judge.base_info->shooter_id1_cooling_heat = judge.data->power_heat_data.shooter_id1_17mm_cooling_heat;
			judge.base_info->shooter_id2_cooling_heat = judge.data->power_heat_data.shooter_id2_17mm_cooling_heat;
		
			break;
		case ID_game_robot_state:
			/*数据接收-----------------------------------------------------------------------------------------*/			
			memcpy(&judge.data->game_robot_status, rxBuf, LEN_game_robot_state);
			cap_send_2F();
			/*数据处理-----------------------------------------------------------------------------------------*/
			if(judge.data->game_robot_status.robot_id == 7)
			{
				judge.base_info->car_color = 0;
			}
			else if(judge.data->game_robot_status.robot_id == 107)
			{
				judge.base_info->car_color = 1;
			}
			judge.base_info->chassis_power_limit = judge.data->game_robot_status.chassis_power_limit;
			judge.base_info->last_HP             = judge.base_info->remain_HP;
			judge.base_info->remain_HP           = judge.data->game_robot_status.remain_HP;
			
			break;
		case ID_shoot_data:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->shoot_data, rxBuf, LEN_shoot_data);
			/*数据处理-----------------------------------------------------------------------------------------*/
			/*右1左2*/
			if(judge.data->shoot_data.shooter_id == 1)
			{
				judge.base_info->shooter_id1_speed = judge.data->shoot_data.bullet_speed;
				if(judge.base_info->shooter_id1_speed >= 28.5)
				{
					shoot_1_over_speed_flag -= 3;
				}
				else if(judge.base_info->shooter_id1_speed <= 27.5)
				{
					shoot_1_over_speed_flag++;
				}
				
				shoot_1_time = HAL_GetTick();
			}
			else if(judge.data->shoot_data.shooter_id == 2)
			{
				judge.base_info->shooter_id2_speed = judge.data->shoot_data.bullet_speed;
				if(judge.base_info->shooter_id2_speed >=  28.5)
				{
					shoot_2_over_speed_flag -= 3	;
				}
				else if(judge.base_info->shooter_id1_speed <= 27.5)
				{
					shoot_2_over_speed_flag++;
				}
				
				shoot_2_time = HAL_GetTick();
			}
			elapsed_time = shoot_1_time - shoot_2_time;
			elapsed_seconds = (float)elapsed_time / HAL_GetTickFreq();
			
			break;
		case ID_game_robot_pos:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->game_robot_pos, rxBuf, LEN_game_robot_pos);
			/*数据处理-----------------------------------------------------------------------------------------*/
			break;
		case ID_robot_hurt:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->ext_robot_hurt, rxBuf, LEN_robot_hurt);
			/*数据处理-----------------------------------------------------------------------------------------*/
			judge.base_info->last_armor_id = judge.base_info->armor_id;
			judge.base_info->armor_id      = judge.data->ext_robot_hurt.armor_id;
			
			break;
		case ID_aerial_data:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->ext_robot_command, rxBuf, LEN_aerial_data);
			/*数据处理-----------------------------------------------------------------------------------------*/
			judge.base_info->last_commond = judge.base_info->robot_commond;
			judge.base_info->robot_commond = judge.data->ext_robot_command.commd_keyboard;
			
			break;
		case ID_game_robot_HP:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->game_robot_HP, rxBuf, LEN_game_robot_HP);
			/*数据处理-----------------------------------------------------------------------------------------*/
			if(judge.data->game_robot_status.robot_id == 7)
			{
				judge.base_info->friendly_outposts_HP = (uint8_t)(judge.data->game_robot_HP.red_outpost_HP/10);
				judge.base_info->enemy_outposts_HP = (uint8_t)(judge.data->game_robot_HP.blue_outpost_HP/10);
				judge.base_info->enemy_hero_HP = (uint8_t)(judge.data->game_robot_HP.blue_1_robot_HP/5);
				judge.base_info->enemy_infanry3_HP = (uint8_t)(judge.data->game_robot_HP.blue_3_robot_HP/5);
				judge.base_info->enemy_infanry4_HP = (uint8_t)(judge.data->game_robot_HP.blue_4_robot_HP/5);
				judge.base_info->enemy_infanry5_HP = (uint8_t)(judge.data->game_robot_HP.blue_5_robot_HP/5);
				judge.base_info->enemy_Shaobing_HP = (uint8_t)(judge.data->game_robot_HP.blue_7_robot_HP/5);
				judge.base_info->remain_time = (uint8_t)(judge.data->game_status.stage_remain_time/5);

			}
			else if(judge.data->game_robot_status.robot_id == 107)
			{

				judge.base_info->friendly_outposts_HP = (uint8_t)(judge.data->game_robot_HP.blue_outpost_HP/10);
				judge.base_info->enemy_outposts_HP = (uint8_t)(judge.data->game_robot_HP.red_outpost_HP/10);
				judge.base_info->enemy_hero_HP = (uint8_t)(judge.data->game_robot_HP.blue_1_robot_HP/5);
				judge.base_info->enemy_infanry3_HP = (uint8_t)(judge.data->game_robot_HP.blue_3_robot_HP/5);
				judge.base_info->enemy_infanry4_HP = (uint8_t)(judge.data->game_robot_HP.blue_4_robot_HP/5);
				judge.base_info->enemy_infanry5_HP = (uint8_t)(judge.data->game_robot_HP.blue_5_robot_HP/5);
				judge.base_info->enemy_Shaobing_HP = (uint8_t)(judge.data->game_robot_HP.blue_7_robot_HP/5);
				judge.base_info->remain_time = (uint8_t)(judge.data->game_status.stage_remain_time/5);

			}
			break;
		case ID_bullet_remaining:
			/*数据接收-----------------------------------------------------------------------------------------*/
			memcpy(&judge.data->bullet_remaining, rxBuf, LEN_bullet_remaining);
			/*数据处理-----------------------------------------------------------------------------------------*/
			judge.base_info->remain_bullte       = judge.data->bullet_remaining.bullet_remaining_num_17mm;
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


/*串口五中断回调函数*/
void USART5_rxDataHandler(uint8_t *rxBuf)
{
	judge_recive(rxBuf);
}