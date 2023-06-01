/**
  ******************************************************************************
  * @file           : judge.c\h
  * @brief          : 
  * @update         : finish 2022-2-13 20:06:34 
  ******************************************************************************
  */

#include "judge.h"
#include "drv_can.h"
#include "remote.h"
#include "Car.h"

extern rc_t                   rc_structure;

extern CAN_HandleTypeDef hcan1;

judge_config_t judge_config = 
{
	.buffer_max = 200,
};

judge_base_info_t judge_base_info =
{
	.remain_bullte = 150,
};

judge_info_t judge_info = 
{
	.offline_cnt_max = 1000,
	
};
judge_t judge = 
{
	.config = &judge_config,
	.base_info = &judge_base_info,
	.info = &judge_info,
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
void Judge_Get_Data(uint32_t id, uint8_t *data)
{
	judge.info->offline_cnt = 0;
	switch(id)
	{
	case game_robot_status:
		judge.base_info->last_HP = judge.base_info->remain_HP;
		memcpy(&judge.base_info->shooter_cooling_limit, data, 2);
		memcpy(&judge.base_info->car_color, &data[2], 1);
		memcpy(&judge.base_info->chassis_power_limit, &data[3], 2);
		memcpy(&judge.base_info->remain_HP, &data[5], 2);
	
		judge.base_info->last_commond = judge.base_info->robot_commond;
		judge.base_info->robot_commond = (uint8_t)data[7];
		
		judge.info->offline_cnt = 0;
		//judge.info->status = DEV_ONLINE;
	break;
	
	case power_heat_data://功率缓冲、热量
		
		/*功率缓冲区*/
		memcpy(&judge.base_info->chassis_power_buffer,data,2);
	
		/*一号枪管热量限制*/
		memcpy(&judge.base_info->shooter_id1_cooling_heat,&data[2],2);
	
		/*二号枪管热量限制*/
		memcpy(&judge.base_info->shooter_id2_cooling_heat,&data[4],2);
	
		/*比赛开始状态*/
		memcpy(&judge.base_info->game_progress, &data[6], 1);
	
		/*装甲板伤害*/
		judge.base_info->last_armor_id = judge.base_info->armor_id;
		memcpy(&judge.base_info->armor_id, &data[7], 1);
		
		if(S1_UP && S2_UP)
		{
			judge.base_info->game_progress = 4; 
		}
		
		judge.info->offline_cnt = 0;
	break;
	
	/*右1左2*/
		case shoot_data://射速
			if(data[0] == 1)
			{
				memcpy(&judge.base_info->shooter_id1_speed,&data[1],4);
				if(judge.base_info->shooter_id1_speed >= 28.5)
				{
					shoot_1_over_speed_flag -= 3;
				}
				else if(judge.base_info->shooter_id1_speed <= 27.5)
				{
					shoot_1_over_speed_flag++;
				}
				
				judge.base_info->remain_bullte = data[7];
			}
			else if(data[0] == 2)
			{
				memcpy(&judge.base_info->shooter_id2_speed,&data[1],4);
				if(judge.base_info->shooter_id2_speed >=  28.5)
				{
					shoot_2_over_speed_flag -= 3	;
				}
				else if(judge.base_info->shooter_id1_speed <= 27.5)
				{
					shoot_2_over_speed_flag++;
				}
				judge.base_info->remain_bullte = data[7];
			}
			
			memcpy(&judge.base_info->shooter_cooling_limit,&data[1],2);
			
			judge.info->offline_cnt = 0;
			break;
		case game_robot_pos://yaw轴角度
			memcpy(&judge.base_info->gimbal_yaw_angle,data,4);
			judge.info->offline_cnt = 0;

			break;
		case game_robot_HP:
			
			judge.base_info->friendly_outposts_HP = data[0];
			judge.base_info->enemy_outposts_HP = data[1];
			judge.base_info->enemy_hero_HP = data[2];
			judge.base_info->enemy_infanry3_HP = data[3];
			judge.base_info->enemy_infanry4_HP = data[4];
			judge.base_info->enemy_infanry5_HP = data[5];
			judge.base_info->enemy_Shaobing_HP = data[6];
			judge.base_info->remain_time = data[7];
		
	break;
	}

}
