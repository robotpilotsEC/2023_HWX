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

#define SHOOT_SPEED shoot_statistics.speed_

extern rc_t                   rc_structure;

extern CAN_HandleTypeDef hcan1;

shoot_data_t shoot_statistics;

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
		{
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
			
			float s_speed = 0;
			memcpy(&s_speed,&data[1],4);

			if (s_speed >= 26.0 && s_speed <= 27.0)
			{
				shoot_statistics.speed_260++;
			}
			else if (s_speed >= 27.0 && s_speed <= 27.1)
			{
				shoot_statistics.speed_270++;
			}
			else if (s_speed >= 27.1 && s_speed <= 27.2)
			{
				shoot_statistics.speed_271++;
			}
			else if (s_speed >= 27.2 && s_speed <= 27.3)
			{
				shoot_statistics.speed_272++;
			}
			else if (s_speed >= 27.3 && s_speed <= 27.4)
			{
				shoot_statistics.speed_273++;
			}
			else if (s_speed >= 27.4 && s_speed <= 27.5)
			{
				shoot_statistics.speed_274++;
			}
			else if (s_speed >= 27.5 && s_speed <= 27.6)
			{
				shoot_statistics.speed_275++;
			}
			else if (s_speed >= 27.6 && s_speed <= 27.7)
			{
				shoot_statistics.speed_276++;
			}
			else if (s_speed >= 27.7 && s_speed <= 27.8)
			{
				shoot_statistics.speed_277++;
			}
			else if (s_speed >= 27.8 && s_speed <= 27.9)
			{
				shoot_statistics.speed_278++;
			}
			else if (s_speed >= 27.9 && s_speed <= 28.0)
			{
				shoot_statistics.speed_279++;
			}
			else if (s_speed >= 28.0 && s_speed <= 28.1)
			{
				shoot_statistics.speed_280++;
			}
			else if (s_speed >= 28.1 && s_speed <= 28.2)
			{
				shoot_statistics.speed_281++;
			}
			else if (s_speed >= 28.2 && s_speed <= 28.3)
			{
				shoot_statistics.speed_282++;
			}
			else if (s_speed >= 28.3 && s_speed <= 28.4)
			{
				shoot_statistics.speed_283++;
			}
			else if (s_speed >= 28.4 && s_speed <= 28.5)
			{
				shoot_statistics.speed_284++;
			}
			else if (s_speed >= 28.5 && s_speed <= 28.6)
			{
				shoot_statistics.speed_285++;
			}
			else if (s_speed >= 28.6 && s_speed <= 28.7)
			{
				shoot_statistics.speed_286++;
			}
			else if (s_speed >= 28.7 && s_speed <= 28.8)
			{
				shoot_statistics.speed_287++;
			}
			else if (s_speed >= 28.8 && s_speed <= 28.9)
			{
				shoot_statistics.speed_288++;
			}
			else if (s_speed >= 28.9 && s_speed <= 29.0)
			{
				shoot_statistics.speed_289++;
			}
			else if (s_speed >= 29.0 && s_speed <= 29.1)
			{
				shoot_statistics.speed_290++;
			}
			
			/*统计部分*/
			shoot_statistics.shoot_num++;
			
			shoot_statistics.mean = (shoot_statistics.speed_260 * 26.0 + shoot_statistics.speed_270 * 27.0 + shoot_statistics.speed_271 * 27.1 + shoot_statistics.speed_272 * 27.2 + shoot_statistics.speed_273 * 27.3 + shoot_statistics.speed_274 * 27.4 + shoot_statistics.speed_275 * 27.5 + shoot_statistics.speed_276 * 27.6 + shoot_statistics.speed_277 * 27.7 + shoot_statistics.speed_278 * 27.8 + shoot_statistics.speed_279 * 27.9 + shoot_statistics.speed_280 * 28.0 + shoot_statistics.speed_281 * 28.1 + shoot_statistics.speed_282 * 28.2 + shoot_statistics.speed_283 * 28.3 + shoot_statistics.speed_284 * 28.4 + shoot_statistics.speed_285 * 28.5 + shoot_statistics.speed_286 * 28.6 + shoot_statistics.speed_287 * 28.7 + shoot_statistics.speed_288 * 28.8 + shoot_statistics.speed_289 * 28.9 + shoot_statistics.speed_290 * 29.0) / (float)shoot_statistics.shoot_num;
			
			// Calculate the variance
			float sum_of_squares = (shoot_statistics.speed_260 * ((26.0 - shoot_statistics.mean) * (26.0 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_270 * ((27.0 - shoot_statistics.mean) * (27.0 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_271 * ((27.1 - shoot_statistics.mean) * (27.1 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_272 * ((27.2 - shoot_statistics.mean) * (27.2 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_273 * ((27.3 - shoot_statistics.mean) * (27.3 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_274 * ((27.4 - shoot_statistics.mean) * (27.4 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_275 * ((27.5 - shoot_statistics.mean) * (27.5 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_276 * ((27.6 - shoot_statistics.mean) * (27.6 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_277 * ((27.7 - shoot_statistics.mean) * (27.7 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_278 * ((27.8 - shoot_statistics.mean) * (27.8 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_279 * ((27.9 - shoot_statistics.mean) * (27.9 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_280 * ((28.0 - shoot_statistics.mean) * (28.0 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_281 * ((28.1 - shoot_statistics.mean) * (28.1 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_282 * ((28.2 - shoot_statistics.mean) * (28.2 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_283 * ((28.3 - shoot_statistics.mean) * (28.3 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_284 * ((28.4 - shoot_statistics.mean) * (28.4 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_285 * ((28.5 - shoot_statistics.mean) * (28.5 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_286 * ((28.6 - shoot_statistics.mean) * (28.6 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_287 * ((28.7 - shoot_statistics.mean) * (28.7 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_288 * ((28.8 - shoot_statistics.mean) * (28.8 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_289 * ((28.9 - shoot_statistics.mean) * (28.9 - shoot_statistics.mean))) +
								   (shoot_statistics.speed_290 * ((29.0 - shoot_statistics.mean) * (29.0 - shoot_statistics.mean)));

			shoot_statistics.variance = sum_of_squares / (float)shoot_statistics.shoot_num;
		}
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
