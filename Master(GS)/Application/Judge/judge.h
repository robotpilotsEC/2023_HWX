/**
  ******************************************************************************
  * @file           : judge.h
  * @brief          : 
  * @update         : finish 2022-2-13 20:10:12
  ******************************************************************************
  */

#ifndef __JUDGE_H
#define __JUDGE_H

#include "stm32f4xx_hal.h"

/* 下主控can接收id */
#define power_heat_data   0x300
#define game_robot_status 0x301
#define shoot_data        0x302
#define game_robot_pos    0x303
#define robot_hurt        0x304
#define game_robot_HP     0x305

typedef enum
{
	ammo_HURT      = 0,
	offline_HURT   = 1,
	overshoot_HURT = 2,
	overheat_HURT  = 3,
	overpower_HURT = 4,
	crush_HURT     = 5,
	no_HURT        = 6,
	
}hurt_type_e;

#define JUDGE_ONLINE 1
#define JUDGE_OFFLINE 0

typedef struct
{
	uint16_t remain_HP;
	uint16_t last_HP;
	int16_t chassis_power_buffer;           //底盘缓存功率
	int32_t chassis_out_put_max;            //底盘最大输出
	uint16_t shooter_cooling_limit;		//机器人 17mm 枪口热量上限
	uint16_t shooter_id1_cooling_heat;
	uint16_t shooter_id2_cooling_heat; 		//机器人 17mm 枪口热量
	float    shooter_id1_speed;
	float    shooter_id2_speed;
	float   gimbal_yaw_angle;             //枪管yaw轴角度 ?? 
	hurt_type_e hurt_type;				//伤害类型
	uint8_t   last_armor_id;
	uint8_t   armor_id;
	
	uint8_t car_color;                      //1自己蓝色 0自己红色

	//伤害种类
	uint16_t chassis_power_limit;           //底盘功率限制
	uint16_t shooter_id1_42mm_speed_limit;  //射速上限
	uint8_t rfid;
	uint8_t game_progress;
	
	uint8_t friendly_outposts_HP;
	uint8_t enemy_outposts_HP;
	uint8_t enemy_hero_HP;
	uint8_t enemy_infanry3_HP;
	uint8_t enemy_infanry4_HP;
	uint8_t enemy_infanry5_HP;
	uint8_t enemy_Shaobing_HP;
	
	uint8_t remain_time;
	uint8_t remain_bullte;
	
	uint8_t robot_commond;
	uint8_t last_commond;
	
}judge_base_info_t;



typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
}judge_info_t;

typedef struct
{
	uint16_t shoot_num;
	
	uint16_t speed_260;
	uint16_t speed_270;
	uint16_t speed_271;
	uint16_t speed_272;
	uint16_t speed_273;
	uint16_t speed_274;
	uint16_t speed_275;
	uint16_t speed_276;
	uint16_t speed_277;
	uint16_t speed_278;
	uint16_t speed_279;
	uint16_t speed_280;
	uint16_t speed_281;
	uint16_t speed_282;
	uint16_t speed_283;
	uint16_t speed_284;
	uint16_t speed_285;
	uint16_t speed_286;
	uint16_t speed_287;
	uint16_t speed_288;
	uint16_t speed_289;
	uint16_t speed_290;
	
	float mean;
	float variance;
	
}shoot_data_t;

typedef struct
{
	int16_t buffer_max;
}judge_config_t;

typedef struct 
{
	judge_config_t *config;
	judge_base_info_t *base_info;
	judge_info_t *info;
}judge_t;


void Judge_Get_Data(uint32_t id, uint8_t *data);
void judge_heart();
#endif
