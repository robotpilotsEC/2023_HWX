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
#include "judge_protocol.h"



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
	int16_t  chassis_power_buffer;           //���̻��湦��
	int32_t  chassis_out_put_max;            //����������
	uint16_t shooter_cooling_limit;		//������ 17mm ǹ����������
	uint16_t shooter_id1_cooling_heat;
	uint16_t shooter_id2_cooling_heat; 		//������ 17mm ǹ������
	float    shooter_id1_speed;
	float    shooter_id2_speed;
	float    gimbal_yaw_angle;             //ǹ��yaw��Ƕ� ?? 
	hurt_type_e hurt_type;				//�˺�����
	uint8_t   last_armor_id;
	uint8_t   armor_id;
	
	uint8_t car_color;                      //1�Լ���ɫ 0�Լ���ɫ

	//�˺�����
	uint16_t chassis_power_limit;           //���̹�������
	uint16_t shooter_id1_42mm_speed_limit;  //��������
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
	ext_rfid_status_t       rfid_status;
	ext_game_status_t       game_status;
	ext_game_robot_status_t game_robot_status;
	ext_power_heat_data_t   power_heat_data;
	ext_shoot_data_t        shoot_data;
	ext_game_robot_pos_t    game_robot_pos;
	ext_robot_hurt_t        ext_robot_hurt;
	ext_aerial_data_t       ext_aerial_data;
	ext_robot_command_t     ext_robot_command;
	ext_game_robot_HP_t     game_robot_HP;
	ext_bullet_remaining_t  bullet_remaining;
	ground_robot_position_t robot_position;
	
}judge_rawdata_t;


typedef struct 
{
	judge_config_t *config;
	judge_base_info_t *base_info;
	judge_info_t *info;
	judge_rawdata_t *data;
}judge_t;


void Judge_Get_Data(uint32_t id, uint8_t *data);
void judge_heart(void);
#endif
