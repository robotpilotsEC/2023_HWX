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

/* ������can����id */
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
	int16_t chassis_power_buffer;           //���̻��湦��
	int32_t chassis_out_put_max;            //����������
	uint16_t shooter_cooling_limit;		//������ 17mm ǹ����������
	uint16_t shooter_id1_cooling_heat;
	uint16_t shooter_id2_cooling_heat; 		//������ 17mm ǹ������
	float    shooter_id1_speed;
	float    shooter_id2_speed;
	float   gimbal_yaw_angle;             //ǹ��yaw��Ƕ� ?? 
	hurt_type_e hurt_type;				//�˺�����
	
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


/*robot_commond ����*/

/*
 z ֹͣ��
 x ������

 w ǰ��һ��
 s ����һ��
 a ����һ��
 d ����һ��
*/




typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
}judge_info_t;

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
