/**
  ******************************************************************************
  * @file           : up_com_protocol.c\h
	* @author         : czf
	* @date           : 2022-4-22 20:16:32
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "up_com_protocol.h"
#include "judge.h"
#include "string.h"
#include "drv_can.h"
#define DEBUG_MODE 0
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern IWDG_HandleTypeDef hiwdg;


uint8_t up_com_tx_buf_1[8]; //power_heat
uint8_t up_com_tx_buf_2[8]; //game_robot_status
uint8_t up_com_tx_buf_3[8]; //shoot_data
uint8_t up_com_tx_buf_4[8]; //game_robot_pos
uint8_t up_com_tx_buf_5[8]; //robot_hurt
uint8_t up_com_tx_buf_6[8]; //robot_HP


uint8_t tem_hurt = 0;
void up_send_power_heat(void)
{
	//uint8_t rfid = judge.info->rfid_status.rfid_status;
	
	/*功率缓冲区*/
	memcpy(up_com_tx_buf_1, (void*)&judge.info->power_heat_data.chassis_power_buffer, 2);
	
	/*一号枪管热量限制*/
	memcpy(&up_com_tx_buf_1[2], (void*)&judge.info->power_heat_data.shooter_id1_17mm_cooling_heat, 2);
	
	/*二号枪管热量限制*/
	memcpy(&up_com_tx_buf_1[4], (void*)&judge.info->power_heat_data.shooter_id2_17mm_cooling_heat, 2);
	
	/*比赛开始状态*/
	up_com_tx_buf_1[6] = judge.info->game_status.game_progress;
	
	/*伤害类型*/
	up_com_tx_buf_1[7] = judge.info->ext_robot_hurt.armor_id;
	
	CAN_u8_SendData(&hcan1, 0x300, up_com_tx_buf_1);
}

void up_send_game_robot_state(void)
{
	memcpy(up_com_tx_buf_2, (void*)&judge.info->game_robot_status.shooter_id1_42mm_cooling_limit, 2);
	
	/*我方是蓝色还是红色*/
	if(judge.info->game_robot_status.robot_id == 7)
	{
		up_com_tx_buf_2[2] = 0;
	}
	else if(judge.info->game_robot_status.robot_id == 107)
	{
			up_com_tx_buf_2[2] = 1;
	}
	/*底盘功率上限*/
	memcpy(&up_com_tx_buf_2[3], (void*)&judge.info->game_robot_status.chassis_power_limit, 2);
	
	/*当前血量*/
	memcpy(&up_com_tx_buf_2[5], (void*)&judge.info->game_robot_status.remain_HP, 2);
	
	/*云台手命令*/
	memcpy(&up_com_tx_buf_2[7], (void*)&judge.info->ext_robot_command.commd_keyboard, 1);
	
	#if DEBUG_MODE
	HAL_IWDG_Refresh(&hiwdg);
	#endif
	CAN_u8_SendData(&hcan1, 0x301, up_com_tx_buf_2);
}

void up_send_shoot(void)
	
{	/*射速*/
	memcpy(up_com_tx_buf_3, (void*)&judge.info->shoot_data.shooter_id, 1);
	memcpy(&up_com_tx_buf_3[1], (void*)&judge.info->shoot_data.bullet_speed, 4);
	memcpy(&up_com_tx_buf_3[5], (void*)&judge.info->game_robot_status.shooter_id1_17mm_speed_limit, 2);
	up_com_tx_buf_3[7] = (uint8_t)(judge.info->bullet_remaining.bullet_remaining_num_17mm/5);
	CAN_u8_SendData(&hcan1, 0x302, up_com_tx_buf_3);
}

void up_send_robot_pos(void)
{
	memcpy(up_com_tx_buf_4, (void*)&judge.info->game_robot_pos.yaw, 4);
	//CAN_u8_SendData(&hcan1, 0x303, up_com_tx_buf_4);
}

void up_send_robot_hurt(void)
{
		
		up_com_tx_buf_5[0] = judge.info->ext_robot_hurt.hurt_type;
		CAN_u8_SendData(&hcan1, 0x304, up_com_tx_buf_5);
}

void up_send_robot_HP(void)
{
	if(judge.info->game_robot_status.robot_id == 7)
	{
		up_com_tx_buf_6[0] = (uint8_t)(judge.info->game_robot_HP.blue_outpost_HP/5);
		up_com_tx_buf_6[1] = (uint8_t)(judge.info->game_robot_HP.red_outpost_HP/5);
		up_com_tx_buf_6[2] = (uint8_t)(judge.info->game_robot_HP.blue_1_robot_HP/5);
		up_com_tx_buf_6[3] = (uint8_t)(judge.info->game_robot_HP.blue_3_robot_HP/5);
		up_com_tx_buf_6[4] = (uint8_t)(judge.info->game_robot_HP.blue_4_robot_HP/5);
		up_com_tx_buf_6[5] = (uint8_t)(judge.info->game_robot_HP.blue_5_robot_HP/5);
		up_com_tx_buf_6[6] = (uint8_t)(judge.info->game_robot_HP.blue_7_robot_HP/5);
		up_com_tx_buf_6[7] = (uint8_t)(judge.info->game_status.stage_remain_time/5);

	}
	else if(judge.info->game_robot_status.robot_id == 107)
	{
		up_com_tx_buf_6[0] = (uint8_t)(judge.info->game_robot_HP.blue_outpost_HP/5);
		up_com_tx_buf_6[1] = (uint8_t)(judge.info->game_robot_HP.red_outpost_HP/5);
		up_com_tx_buf_6[2] = (uint8_t)(judge.info->game_robot_HP.red_1_robot_HP/5);
		up_com_tx_buf_6[3] = (uint8_t)(judge.info->game_robot_HP.red_3_robot_HP/5);
		up_com_tx_buf_6[4] = (uint8_t)(judge.info->game_robot_HP.red_4_robot_HP/5);
		up_com_tx_buf_6[5] = (uint8_t)(judge.info->game_robot_HP.red_5_robot_HP/5);
		up_com_tx_buf_6[6] = (uint8_t)(judge.info->game_robot_HP.red_7_robot_HP/5);
		up_com_tx_buf_6[7] = (uint8_t)(judge.info->game_status.stage_remain_time/5);

	}
	CAN_u8_SendData(&hcan1, 0x305, up_com_tx_buf_6);
}
