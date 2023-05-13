/**
  ******************************************************************************
  * @file           : config_judge.c\h
	* @author         : czf
	* @date           : 2022-4-22 15:13:20
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "config_judge.h"
#include "cap_protocol.h"
#include "judge_protocol.h"
#include "judge.h"
#include "string.h"
#include "drv_can.h"
#include "cap_protocol.h"
#include "up_com_protocol.h"
#include "cap.h"

extern uint8_t cap_tx_buf_1[8]; //0x2E
extern uint8_t cap_tx_buf_2[8]; //0x2F

void judge_update(uint16_t id, uint8_t *rxBuf)
{
	switch(id)
	{
		case ID_rfid_status:
			memcpy(&judge.info->rfid_status, rxBuf, LEN_rfid_status);
			break;
		case ID_game_state:
			memcpy(&judge.info->game_status, rxBuf, LEN_game_state);
			if(judge.info->game_status.game_progress == 0x04)
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
			memcpy(&judge.info->power_heat_data, rxBuf, LEN_power_heat_data);
			cap_send_2E();
			up_send_power_heat();
			break;
		case ID_game_robot_state:	
			memcpy(&judge.info->game_robot_status, rxBuf, LEN_game_robot_state);
			cap_send_2F();
			up_send_game_robot_state();
			break;
		case ID_shoot_data:
			memcpy(&judge.info->shoot_data, rxBuf, LEN_shoot_data);
			up_send_shoot();
			break;
		case ID_game_robot_pos:
			memcpy(&judge.info->game_robot_pos, rxBuf, LEN_game_robot_pos);
			//up_send_robot_pos();
			break;
		case ID_robot_hurt:
			memcpy(&judge.info->ext_robot_hurt, rxBuf, LEN_robot_hurt);
			//up_send_robot_hurt();
			break;
		case ID_aerial_data:
			memcpy(&judge.info->ext_robot_command, rxBuf, LEN_aerial_data);
			//发送
		case ID_game_robot_HP:
			memcpy(&judge.info->game_robot_HP, rxBuf, LEN_game_robot_HP);
			up_send_robot_HP();
			break;
		case ID_bullet_remaining:
			memcpy(&judge.info->bullet_remaining, rxBuf, LEN_bullet_remaining);
		
			break;
		default:
			break;
	}
}
