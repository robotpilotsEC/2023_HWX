/**
  ******************************************************************************
  * @file           : RP_ui.h
	* @author         : czf
	* @date           : 2022年6月25日14:57:42
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "RP_ui.h"

RP_ui_info_t RP_ui_info;
RP_ui_t RP_ui = 
{
	.info = &RP_ui_info,
};

/**
  * @brief  实时任务
  * @param  
  * @retval 
  */
void Client_task(void)
{
	uint8_t Client_circle = 10;
	
	static uint32_t i;
	
	Client_info_update();
	
	if(i%Client_circle == 0)
	{
		Client_graphic_Init();  //不用一直更新，但是无法判断什么时候进入客户端所以需要轮询,可以操作手key控制
	}
	else if(i%Client_circle == 1)
	{
		Client_graphic_Info_update();
		update_figure_flag = MODIFY;
	}
	else if(i%Client_circle == 4)
	{
		Client_supercapacitor_update();
		update_supercapacitor_flag = MODIFY;
	}
	else if(i%Client_circle == 2)
	{
		Client_aim_update();
		update_aim_flag = MODIFY;
	}
	else if(i%Client_circle == 3)
	{
		Client_gimbal_angle_update();
		update_float_flag = MODIFY;
	}
	else if(i%Client_circle == 5)
	{
		//Client_bullet_int_update();
		update_int_flag = MODIFY;
	}
	else if(i%Client_circle == 6)
	{
    Client_gimbalangle_figure_update();
    update_gimbalangle_flag = MODIFY;
	}
    
 //准星部分
	if(i%200 == 1)
	{
		_lowshortstem_aim_4();
	}
	else if(i%200 == 10)
	{
		_high_aim_();
	}
	else if(i%200 == 20)
	{
		_lowshort_aim_2();
	}
	else if(i%200 == 30)
	{
		_lowshort_aim_3();
	}
	else if(i%200 == 40)
	{
		_lowlong_aim_();
	}
	
	
	if(i%100 == 0)
	{
		update_figure_flag = ADD;
		update_aim_flag = ADD;
		update_float_flag = ADD;
		update_supercapacitor_flag = ADD;
		update_int_flag = ADD;
		update_gimbalangle_flag = ADD;
	}
	
	i++;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void Client_info_update(void)
{
	global_fiction = Slaver_flag.global_fiction;
	global_clip = 1;
	global_spin = Slaver_flag.global_spin;
	global_auto_aim = Slaver_flag.global_auto_aim;
	global_anti_top = 0;
	global_twist = Slaver_flag.global_twist;
	global_supercapacitor_volt = cap_receive_data.cap_Ucr;
	global_supercapacitor_point = 15;
	global_gimbal_pitch = Slaver_flag.pitch;
	global_gimbal_yaw = Slaver_flag.yaw;
	robot_id = judge.info->game_robot_status.robot_id;
	if(robot_id == 1)
	{
		client_id = 0x0101;
	}
	else if(robot_id == 101)
	{
		client_id = 0x0165;
	}
	else
	{
		client_id = 0x0101;
	}
	if(global_supercapacitor_volt > 24.5f)
	{
		global_supercapacitor_volt = 24.5f;
	}
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void Client_graphic_Init()
{
	if(state_first_graphic>=7)
	{
		state_first_graphic = 0;
	}
	//帧头
	tx_client_char.txFrameHeader.sof = JUDGE_FRAME_HEADER;
	tx_client_char.txFrameHeader.data_length = sizeof(ext_client_data_header_t) + sizeof(ext_client_string_t);
	tx_client_char.txFrameHeader.seq = 0;  //包序号
	memcpy(CliendTxBuffer,&tx_client_char.txFrameHeader,sizeof(std_frame_header_t));
	Append_CRC8_Check_Num(CliendTxBuffer, sizeof(std_frame_header_t));//头校验

	//命令码
	tx_client_char.CmdID = ID_interactive_header_data;
	
	//数据段头结构
	tx_client_char.dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
	tx_client_char.dataFrameHeader.send_ID     = robot_id;
	tx_client_char.dataFrameHeader.receiver_ID = client_id;
	
	//数据段
	Draw_char();
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_client_char.CmdID, LEN_CMD_ID+tx_client_char.txFrameHeader.data_length);//加上命令码长度2
	
	//帧尾
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(tx_client_char));
	
	Client_Sent_String(CliendTxBuffer, sizeof(tx_client_char));
}