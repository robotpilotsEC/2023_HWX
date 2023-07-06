#include "navigation.h"
#include "vision.h"
#include "bmi.h"
#include "can_protocol.h"
#include "rp_shoot.h"
#include "rp_gimbal.h"
#include "judge.h"
#include "Car.h"

/*使用串口1*/
extern UART_HandleTypeDef huart1;//使用串口1

extern judge_t          judge;
extern Master_Head_t    Master_Head_structure;
extern bmi_t            bmi_structure;
extern car_t            car_structure;

navigation_rx_packet_t      navigation_rx_pack;
navigation_tx_packet_t      navigation_tx_pack;
navigation_t                navigation_structure;


/*发送Buffer*/
uint8_t                 navigation_txBuf[35]; 


/**
  * @Name    Vision_Init
  * @brief   视觉部分结构体初始化
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
void Navigation_Init(void)
{
	/*结构体初始化*/
	navigation_structure.rx_pack = &navigation_rx_pack;
	navigation_structure.tx_pack = &navigation_tx_pack;
	
	/*数据初始化*/
	navigation_structure.tx_pack->FrameHeader.sof   = VISION_SEND_ID;
	navigation_structure.tx_pack->FrameHeader.cmd_id= CMD_PATROL;
	navigation_structure.tx_pack->TxData.engine_hit_enable = 1;
	navigation_structure.tx_pack->TxData.hero_hit_enable = 0;
	
	
	/*工作状态初始化*/
	navigation_structure.state.offline_max_cnt = VISION_OFFLINE_MAX_CNT;
	navigation_structure.state.offline_cnt     = VISION_OFFLINE_MAX_CNT;
	navigation_structure.state.work_state      = VISION_OFFLINE;
	
}


/**
  * @Name    Vision_SendData
  * @brief   与小电脑通讯，CRC校验数据肯定要改
  * @param   None                              
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
bool Navigation_SendData(void)
{
	/*判断是否掉线*/
	if(navigation_structure.state.offline_cnt++ >= navigation_structure.state.offline_max_cnt)
	{
		navigation_structure.state.offline_cnt--;
		navigation_structure.state.work_state = VISION_OFFLINE;
	}
	else
	{
		navigation_structure.state.work_state = VISION_ONLINE;
	}
	
	/*数据发送*/	
	memcpy(navigation_txBuf, &navigation_tx_pack, sizeof(vision_tx_packet_t));
	
	/*增加CRC校验位*/
	Append_CRC8_Check_Sum(navigation_txBuf, LEN_FRAME_HEADER);
	Append_CRC16_Check_Sum(navigation_txBuf, LEN_VISION_TX_PACKET);
	
	
	if(HAL_UART_Transmit_DMA(&huart1,navigation_txBuf,sizeof(vision_tx_packet_t)) == HAL_OK)
	{
		memset(&navigation_tx_pack,sizeof(navigation_tx_pack),0);
		return true;
	}
	else
	{
		memset(&navigation_tx_pack,sizeof(navigation_tx_pack),0);
		return false;
	}
}


/**
  * @Name    Vision_GetData
  * @brief   接收数据，一样的CRC校验的参数可能要改
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
bool Navigation_GetData(uint8_t *rxBuf)
{

	
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEADER) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf,LEN_VISION_RX_PACKET) == true)
			{
				memcpy(&navigation_rx_pack, rxBuf, LEN_VISION_RX_PACKET);

				/*异常数据清除*/
				if(navigation_rx_pack.RxData.chassis_front >= 3000)
				{
					navigation_rx_pack.RxData.chassis_front = 3000; 
				}
				else if(navigation_rx_pack.RxData.chassis_front <= -3000)
				{
					navigation_rx_pack.RxData.chassis_front = -3000; 
				}
				
				if(navigation_rx_pack.RxData.chassis_right >= 3000)
				{
					navigation_rx_pack.RxData.chassis_right = 3000; 
				}
				else if(navigation_rx_pack.RxData.chassis_right <= -3000)
				{
					navigation_rx_pack.RxData.chassis_right = -3000; 
				}
				
				if(navigation_rx_pack.RxData.L_shoot_speed >= 150)
				{
					navigation_rx_pack.RxData.chassis_right = 150; 
				}

				if(navigation_rx_pack.RxData.R_shoot_speed >= 150)
				{
					navigation_rx_pack.RxData.R_shoot_speed = 150	; 
				}
				
				/*掉线计数器清零*/
				navigation_structure.state.offline_cnt = 0;
				return true;
			}
		}
	}
	return false;
}

/**
  * @Name    Vision_Update
  * @brief   解析数据
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void Navigation_Update(void)
{
	//数据接收已经在校验完成时便结束了
		
}

/**
  * @Name    USART1_rxDataHandler
  * @brief   串口1接收中断回调函数
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void USART1_rxDataHandler(uint8_t *rxBuf)
{	
	Navigation_GetData(rxBuf);
}


void Navigation_Heart()
{
	if(navigation_structure.state.offline_cnt++ >= navigation_structure.state.offline_max_cnt)
	{
		navigation_structure.state.offline_cnt = navigation_structure.state.offline_max_cnt;
		navigation_structure.state.work_state = VISION_OFFLINE;
	}
	else
	{
		navigation_structure.state.work_state = VISION_ONLINE;
	}
}

/**
  * @Name    vision_task
  * @brief  
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-24
**/
void Navigation_task(void)
{	
	
	
	/*发送数据处理*/
		if(car_structure.mode == aim_CAR)
		{
			navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_CHECK;
			
		}
		else
		{
			if(judge.base_info->robot_commond == 'Y')//Y
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_A;
			}
			else if(judge.base_info->robot_commond == 'K')//K
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_B;
			}
			else if(judge.base_info->robot_commond == 'U')//U
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_C;
			}
			else if(judge.base_info->robot_commond == 'I')//I
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_D;
			}
			else if(judge.base_info->robot_commond == 'V')//V
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_E;
			}
			else if(judge.base_info->robot_commond == 'H')//H
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_STOP;
			}
			else if(judge.base_info->robot_commond == 'B')//B
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_F;
			}
			else if(judge.base_info->robot_commond == 'O')//O
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_G;
			}
			else if(judge.base_info->robot_commond == 'L')//L
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_H;
			}
			else if(judge.base_info->robot_commond == 'T')//T
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_I;
			}
			else if(judge.base_info->robot_commond == 'G')//G
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_J;
			}
			else if(judge.base_info->robot_commond == 'F')//F
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_POINT_K;
			}
			else
			{
				navigation_structure.tx_pack->FrameHeader.cmd_id   = CMD_STOP;
			}
			
		}

		if(judge.base_info->robot_commond == 'E')//E
		{
			navigation_structure.tx_pack->TxData.engine_hit_enable = 1;
		}else if(judge.base_info->robot_commond == 'R')//R
		{
			navigation_structure.tx_pack->TxData.engine_hit_enable = 0;
		}
		
		if(judge.base_info->robot_commond == 'C')//C
		{
			navigation_structure.tx_pack->TxData.hero_hit_enable = 1;
		}else if(judge.base_info->robot_commond == 'N')//N
		{
			navigation_structure.tx_pack->TxData.hero_hit_enable = 0;
		}
		
		
		navigation_structure.tx_pack->TxData.R_pitch_angle       = Master_Head_structure.From_R_Head.measure_pit;
		navigation_structure.tx_pack->TxData.R_yaw_angle         = Master_Head_structure.From_R_Head.measure_yaw;
		navigation_structure.tx_pack->TxData.L_pitch_angle       = Master_Head_structure.From_L_Head.measure_pit;
		navigation_structure.tx_pack->TxData.L_yaw_angle         = Master_Head_structure.From_L_Head.measure_yaw;
		navigation_structure.tx_pack->TxData.B_yaw_angle         = bmi_structure.yaw_angle;
		navigation_structure.tx_pack->TxData.my_color            = judge.base_info->car_color;
		navigation_structure.tx_pack->TxData.game_progress       = judge.base_info->game_progress;
		
		navigation_structure.tx_pack->TxData.remain_time         = judge.base_info->remain_time;
		navigation_structure.tx_pack->TxData.remain_bullet       = judge.base_info->remain_bullte;
		navigation_structure.tx_pack->TxData.friendly_outpose_HP = judge.base_info->friendly_outposts_HP;
		navigation_structure.tx_pack->TxData.my_HP               = judge.base_info->remain_HP/10;
		navigation_structure.tx_pack->TxData.enemy_hero_HP       = judge.base_info->enemy_hero_HP;
		navigation_structure.tx_pack->TxData.enemy_infantry3_HP  = judge.base_info->enemy_infanry3_HP;
		navigation_structure.tx_pack->TxData.enemy_infantry4_HP  = judge.base_info->enemy_infanry4_HP;
		navigation_structure.tx_pack->TxData.enemy_sentry_HP     = judge.base_info->enemy_Shaobing_HP;
		navigation_structure.tx_pack->TxData.enemy_outpose_HP    = judge.base_info->enemy_outposts_HP;
		
	
	/*视觉掉线处理*/
	if(navigation_structure.state.work_state == VISION_OFFLINE)	
	{
		navigation_structure.rx_pack->RxData.L_shoot_speed = 0;
		navigation_structure.rx_pack->RxData.R_shoot_speed = 0;
		navigation_structure.rx_pack->RxData.chassis_front = 0;
		navigation_structure.rx_pack->RxData.chassis_cycle = 0;
		navigation_structure.rx_pack->RxData.chassis_right = 0;
	}
	
	
	Navigation_SendData();

}




