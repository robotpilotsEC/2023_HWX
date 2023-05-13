#include "vision.h"
#include "bmi.h"
#include "can_protocol.h"
#include "rp_shoot.h"
#include "rp_gimbal.h"
#include "judge.h"
#include "Car.h"

/*ʹ�ô���3*/
extern UART_HandleTypeDef huart3;//ʹ�ô���3

extern judge_t          judge;
extern Master_Head_t    Master_Head_structure;
extern bmi_t            bmi_structure;
extern car_t            car_structure;

vision_rx_packet_t      vision_rx_pack;
vision_tx_packet_t      vision_tx_pack;
vision_t                vision_structure;

Jud2Vis_tx_packet_t     Jud2Vis_rx_pack;


uint8_t                 vision_txBuf[20]; 


/**
  * @Name    VISION_INIT
  * @brief   �Ӿ����ֽṹ���ʼ��
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
void VISION_INIT(void)
{
	/*�ṹ���ʼ��*/
	vision_structure.rx_pack = &vision_rx_pack;
	vision_structure.tx_pack = &vision_tx_pack;
	
	/*���ݳ�ʼ��*/
	vision_structure.tx_pack->FrameHeader.sof   = VISION_SEND_ID;
	vision_structure.tx_pack->FrameHeader.cmd_id= CMD_PATROL;
	
	/*����״̬��ʼ��*/
	vision_structure.state.offline_max_cnt = VISION_OFFLINE_MAX_CNT;
	vision_structure.state.offline_cnt     = VISION_OFFLINE_MAX_CNT;
	vision_structure.state.work_state      = VISION_OFFLINE;
	
	Jud2Vis_rx_pack.FrameHeader.sof = VISION_SEND_ID;
	Jud2Vis_rx_pack.FrameHeader.cmd_id = CMD_DATA;
	
	
}


/**
  * @Name    VISION_SEND_DATA
  * @brief   ��С����ͨѶ��CRCУ�����ݿ϶�Ҫ��
  * @param   None                              
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
bool VISION_SEND_DATA(void)
{
	/*�ж��Ƿ����*/
	if(vision_structure.state.offline_cnt++ >= vision_structure.state.offline_max_cnt)
	{
		vision_structure.state.offline_cnt--;
		vision_structure.state.work_state = VISION_OFFLINE;
	}
	else
	{
		vision_structure.state.work_state = VISION_ONLINE;
	}
	
	/*���ݷ���*/	
	memcpy(vision_txBuf, &vision_tx_pack, sizeof(vision_tx_packet_t));
	
	/*����CRCУ��λ*/
	Append_CRC8_Check_Sum(vision_txBuf, LEN_FRAME_HEADER);
	Append_CRC16_Check_Sum(vision_txBuf, LEN_VISION_TX_PACKET);
	
	
	if(HAL_UART_Transmit_DMA(&huart3,vision_txBuf,sizeof(vision_tx_packet_t)) == HAL_OK)
	{
		memset(&vision_tx_pack,sizeof(vision_tx_pack),0);
		return true;
	}
	else
	{
		memset(&vision_tx_pack,sizeof(vision_tx_pack),0);
		return false;
	}
}


/**
  * @Name    VISION_GET_DATA
  * @brief   �������ݣ�һ����CRCУ��Ĳ�������Ҫ��
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
bool VISION_GET_DATA(uint8_t *rxBuf)
{

	
	if(rxBuf[0] == 0xA5)
	{
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEADER) == true)
		{
			if(Verify_CRC16_Check_Sum(rxBuf,LEN_VISION_RX_PACKET) == true)
			{
				memcpy(&vision_rx_pack, rxBuf, LEN_VISION_RX_PACKET);
				

				/*���߼���������*/
				vision_structure.state.offline_cnt = 0;
				return true;
			}
		}
	}
	return false;
}

/**
  * @Name    VISION_UPDATE
  * @brief   ��������
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void VISION_UPDATE(void)
{
	vision_rx_data_t *data = &vision_structure.rx_pack->RxData;
	//���ݽ����Ѿ���У�����ʱ�������
		
}

/**
  * @Name    USART3_rxDataHandler
  * @brief   ����3�����жϻص�����
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void USART3_rxDataHandler(uint8_t *rxBuf)
{	
	VISION_GET_DATA(rxBuf);
}


void Vision_Heart()
{
	if(vision_structure.state.offline_cnt++ >= vision_structure.state.offline_max_cnt)
	{
		vision_structure.state.offline_cnt = vision_structure.state.offline_max_cnt;
		vision_structure.state.work_state = VISION_OFFLINE;
	}
	else
	{
		vision_structure.state.work_state = VISION_ONLINE;
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
uint16_t yaaawwwwwww = 0;
bool send_ok;
uint16_t judge_data_cnt = 0;
void vision_task(void)
{	
	
	
	/*�������ݴ���*/
	//if(judge_data_cnt++ % 200 == 1)
	if(0)
	{
		
		judge_data_cnt = 0;
		
		vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_DATA;//66
		
		
//		vision_structure.tx_pack->TxData.R_pitch_angle = judge.base_info->friendly_outposts_HP;
//		vision_structure.tx_pack->TxData.R_pitch_angle <<= 8;
//		vision_structure.tx_pack->TxData.R_pitch_angle |= judge.base_info->remain_bullte;
//		vision_structure.tx_pack->TxData.R_yaw_angle   =  judge.base_info->enemy_outposts_HP;
//		vision_structure.tx_pack->TxData.L_pitch_angle =  judge.base_info->enemy_hero_HP;
//		vision_structure.tx_pack->TxData.L_yaw_angle   =  judge.base_info->enemy_infanry3_HP;
//		vision_structure.tx_pack->TxData.B_yaw_angle   =  judge.base_info->enemy_infanry4_HP;
//		vision_structure.tx_pack->TxData.shoot_speed   =  judge.base_info->enemy_infanry5_HP;
//		vision_structure.tx_pack->TxData.my_color      =  judge.base_info->enemy_Shaobing_HP;
//		vision_structure.tx_pack->TxData.game_progress =  judge.base_info->remain_time;




		vision_structure.tx_pack->TxData.R_pitch_angle = 1;
		vision_structure.tx_pack->TxData.R_pitch_angle <<= 8;
		vision_structure.tx_pack->TxData.R_pitch_angle |= 2;
		vision_structure.tx_pack->TxData.R_yaw_angle   =  3;
		vision_structure.tx_pack->TxData.L_pitch_angle =  4;
		vision_structure.tx_pack->TxData.L_yaw_angle   =  5;
		vision_structure.tx_pack->TxData.B_yaw_angle   =  6;
		vision_structure.tx_pack->TxData.shoot_speed   =  7;
		vision_structure.tx_pack->TxData.my_color      =  8;
		vision_structure.tx_pack->TxData.game_progress =  9;
		
	}
	else
	{
		if(car_structure.mode == aim_CAR)
		{
			vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_CHECK;
			
		}
		else
		{
			vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_PATROL;
		}
		/*�������*/
		//vision_structure.tx_pack->FrameHeader.cmd_id   = CMD_CHECK;
		
		vision_structure.tx_pack->TxData.R_pitch_angle = Master_Head_structure.From_R_Head.measure_pit;
		vision_structure.tx_pack->TxData.R_yaw_angle   = Master_Head_structure.From_R_Head.measure_yaw;
		vision_structure.tx_pack->TxData.L_pitch_angle = Master_Head_structure.From_L_Head.measure_pit;
		vision_structure.tx_pack->TxData.L_yaw_angle   = Master_Head_structure.From_L_Head.measure_yaw;
		vision_structure.tx_pack->TxData.B_yaw_angle   = bmi_structure.yaw_angle;
		vision_structure.tx_pack->TxData.shoot_speed   = 30;
		vision_structure.tx_pack->TxData.my_color      = judge.base_info->car_color;
		vision_structure.tx_pack->TxData.game_progress = judge.base_info->game_progress;
	}
	
	/*�Ӿ����ߴ���*/
	if(vision_structure.state.work_state == VISION_OFFLINE)	
	{
		vision_structure.rx_pack->RxData.L_shoot_speed = 0;
		vision_structure.rx_pack->RxData.R_shoot_speed = 0;
	}
	

	send_ok = VISION_SEND_DATA();

}




