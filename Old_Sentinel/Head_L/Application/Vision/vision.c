#include "vision.h"
#include "bmi.h"



/*使用串口3*/
extern UART_HandleTypeDef huart3;//使用串口3

extern bmi_t            bmi_structure;

vision_rx_packet_t      vision_rx_pack;
vision_tx_packet_t      vision_tx_pack;
vision_t                vision_structure;

uint8_t                 vision_txBuf[30]; //uint会不会有问题 大小也不确定


/**
  * @Name    VISION_INIT
  * @brief   视觉部分结构体初始化
  * @param   None
  * @retval  
  * @author  HWX
  * @Date    2022-10-21
**/
void VISION_INIT(void)
{
	/*结构体初始化*/
	vision_structure.rx_pack = &vision_rx_pack;
	vision_structure.tx_pack = &vision_tx_pack;
	
	/*数据初始化*/
	vision_structure.tx_pack->FrameHeader.sof   = VISION_SEND_ID;
	vision_structure.tx_pack->FrameHeader.cmd_id= CMD_AIM_OFF;
	
	
	
}


/**
  * @Name    VISION_SEND_DATA
  * @brief   与小电脑通讯，CRC校验数据肯定要改
  * @param   None                              
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
bool VISION_SEND_DATA(void)
{
	memcpy(vision_txBuf, &vision_tx_pack, sizeof(vision_tx_packet_t));
	
	Append_CRC8_Check_Sum(vision_txBuf, LEN_FRAME_HEADER);//crc数据肯定要改
	Append_CRC16_Check_Sum(vision_txBuf, LEN_VISION_TX_PACKET);
	
	if(HAL_UART_Transmit_DMA(&huart3,vision_txBuf,sizeof(vision_tx_packet_t)) == HAL_OK)return true;
	return false;
}


/**
  * @Name    VISION_GET_DATA
  * @brief   接收数据，一样的CRC校验的参数可能要改
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
				return true;
			}
		}
	}
	return false;
}

/**
  * @Name    VISION_UPDATE
  * @brief   解析数据
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void VISION_UPDATE(void)
{
	vision_rx_data_t *data = &vision_structure.rx_pack->RxData;
	//数据接收已经在校验完成时便结束了
		
}

/**
  * @Name    USART3_rxDataHandler
  * @brief   串口3接收中断回调函数
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
void USART3_rxDataHandler(uint8_t *rxBuf)
{	
	VISION_GET_DATA(rxBuf);
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
void vision_task(void)
{	
	//vision_tx_pack.TxData.pitch_angle    = PIT_MIDDLE;
	
	vision_tx_pack.TxData.yaw_angle      = yaaawwwwwww++;
	
	vision_tx_pack.TxData.is_find_dafu   = 0;
	vision_tx_pack.TxData.is_find_target = 0;
	vision_tx_pack.TxData.is_hit_enable  = 0;
	if(yaaawwwwwww >= 8191)
	{
		yaaawwwwwww = 0;
	}
	send_ok = VISION_SEND_DATA();
}




