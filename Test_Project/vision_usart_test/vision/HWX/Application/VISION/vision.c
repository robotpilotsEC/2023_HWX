#include "vision.h"
#include "bmi.h"
#include "my_gimbal.h"



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
	vision_structure.tx_pack->FrameHeader.cmd_id=CMD_AIM_OFF;
	
	
	
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
				memcpy(&vision_rx_pack, rxBuf, 16);
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
//	//数据接收已经在校验完成时便结束了
//	
//	if(data->pitch_angle == NULL)data->pitch_angle = bmi_structure.pit_angle;  
//	if(data->yaw_angle   == NULL)data->yaw_angle   = bmi_structure.yaw_angle;
//	
//	if(data->pitch_angle <= 8191 || data->pitch_angle >= 0)data->pitch_angle = data->pitch_angle;
//	
//	else                                                   data->pitch_angle = bmi_structure.pit_angle;
//	
//	if(data->yaw_angle   <= 8191 || data->yaw_angle   >= 0)data->yaw_angle   = data->yaw_angle  ;
//	
//	else                                                   data->yaw_angle   = bmi_structure.yaw_angle;
		
}

/**
  * @Name    USART3_rxDataHandler
  * @brief   串口3接收中断回调函数
  * @param   None
  * @retval
  * @author  HWX
  * @Date    2022-10-21
**/
int16_t ccnt = 0;
int16_t cccnt = 0;
void USART3_rxDataHandler(uint8_t *rxBuf)
{	
	VISION_GET_DATA(rxBuf);
	
	if(ccnt++ >= 100)
	{
		//HAL_GPIO_TogglePin(GPIOC,LED_BLUE_Pin);
		LED_ORANGE_TOGGLE();
		ccnt = 0;
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
void vision_task(void)
{	
	vision_tx_pack.TxData.pitch_angle    = PIT_MIDDLE;
	
	vision_tx_pack.TxData.yaw_angle      = 1234;
	
	vision_tx_pack.TxData.is_find_dafu   = 1;
	vision_tx_pack.TxData.is_find_target = 0;
	vision_tx_pack.TxData.is_hit_enable  = 1;
	if(yaaawwwwwww >= 8191)
	{
		yaaawwwwwww = 0;
	}
	send_ok = VISION_SEND_DATA();
	if (cccnt++ > 1000)
	{
		//HAL_GPIO_TogglePin(GPIOC,LED_ORANGE_Pin);
		cccnt = 0;
		LED_RED_TOGGLE();
	}
}



#if 0
void vision_init(vision_sensor_t *vision)
{
	// 初始化为离线状态
	vision->info->State.offline_cnt = vision->info->State.offline_max_cnt + 1;
	vision->work_state = DEV_OFFLINE;
	
	if(vision->id == DEV_ID_VISION)
		vision->errno = NONE_ERR;
	else
		vision->errno = DEV_ID_ERR;	
}


/**
 *	@brief	视觉数据解析协议
 */

void vision_update(vision_sensor_t *vision_sen, uint8_t *rxBuf)
{
	vision_info_t *vision_info = vision_sen->info;

	uint8_t res = false;
	vision_info->State.rx_cnt++;
	/* 帧首字节是否为0xA5 */
	if(rxBuf[sof] == VISION_FRAME_HEADER) 
	{	
		res = Verify_CRC8_Check_Sum( rxBuf, LEN_FRAME_HEADER );
		/* 帧头CRC8校验*/
		if(res == true)
		{
			res = Verify_CRC16_Check_Sum( rxBuf, LEN_VISION_RX_PACKET );
			/* 帧尾CRC16校验 */
			if(res == true) 
			{
				/* 数据正确则拷贝接收包 */
				memcpy(vision_info->RxPacket.RxData.data, rxBuf, LEN_VISION_RX_PACKET);
				vision_info->State.rx_data_update = true;	// 视觉数据更新	
				{
						memcpy((void*)(vision_info->RxPacket.RxData.jiesuan),vision_info->RxPacket.RxData.data+3,LEN_VISION_RX_PACKET-5);
				}
				/* 帧率计算 */
				vision_info->State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_info->State.rx_time_fps  = vision_info->State.rx_time_now - vision_info->State.rx_time_prev;
				vision_info->State.rx_time_prev = vision_info->State.rx_time_now;		
				vision_info->State.offline_cnt  = 0;
			}
		}
	}	
	/* 数据有效性判断 */
	if(res == true) 
	{
		vision_info->State.rx_data_valid = true;
	} 
	else if(res == false) 
	{
		vision_info->State.rx_data_valid = false;
		vision_info->State.rx_err_cnt++;
	}
//	radar_Info_store();//雷达
}
#endif
