#include "can_protocol.h"

Master_Head_t Master_Head_structure;


/**
 *	@brief	掉线检测
 */
void Master_Head_HEART(Master_Head_t * pack)
{
	if(pack->L_Head_status.offline_cnt ++ >= M2H_OFFLINE_TIME_MAX)
	{
		pack->L_Head_status.status = M2H_OFFLINE;
		pack->L_Head_status.offline_cnt = M2H_OFFLINE_TIME_MAX;
	}
	else
	{
		pack->L_Head_status.status = M2H_ONLINE;
	}
	
	if(pack->R_Head_status.offline_cnt ++ >= M2H_OFFLINE_TIME_MAX)
	{
		pack->R_Head_status.status = M2H_OFFLINE;
		pack->R_Head_status.offline_cnt = M2H_OFFLINE_TIME_MAX;
	}
	else
	{
		pack->R_Head_status.status = M2H_ONLINE;
	}
	
}



/**
 *	@brief	数据包初始化
 */
void Master_Head_INIT(Master_Head_t * pack)
{

	pack->Send_L_Head.target_yaw = 0;
	pack->Send_L_Head.target_pit = 0;
	pack->Send_L_Head.target_pit = M2H_MODE_INIT;
	
	pack->Send_R_Head.target_yaw = 0;
	pack->Send_R_Head.target_pit = 0;
	pack->Send_R_Head.target_pit = M2H_MODE_INIT;
	
	pack->L_Head_status.offline_cnt     = M2H_OFFLINE_TIME_MAX;
	pack->L_Head_status.offline_cnt_max = M2H_OFFLINE_TIME_MAX;
	pack->L_Head_status.status          = M2H_OFFLINE;
	
	pack->R_Head_status.offline_cnt     = M2H_OFFLINE_TIME_MAX;
	pack->R_Head_status.offline_cnt_max = M2H_OFFLINE_TIME_MAX;
	pack->R_Head_status.status          = M2H_OFFLINE;
	
	
}

/**
*	@brief	数据信息发送
 */
void M2H_SENT_DATA(Master_Head_t * pack)
{

	uint32_t txMailBox;//发送邮箱
    CAN_TxHeaderTypeDef txFrameHeader;
    uint8_t data[8];
	
    /*数据处理*/
	data[0] = (uint8_t)((int16_t)pack->Send_L_Head.target_yaw >> 8);
	data[1] = (uint8_t)((int16_t)pack->Send_L_Head.target_yaw);
	
	data[2] = (uint8_t)((int16_t)pack->Send_L_Head.target_pit >> 8);
	data[3] = (uint8_t)((int16_t)pack->Send_L_Head.target_pit);
	
	data[4] = (uint8_t)((int16_t)pack->Send_R_Head.target_yaw >> 8);
	data[5] = (uint8_t)((int16_t)pack->Send_R_Head.target_yaw);
	
	data[6] = (uint8_t)((int16_t)pack->Send_R_Head.target_pit >> 8);
	data[7] = (uint8_t)((int16_t)pack->Send_R_Head.target_pit);		
    /*数据帧开头*/
    txFrameHeader.StdId = GIMBAL_DATA_ID;
    txFrameHeader.IDE   = CAN_ID_STD;
    txFrameHeader.RTR   = CAN_RTR_DATA;
    txFrameHeader.DLC   = 0x08;


    /*默认使用CAN1*/
	HAL_CAN_AddTxMessage(&MH_DRV_CAN_USE, &txFrameHeader, data, &txMailBox);

    memset(data,0,sizeof(data));

	
}

/**
*	@brief	数据信息接收解析
 */
void H2M_GETT_DATA( uint32_t id ,Master_Head_t * pack , uint8_t *rxBuf)
{
	switch (id)
	{
		
		case GIMBAL_DATA_ID:
			pack->Send_L_Head.target_yaw = rxBuf[0];
			pack->Send_L_Head.target_yaw <<= 8;
			pack->Send_L_Head.target_yaw |= rxBuf[1];
		
			pack->Send_L_Head.target_pit = rxBuf[2];
			pack->Send_L_Head.target_pit <<= 8;
			pack->Send_L_Head.target_pit |= rxBuf[3];
		
			pack->Send_R_Head.target_yaw = rxBuf[4];
			pack->Send_R_Head.target_yaw <<= 8;
			pack->Send_R_Head.target_yaw |= rxBuf[5];
		
			pack->Send_R_Head.target_pit = rxBuf[6];
			pack->Send_R_Head.target_pit <<= 8;
			pack->Send_R_Head.target_pit |= rxBuf[7];
			
			pack->L_Head_status.offline_cnt = 0;
			pack->R_Head_status.offline_cnt = 0;
			break;
		case MASTER_MODE_ID:
			pack->Send_L_Head.gimbal_mode = rxBuf[4];
			pack->Send_R_Head.gimbal_mode = rxBuf[5];
			pack->Send_L_Head.shoot_mode  = rxBuf[6];
			pack->Send_R_Head.shoot_mode  = rxBuf[7];
			break;

		
		case FROM_L_HEAD_ID:
			pack->From_L_Head.measure_yaw = (int16_t)rxBuf[0];
			pack->From_L_Head.measure_yaw <<= 8;
			pack->From_L_Head.measure_yaw |= (int16_t)rxBuf[1];
		
			pack->From_L_Head.measure_pit = (int16_t)rxBuf[2];
			pack->From_L_Head.measure_pit <<= 8;
			pack->From_L_Head.measure_pit |= (int16_t)rxBuf[3];
		
			pack->From_L_Head.gimbal_mode = (int16_t)rxBuf[6];
			pack->From_L_Head.shoot_mode = (int16_t)rxBuf[7];
		
			pack->L_Head_status.offline_cnt = 0;//掉线清零
			break;
		
		case FROM_R_HEAD_ID:
			
			pack->From_R_Head.measure_yaw = (int16_t)rxBuf[2];
			pack->From_R_Head.measure_yaw <<= 8;
			pack->From_R_Head.measure_yaw |= (int16_t)rxBuf[3];
		
			pack->From_R_Head.measure_pit = (int16_t)rxBuf[4];
			pack->From_R_Head.measure_pit <<= 8;
			pack->From_R_Head.measure_pit |= (int16_t)rxBuf[5];
		
			pack->From_R_Head.gimbal_mode = (int16_t)rxBuf[6];
			pack->From_R_Head.shoot_mode = (int16_t)rxBuf[7];
			
			pack->R_Head_status.offline_cnt = 0;//掉线清零
			break;	
	}
		

}


