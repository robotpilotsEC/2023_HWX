/**
  ******************************************************************************
  * @file           : vision.c
  * @brief          : 
  * @update         : finish 2022-2-13 19:34:49
  ******************************************************************************
  */

#include "vision.h"
#include "string.h"
#include "config.h"

vision_t vision = 
{
	.tx_info = &vision_tx_info,
	.rx_info = &vision_rx_info,
};

void vision_init(vision_t *vision)
{
	vision->pitch_err = 0;
	vision->yaw_err = 0;
	vision->status = DEV_OFFLINE;
}

bool vision_send( uint8_t  datau8_1,\
                  uint8_t  datau8_2,\
                  uint8_t  datau8_3,\
                  float    dataf_1, \
                  float    dataf_2, \
                  uint8_t  datau8_4)
{
	memcpy(&vision_tx_info.datau8_1, &datau8_1, 1);
	memcpy(&vision_tx_info.datau8_2, &datau8_2, 1);
	memcpy(&vision_tx_info.datau8_3, &datau8_3, 1);
	memcpy((void*)&vision_tx_info.dataf_1, &dataf_1, 4);
	memcpy((void*)&vision_tx_info.dataf_2, &dataf_2, 4);
	memcpy(&vision_tx_info.datau8_4, &datau8_4, 1);
	if(vision_send_data() == true)
	{
			return true;
	}
	return false;
}

bool vision_recieve(uint8_t  *rxBuf)
{
	if(vision_recieve_data(rxBuf) == true)
	{
//		memcpy(&datau8_1, &Vision_Rx_Info.datau8_1, 1);
		memcpy(&vision.pitch_err, (void*)&vision_rx_info.dataf_1, 4);
		memcpy(&vision.yaw_err, (void*)&vision_rx_info.dataf_2, 4);
//		memcpy(&dataf_3, (void*)&Vision_Rx_Info.dataf_3, 4);
//		memcpy(&datau8_2, &Vision_Rx_Info.datau8_2, 1);
//		memcpy(&datau8_3, &Vision_Rx_Info.datau8_3, 1);
//		memcpy(&datau8_4, &Vision_Rx_Info.datau8_4, 1);
//		memcpy(&datau8_5, &Vision_Rx_Info.datau8_5, 1);
//		memcpy(&datau8_6, &Vision_Rx_Info.datau8_6, 1);
//		memcpy(&datau8_7, &Vision_Rx_Info.datau8_7, 1);
		return true;
   }
   return false;
}
