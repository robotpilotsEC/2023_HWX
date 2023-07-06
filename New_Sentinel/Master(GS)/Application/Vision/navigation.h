#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include "crc.h"
#include "vision.h"
/* 宏定义与全局变量 ------------------------------------------------------------*/


/* 指令与配置 ------------------------------------------------------------*/



/* 数据长度 */
typedef enum {
	/* Std */
	LEN_NAVIGATION_RX_PACKET	= 25,	// 接收包整包长度
	LEN_NAVIGATION_TX_PACKET	= 	18 + 8 + 1 + 1,	// 发送包整包长度

	LEN_NAVIGATION_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// 接收数据段长度
	LEN_NAVIGATION_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// 发送数据段长度
	
	LEN_NAVIGATION_FRAME_HEADER 	  	= 3,	// 帧头长度
	LEN_NAVIGATION_FRAME_TAILER 		= 2,	// 帧尾CRC16
	
}navigation_data_length_t;


/* 数据包格式 ------------------------------------------------------------*/


/* 发送数据段格式 */
typedef __packed struct 
{
	/*左云台*/
    int16_t     L_yaw_angle;
    int16_t     L_pitch_angle;
	
	/*右云台*/
	int16_t     R_yaw_angle;
    int16_t     R_pitch_angle;
	
	/*大YAW轴*/
	int16_t     B_yaw_angle;

	uint8_t   my_color;
	/*比赛信息*/
	uint8_t   game_progress;
	
	/*比赛数据*/
	uint8_t remain_time;
	uint8_t remain_bullet;
	uint8_t friendly_outpose_HP;
	uint8_t my_HP;
	uint8_t enemy_hero_HP;
	uint8_t enemy_infantry3_HP;
	uint8_t enemy_infantry4_HP;
	uint8_t enemy_sentry_HP;
	uint8_t enemy_outpose_HP;
	
	uint8_t engine_hit_enable;
	uint8_t hero_hit_enable;

}navigation_tx_data_t;

/*做好视觉数据突然掉线的准备*/
typedef __packed struct 
{
	int16_t   chassis_front; //底盘前进速度 
	int16_t   chassis_right; //底盘平移速度
	int16_t   chassis_cycle; //底盘旋转速度
	
	/*左云台*/
	int16_t      L_yaw_angle;
    int16_t      L_pitch_angle;
	
	uint8_t      L_shoot_speed;
    uint8_t      L_is_find_target;
	
	/*右云台*/
	uint16_t     R_yaw_angle;
    uint16_t     R_pitch_angle;
	
	uint8_t      R_shoot_speed;
    uint8_t      R_is_find_target;
	
	uint16_t     B_yaw_angle;
	
}navigation_rx_data_t;

/* 发送包格式 */
typedef __packed struct
{
	vision_frame_header_t   FrameHeader;	// 帧头
	navigation_tx_data_t	TxData;		    // 数据
	vision_frame_tailer_t   FrameTailer;	// 帧尾	
}navigation_tx_packet_t;

/* 接收包格式 */
typedef __packed struct 
{
	vision_frame_header_t   FrameHeader;	// 帧头
	navigation_rx_data_t	RxData;		    // 数据
	vision_frame_tailer_t   FrameTailer;	// 帧尾	
} navigation_rx_packet_t;


/* 工作模式 ------------------------------------------------------------*/
/* 汇总 ------------------------------------------------------------*/

typedef  struct 
{
	navigation_rx_packet_t *rx_pack;
	navigation_tx_packet_t *tx_pack;
	vision_state_t          state;
	
	
}navigation_t;

/* 相关函数 ------------------------------------------------------------*/

void Navigation_Init(void);
void Navigation_Update(void);

bool Navigation_SendData(void);
bool Navigation_GetData(uint8_t *rxBuf);

void Navigation_task(void);

#endif
