#ifndef __VISION_H
#define __VISION_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include "crc.h"
/* 宏定义与全局变量 ------------------------------------------------------------*/

/*起始字节，协议固定尾0xA5*/
#define VISION_SEND_ID 0xA5


/* 指令与配置 ------------------------------------------------------------*/
typedef enum {
	CMD_AIM_OFF 		    = 0x00,	// 不启动自瞄
	CMD_AIM_AUTO		    = 0x01,	// 启动自瞄
	CMD_AIM_SMALL_BUFF	    = 0x02,	// 识别小符
	CMD_AIM_BIG_BUFF	    = 0x03,	// 识别大符
	CMD_AIM_ANTOP	   	    = 0x04,	// 击打哨兵
	CMD_AIM_ANDF		    = 0x05,	// 吊射基地
} vision_cmd_id_t;


/* 数据长度 */
typedef enum {
	/* Std */
	LEN_VISION_RX_PACKET	= 16,	// 接收包整包长度
	LEN_VISION_TX_PACKET	= 15,	// 发送包整包长度

	LEN_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// 接收数据段长度
	LEN_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// 发送数据段长度
	
	LEN_FRAME_HEADER 	  	= 3,	// 帧头长度
	LEN_FRAME_TAILER 		= 2,	// 帧尾CRC16
} vision_data_length_t;


/* 数据包格式 ------------------------------------------------------------*/
/* 帧头格式 */
typedef __packed struct
{
	uint8_t  			    sof;		// 同步头
	vision_cmd_id_t  	    cmd_id;	    // 命令码
	uint8_t  			    crc8;		// CRC8校验码
} vision_frame_header_t;

/* 帧尾格式 */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16校验码
} vision_frame_tailer_t;


/* 接收数据段格式 */
typedef __packed struct 
{
    float     pitch_angle;
    float     yaw_angle;
	
    /*以下内容不重要，但包内必须有*/
    uint8_t   is_find_target;
    uint8_t   is_find_dafu;
    uint8_t   is_hit_enable;
    /*以上内容不重要，但包内必须有*/
	
}vision_tx_data_t;

typedef __packed struct 
{

    float     yaw_angle;
    float     pitch_angle;
    /*以下内容不重要，但包内必须有*/
    uint8_t   shoot_speed;
    uint8_t   my_color;
    /*以上内容不重要，但包内必须有*/
    //uint16_t  CRC16;
	
}vision_rx_data_t;

/* 发送包格式 */
typedef __packed struct
{
	vision_frame_header_t   FrameHeader;	// 帧头
	vision_tx_data_t	    TxData;		    // 数据
	vision_frame_tailer_t   FrameTailer;	// 帧尾	
}vision_tx_packet_t;

/* 接收包格式 */
typedef __packed struct 
{
	vision_frame_header_t   FrameHeader;	// 帧头
	vision_rx_data_t	    RxData;		    // 数据
	vision_frame_tailer_t   FrameTailer;	// 帧尾	
} vision_rx_packet_t;


/* 工作模式 ------------------------------------------------------------*/

/*工作模式*/
typedef enum
{
	VISION_MODE_MANUAL		  = 0,	// 手动模式
	VISION_MODE_AUTO		  = 1,	// 自瞄模式
	VISION_MODE_BIG_BUFF	  = 2,	// 打大符模式
	VISION_MODE_SMALL_BUFF	  = 3,	// 打小符模式
} Vision_Mode_t;

/* 辅助标识变量 */
typedef struct
{
	uint8_t 		  my_color;			 // 用0/1表示颜色
	Vision_Mode_t	  mode;				 // 视觉模式
	uint8_t  		  rx_data_valid;     // 接收数据的正确性
	uint16_t 		  rx_err_cnt;		 // 接收数据的错误统计
	uint32_t		  rx_cnt;		     // 接收数据包的统计
	bool		      rx_data_update;    // 接收数据是否更新
	uint32_t 		  rx_time_prev;	     // 接收数据的前一时刻
	uint32_t 		  rx_time_now;	     // 接收数据的当前时刻
	uint16_t 		  rx_time_fps;	     // 帧率
	
	int16_t		      offline_cnt;
	int16_t		      offline_max_cnt;	
} vision_state_t;
/* 汇总 ------------------------------------------------------------*/

typedef  struct 
{
	vision_rx_packet_t *rx_pack;
	vision_tx_packet_t *tx_pack;
	vision_state_t      state;
	
	
}vision_t;

/* 相关函数 ------------------------------------------------------------*/
void VISION_INIT(void);
void VISION_UPDATE(void);

bool VISION_SEND_DATA(void);
bool VISION_GET_DATA(uint8_t *rxBuf);

void vision_task(void);

#endif
