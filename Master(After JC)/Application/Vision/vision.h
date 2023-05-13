#ifndef __VISION_H
#define __VISION_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include "crc.h"
/* 宏定义与全局变量 ------------------------------------------------------------*/

/*起始字节，协议固定尾0xA5*/
#define VISION_SEND_ID             (0xA5)
#define VISION_OFFLINE_MAX_CNT     (300)
#define VISION_OFFLINE             (0)
#define VISION_ONLINE              (1)

/* 指令与配置 ------------------------------------------------------------*/
typedef enum {
	/*视觉发送模式*/
	CMD_PATROL		        = 0,	// 巡逻模式
	CMD_AUTO     		    = 1,	// 听话模式
	CMD_SIGLE               = 2,    // 单发模式
	CMD_RUNNING             = 3,    // 连发模式
	
	/*电控发送模式*/
	CMD_CHECK               = 2,    // 自瞄测试模式
	
	/*数据模式*/
	CMD_DATA                = 66,
	
	
} vision_cmd_id_t;




/* 数据长度 */
typedef enum {
	/* Std */
	LEN_VISION_RX_PACKET	= 25,	// 接收包整包长度
	LEN_VISION_TX_PACKET	= 18,	// 发送包整包长度

	LEN_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// 接收数据段长度
	LEN_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// 发送数据段长度
	
	LEN_FRAME_HEADER 	  	= 3,	// 帧头长度
	LEN_FRAME_TAILER 		= 2,	// 帧尾CRC16
	
} vision_data_length_t;

typedef enum {
	/* Std */
	J_LEN_VISION_RX_PACKET	= 25,	// 接收包整包长度
	J_LEN_VISION_TX_PACKET	= 18,	// 发送包整包长度

	J_LEN_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// 接收数据段长度
	J_LEN_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// 发送数据段长度
	
	J_LEN_FRAME_HEADER 	  	= 3,	// 帧头长度
	J_LEN_FRAME_TAILER 		= 2,	// 帧尾CRC16
	

} Jud2Vis_data_length_t;


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
	
	uint8_t   shoot_speed;
	uint8_t   my_color;
	/*比赛信息*/
	uint8_t   game_progress;
	
}vision_tx_data_t;

/*低频发送数据*/
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
	
	uint8_t   shoot_speed;
	uint8_t   my_color;

	/*比赛信息*/
	uint8_t   game_progress;
	
}Jud2Vis_tx_data_t;


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
//	uint8_t      L_shoot_cnt;
    uint8_t      L_is_find_target;
	
	/*右云台*/
	uint16_t     R_yaw_angle;
    uint16_t     R_pitch_angle;
	
	uint8_t      R_shoot_speed;
//	uint8_t      R_shoot_cnt;
    uint8_t      R_is_find_target;
	
	uint16_t     B_yaw_angle;
	
}vision_rx_data_t;

/* 发送包格式 */
typedef __packed struct
{
	vision_frame_header_t   FrameHeader;	// 帧头
	Jud2Vis_tx_data_t	    TxData;		    // 数据
	vision_frame_tailer_t   FrameTailer;	// 帧尾	
}vision_tx_packet_t;

/* 发送包格式 */
typedef __packed struct
{
	vision_frame_header_t   FrameHeader;	// 帧头
	vision_tx_data_t	    TxData;		    // 数据
	vision_frame_tailer_t   FrameTailer;	// 帧尾	
}Jud2Vis_tx_packet_t;

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
	
	uint8_t           work_state;
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
