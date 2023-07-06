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
	CMD_POINT_A             = 11,   // 前往敌方肥婆点 Y （防肥婆）
	CMD_POINT_B             = 12,   // 前往我方前哨战 K （防英雄打前哨战）
	CMD_POINT_C             = 13,   // 前往巡逻区左上 U （回家1）
	CMD_POINT_D             = 14,   // 前往巡逻区右下 I （回家2）
	CMD_POINT_E             = 15,   // 前往对面前哨战 V （进攻1）
	CMD_STOP                = 16,   // 停止           H
	
	CMD_POINT_F             = 17,   // 前往对面兑换站 B （进攻2）
	CMD_POINT_G             = 18,   // 前往对面资源岛 O （开局打工程
	CMD_POINT_H             = 19,   // 前往对面巡逻区 L （进攻3）
	
	CMD_POINT_I             = 20,   // 前哨战巡逻模式 T
	CMD_POINT_J             = 21,   // 巡逻区巡逻模式 G
	
	CMD_POINT_K             = 22,   // 前往对面肥婆点 F
	
	//由于前往资源岛指令的加入，因此击打工程是默认开启的
	//击打工程 ‘R’ 关闭   ‘E’开启（开局默认开启，可在前往完资源岛后关闭）
	
	//只打英雄功能： 默认关闭
	//C 只打英雄 N关闭
	
	/*电控发送模式*/
	CMD_CHECK               = 2,    // 自瞄测试模式
	
	/*数据模式*/
	CMD_DATA                = 66,
	
	
} vision_cmd_id_t;


/* 数据长度 */
typedef enum {
	/* Std */
	LEN_VISION_RX_PACKET	= 25,	// 接收包整包长度
	LEN_VISION_TX_PACKET	= 	18 + 8 + 1 + 1,	// 发送包整包长度

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

}vision_tx_data_t;

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

void Vision_Init(void);
void Vision_Update(void);

bool Vision_SendData(void);
bool Vision_GetData(uint8_t *rxBuf);

void vision_task(void);

#endif
