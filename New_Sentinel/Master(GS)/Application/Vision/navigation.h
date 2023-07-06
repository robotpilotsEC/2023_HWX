#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include "crc.h"
#include "vision.h"
/* �궨����ȫ�ֱ��� ------------------------------------------------------------*/


/* ָ�������� ------------------------------------------------------------*/



/* ���ݳ��� */
typedef enum {
	/* Std */
	LEN_NAVIGATION_RX_PACKET	= 25,	// ���հ���������
	LEN_NAVIGATION_TX_PACKET	= 	18 + 8 + 1 + 1,	// ���Ͱ���������

	LEN_NAVIGATION_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// �������ݶγ���
	LEN_NAVIGATION_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// �������ݶγ���
	
	LEN_NAVIGATION_FRAME_HEADER 	  	= 3,	// ֡ͷ����
	LEN_NAVIGATION_FRAME_TAILER 		= 2,	// ֡βCRC16
	
}navigation_data_length_t;


/* ���ݰ���ʽ ------------------------------------------------------------*/


/* �������ݶθ�ʽ */
typedef __packed struct 
{
	/*����̨*/
    int16_t     L_yaw_angle;
    int16_t     L_pitch_angle;
	
	/*����̨*/
	int16_t     R_yaw_angle;
    int16_t     R_pitch_angle;
	
	/*��YAW��*/
	int16_t     B_yaw_angle;

	uint8_t   my_color;
	/*������Ϣ*/
	uint8_t   game_progress;
	
	/*��������*/
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

/*�����Ӿ�����ͻȻ���ߵ�׼��*/
typedef __packed struct 
{
	int16_t   chassis_front; //����ǰ���ٶ� 
	int16_t   chassis_right; //����ƽ���ٶ�
	int16_t   chassis_cycle; //������ת�ٶ�
	
	/*����̨*/
	int16_t      L_yaw_angle;
    int16_t      L_pitch_angle;
	
	uint8_t      L_shoot_speed;
    uint8_t      L_is_find_target;
	
	/*����̨*/
	uint16_t     R_yaw_angle;
    uint16_t     R_pitch_angle;
	
	uint8_t      R_shoot_speed;
    uint8_t      R_is_find_target;
	
	uint16_t     B_yaw_angle;
	
}navigation_rx_data_t;

/* ���Ͱ���ʽ */
typedef __packed struct
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	navigation_tx_data_t	TxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
}navigation_tx_packet_t;

/* ���հ���ʽ */
typedef __packed struct 
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	navigation_rx_data_t	RxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
} navigation_rx_packet_t;


/* ����ģʽ ------------------------------------------------------------*/
/* ���� ------------------------------------------------------------*/

typedef  struct 
{
	navigation_rx_packet_t *rx_pack;
	navigation_tx_packet_t *tx_pack;
	vision_state_t          state;
	
	
}navigation_t;

/* ��غ��� ------------------------------------------------------------*/

void Navigation_Init(void);
void Navigation_Update(void);

bool Navigation_SendData(void);
bool Navigation_GetData(uint8_t *rxBuf);

void Navigation_task(void);

#endif
