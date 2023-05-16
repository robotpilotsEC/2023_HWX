#ifndef __VISION_H
#define __VISION_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include "crc.h"
/* �궨����ȫ�ֱ��� ------------------------------------------------------------*/

/*��ʼ�ֽڣ�Э��̶�β0xA5*/
#define VISION_SEND_ID             (0xA5)
#define VISION_OFFLINE_MAX_CNT     (300)
#define VISION_OFFLINE             (0)
#define VISION_ONLINE              (1)

/* ָ�������� ------------------------------------------------------------*/
typedef enum {
	/*�Ӿ�����ģʽ*/
	CMD_PATROL		        = 0,	// Ѳ��ģʽ
	CMD_AUTO     		    = 1,	// ����ģʽ
	CMD_SIGLE               = 2,    // ����ģʽ
	CMD_RUNNING             = 3,    // ����ģʽ
	
	/*��ط���ģʽ*/
	CMD_CHECK               = 2,    // �������ģʽ
	
	/*����ģʽ*/
	CMD_DATA                = 66,
	
	
} vision_cmd_id_t;




/* ���ݳ��� */
typedef enum {
	/* Std */
	LEN_VISION_RX_PACKET	= 25,	// ���հ���������
	LEN_VISION_TX_PACKET	= 18,	// ���Ͱ���������

	LEN_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// �������ݶγ���
	LEN_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// �������ݶγ���
	
	LEN_FRAME_HEADER 	  	= 3,	// ֡ͷ����
	LEN_FRAME_TAILER 		= 2,	// ֡βCRC16
	
} vision_data_length_t;

typedef enum {
	/* Std */
	J_LEN_VISION_RX_PACKET	= 25,	// ���հ���������
	J_LEN_VISION_TX_PACKET	= 18,	// ���Ͱ���������

	J_LEN_RX_DATA 			= LEN_VISION_RX_PACKET - 5,	// �������ݶγ���
	J_LEN_TX_DATA 			= LEN_VISION_TX_PACKET - 5,	// �������ݶγ���
	
	J_LEN_FRAME_HEADER 	  	= 3,	// ֡ͷ����
	J_LEN_FRAME_TAILER 		= 2,	// ֡βCRC16
	

} Jud2Vis_data_length_t;


/* ���ݰ���ʽ ------------------------------------------------------------*/
/* ֡ͷ��ʽ */
typedef __packed struct
{
	uint8_t  			    sof;		// ͬ��ͷ
	vision_cmd_id_t  	    cmd_id;	    // ������
	uint8_t  			    crc8;		// CRC8У����
} vision_frame_header_t;

/* ֡β��ʽ */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16У����
} vision_frame_tailer_t;


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
	
	uint8_t   shoot_speed;
	uint8_t   my_color;
	/*������Ϣ*/
	uint8_t   game_progress;
	
}vision_tx_data_t;

/*��Ƶ��������*/
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
	
	uint8_t   shoot_speed;
	uint8_t   my_color;

	/*������Ϣ*/
	uint8_t   game_progress;
	
}Jud2Vis_tx_data_t;


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
//	uint8_t      L_shoot_cnt;
    uint8_t      L_is_find_target;
	
	/*����̨*/
	uint16_t     R_yaw_angle;
    uint16_t     R_pitch_angle;
	
	uint8_t      R_shoot_speed;
//	uint8_t      R_shoot_cnt;
    uint8_t      R_is_find_target;
	
	uint16_t     B_yaw_angle;
	
}vision_rx_data_t;

/* ���Ͱ���ʽ */
typedef __packed struct
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	Jud2Vis_tx_data_t	    TxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
}vision_tx_packet_t;

/* ���Ͱ���ʽ */
typedef __packed struct
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	vision_tx_data_t	    TxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
}Jud2Vis_tx_packet_t;

/* ���հ���ʽ */
typedef __packed struct 
{
	vision_frame_header_t   FrameHeader;	// ֡ͷ
	vision_rx_data_t	    RxData;		    // ����
	vision_frame_tailer_t   FrameTailer;	// ֡β	
} vision_rx_packet_t;


/* ����ģʽ ------------------------------------------------------------*/

/*����ģʽ*/
typedef enum
{
	VISION_MODE_MANUAL		  = 0,	// �ֶ�ģʽ
	VISION_MODE_AUTO		  = 1,	// ����ģʽ
	VISION_MODE_BIG_BUFF	  = 2,	// ����ģʽ
	VISION_MODE_SMALL_BUFF	  = 3,	// ��С��ģʽ
} Vision_Mode_t;

/* ������ʶ���� */
typedef struct
{
	uint8_t 		  my_color;			 // ��0/1��ʾ��ɫ
	Vision_Mode_t	  mode;				 // �Ӿ�ģʽ
	uint8_t  		  rx_data_valid;     // �������ݵ���ȷ��
	uint16_t 		  rx_err_cnt;		 // �������ݵĴ���ͳ��
	uint32_t		  rx_cnt;		     // �������ݰ���ͳ��
	bool		      rx_data_update;    // ���������Ƿ����
	uint32_t 		  rx_time_prev;	     // �������ݵ�ǰһʱ��
	uint32_t 		  rx_time_now;	     // �������ݵĵ�ǰʱ��
	uint16_t 		  rx_time_fps;	     // ֡��
	
	uint8_t           work_state;
	int16_t		      offline_cnt;
	int16_t		      offline_max_cnt;
	
} vision_state_t;
/* ���� ------------------------------------------------------------*/

typedef  struct 
{
	vision_rx_packet_t *rx_pack;
	vision_tx_packet_t *tx_pack;
	vision_state_t      state;
	
	
}vision_t;

/* ��غ��� ------------------------------------------------------------*/
void VISION_INIT(void);
void VISION_UPDATE(void);

bool VISION_SEND_DATA(void);
bool VISION_GET_DATA(uint8_t *rxBuf);

void vision_task(void);

#endif
