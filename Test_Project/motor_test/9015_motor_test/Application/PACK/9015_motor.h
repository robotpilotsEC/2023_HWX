 
#ifndef __9015_MOTOR_H_
#define __9015_MOTOR_H_

#include "stm32f4xx_hal.h"
#include "drv_can.h"


#if 0
typedef enum
{
	
	PID_RX_ID			    = 0x30,  // ��ȡPID����
	
	PID_TX_RAM_ID			= 0x31,  // д��PID������RAM
	PID_TX_ROM_ID			= 0x32,  // д��PID������ROM
	
	ACCEL_RX_ID				= 0x33,  // ��ȡ���ٶ�����
	ACCEL_TX_ID				= 0x34,  // д����ٶ�����
	
	ENCODER_RX_ID			= 0x90,  // ��ȡ����������
	
	ZERO_ENCODER_TX_ID		= 0x91,	 //�������
	ZERO_POSNOW_TX_ID		= 0x19,	 //�������
	
	MOTOR_ANGLE_ID			= 0x92,  //��ȡ�ܽǶ�ֵ
	CIRCLE_ANGLE_ID			= 0x94,  //��ȡ��Ȧ�Ƕ�ֵ
	
	CLEAR_ANGLE_ID          = 0x95,  //���õ����ʼλ�ã��Ƕ����㣿��
	
	STATE1_ID				= 0x9A,  //��ȡ���״̬1�ʹ����־����
    STATE2_ID				= 0x9C,  //��ȡ���״̬2�ʹ����־����
	STATE3_ID				= 0x9D,  //��ȡ���״̬3�ʹ����־����
	
	CLEAR_ERROR_State_ID	= 0x9B,  //��������������
	
	MOTOR_STOP_ID			= 0x80,  //����ر�����
	MOTOR_CLOSE_ID			= 0x81,  //���ֹͣ����
	MOTOR_RUN_ID			= 0x88,  //�����������
	
	TORQUE_CRTL_ID          = 0xA0,   //Ť�ƿ�������
	TORQUE_LOOP_CRTL_ID     = 0xA1,   //Ť�رջ�����
	SPEED_LOOP_CRTL_ID      = 0xA2,   //Ť�رջ�����
	
	POSITION_1              = 0xA3,  //λ�ñջ�1
	POSITION_2              = 0xA4,  //λ�ñջ�2
	POSITION_3              = 0xA5,  //λ�ñջ�3
	POSITION_4              = 0xA6,  //λ�ñջ�4
	POSITION_5              = 0xA7,  //λ�ñջ�5
	POSITION_6              = 0xA8,  //λ�ñջ�6

	
	
	
} motor_9015_cmd_e;



typedef struct motor_9025_pid_rx_info_t
{
	uint8_t angleKp;
	uint8_t angleKi;
	uint8_t speedKp;
	uint8_t speedKi;
	uint8_t iqKp;
	uint8_t iqKi;
	
}motor_9025_pid_rx_info_t;

typedef struct motor_9025_pid_tx_info_t
{
	uint8_t angleKp;
	uint8_t angleKi;
	uint8_t speedKp;
	uint8_t speedKi;
	uint8_t iqKp;
	uint8_t iqKi;
	
}motor_9025_pid_tx_info_t;

typedef struct motor_9025_pid_info_t
{
	motor_9025_pid_rx_info_t rx;
	motor_9025_pid_tx_info_t tx;
	
}motor_9025_pid_info_t;

typedef struct
{
	
	int32_t	  accel;	//���ٶ�
	int16_t   speed;            
	int16_t   current;          
	int16_t   temperature;            
	int16_t   target_prm; 			
	int16_t   target_angle;
	int16_t   gap_angle;
	
}motor_9015_rx_info_t;


typedef struct
{
	
	int32_t	  accel;	//���ٶ�
	int16_t   speed;            
	int16_t   current;          
	int16_t   temperature;            
	int16_t   target_prm; 			
	int16_t   target_angle;
	int16_t   gap_angle;
	
}motor_9015_tx_info_t;

typedef struct
{
	
	motor_9015_tx_info_t  tx_data;
	motor_9015_rx_info_t  rx_data;
	
	
}motor_9015_base_info_t;


typedef struct 
{
	motor_9015_base_info_t base_info;
	motor_9025_pid_info_t  pid_info;

}motor_9015_t;

typedef enum
{
	MOTOR_1 = 0x141,
	MOTOR_2 = 0x142,
	MOTOR_3 = 0x143,
	
} MO_9015_id_e;



void Motor_9015_Speed_Ctrl();
void Motor_9015_Get_Data (motor_9015_t * motor , uint8_t *rxBuf );
void Motor_9015_Send_Data(motor_9015_t * motor , motor_9015_cmd_e ID);



#endif



















#endif


 

