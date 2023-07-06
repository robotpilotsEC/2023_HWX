 
#ifndef __9015_MOTOR_H_
#define __9015_MOTOR_H_

#include "stm32f4xx_hal.h"
#include "drv_can.h"


#if 0
typedef enum
{
	
	PID_RX_ID			    = 0x30,  // 读取PID参数
	
	PID_TX_RAM_ID			= 0x31,  // 写入PID参数到RAM
	PID_TX_ROM_ID			= 0x32,  // 写入PID参数到ROM
	
	ACCEL_RX_ID				= 0x33,  // 读取加速度命令
	ACCEL_TX_ID				= 0x34,  // 写入加速度命令
	
	ENCODER_RX_ID			= 0x90,  // 读取编码器命令
	
	ZERO_ENCODER_TX_ID		= 0x91,	 //零点设置
	ZERO_POSNOW_TX_ID		= 0x19,	 //零点设置
	
	MOTOR_ANGLE_ID			= 0x92,  //获取总角度值
	CIRCLE_ANGLE_ID			= 0x94,  //获取单圈角度值
	
	CLEAR_ANGLE_ID          = 0x95,  //设置电机初始位置（角度清零？）
	
	STATE1_ID				= 0x9A,  //读取电机状态1和错误标志命令
    STATE2_ID				= 0x9C,  //读取电机状态2和错误标志命令
	STATE3_ID				= 0x9D,  //读取电机状态3和错误标志命令
	
	CLEAR_ERROR_State_ID	= 0x9B,  //清除电机错误命令
	
	MOTOR_STOP_ID			= 0x80,  //电机关闭命令
	MOTOR_CLOSE_ID			= 0x81,  //电机停止命令
	MOTOR_RUN_ID			= 0x88,  //电机运行命令
	
	TORQUE_CRTL_ID          = 0xA0,   //扭计开环控制
	TORQUE_LOOP_CRTL_ID     = 0xA1,   //扭矩闭环控制
	SPEED_LOOP_CRTL_ID      = 0xA2,   //扭矩闭环控制
	
	POSITION_1              = 0xA3,  //位置闭环1
	POSITION_2              = 0xA4,  //位置闭环2
	POSITION_3              = 0xA5,  //位置闭环3
	POSITION_4              = 0xA6,  //位置闭环4
	POSITION_5              = 0xA7,  //位置闭环5
	POSITION_6              = 0xA8,  //位置闭环6

	
	
	
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
	
	int32_t	  accel;	//加速度
	int16_t   speed;            
	int16_t   current;          
	int16_t   temperature;            
	int16_t   target_prm; 			
	int16_t   target_angle;
	int16_t   gap_angle;
	
}motor_9015_rx_info_t;


typedef struct
{
	
	int32_t	  accel;	//加速度
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


 

