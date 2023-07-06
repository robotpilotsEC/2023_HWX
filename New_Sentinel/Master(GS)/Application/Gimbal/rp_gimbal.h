/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v2.0		
  * @Author     : hwx			
  * @Date       : 2023-7-06         
  * @Description:    
  *
  *
  ******************************************************************************
 */
#ifndef __RP_GIMBAL_H
#define __RP_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "drv_can.h"
#include "can.h"
#include "9015_motor.h"
#include "can_protocol.h"

/* Private macro -------------------------------------------------------------*/
#define OFFLINE_TIME_MAX   25

#define MOROT_9015_MIDDLE (26224)
#define AMMO_1_ANGLE (MOROT_9015_MIDDLE)
#define AMMO_2_ANGLE (MOROT_9015_MIDDLE + 16384)
#define AMMO_3_ANGLE (MOROT_9015_MIDDLE + 32768)
#define AMMO_4_ANGLE (MOROT_9015_MIDDLE - 16384)

/*输入右头当前角度，限制左头*/
//#define Dynamic_lim_L(x) (output=(int16_t(-0.0007*x*x + 9.34*x - 29470)))

/*输入左头当前角度，限制右头*/
//#define Dynamic_lim_R(x) (output=(int16_t(0.0015*x*x - 1.59*x +6543.9)))

//没头脑
#define Y_MOT_UPP_LIMIT       (3276)   //3250 - 0 - 6200
#define Y_MOT_LOW_LIMIT       (1900)   //3279

#define P_MOT_UPP_LIMIT       (3450)  
#define P_MOT_LOW_LIMIT       (2308)

#define P_Init                (2936)
#define Y_Init                (6742)


//不高兴
#define Y_MOT_UPP_LIMIT_       (405)   //405 - 8192 - 3483
#define Y_MOT_LOW_LIMIT_       (3483)   //3279

#define P_MOT_UPP_LIMIT_       (6200)  
#define P_MOT_LOW_LIMIT_       (4619)

#define P_Init_                (5611)
#define Y_Init_                (8136)


#define Auto_Up 0
#define Auto_Down 1
#define Auto_Left 1
#define Auto_Right 0

#define PITCH_ROTATE_UNIT 7
#define YAW_ROTATE_UNIT 12


/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum 
{
	GIMBAL_OFFLINE = 0,	
	
	GIMBAL_ONLINE  = 1,
	

}gimbal_work_state_e;

typedef struct
{
	
	uint8_t          offline_cnt;
	uint8_t          offline_cnt_max;
	gimbal_work_state_e work_sate;  		/*!< @ref 底盘工作状态 */

}gimbal_work_info_t;

typedef struct
{
	int16_t             target_yaw_angle;
	int16_t             target_pit_angle;
	gimbal_work_state_e staus;
	
}gimbal_head_info_t;

typedef struct gimbal_class_t
{
	motor_9015_t         *Yaw_9015;
	pid_t                 pos_pid;
	int32_t               target_yaw_angle;
	int32_t               target_speed;
	
	gimbal_head_info_t    L_Head;
	gimbal_head_info_t	  R_Head;
	gimbal_work_info_t    staus;
	
}gimbal_t;

/* Function  body --------------------------------------------------------*/
void Gimbal_Init(gimbal_t* gimbal, motor_9015_t* motor );
void Gimbal_BigYawSpeed(gimbal_t* gimbal);
void Gimbal_BigYawPosition(gimbal_t* gimbal);
void Gimbal_BigYawBmiPosition(gimbal_t* gimbal);
void Gimbal_DynamicLimit(gimbal_t* gimbal);
void Gimbal_BigYawAngleCheck(gimbal_t* gimbal);


/*输入右头当前角度，限制左头*/
int16_t Dynamic_lim_L(int16_t x);
/*输入左头当前角度，限制右头*/
int16_t Dynamic_lim_R(int16_t x);

/*电机坐标 2 陀螺仪坐标*/
int16_t MOTOR_9015_TO_BMI(gimbal_t* gimbal, uint16_t ammo_angle);

/*装甲板感知*/
int16_t Hurt_And_Find(void);

/*巡逻动头*/
void Yaw_Auto_R(Master_Head_t* M2H);
void Yaw_Auto_L(Master_Head_t* M2H);
void Pitch_Auto_L(Master_Head_t* M2H);
void Pitch_Auto_R(Master_Head_t* M2H);

#endif /*__RP_GIMBAL_H*/
