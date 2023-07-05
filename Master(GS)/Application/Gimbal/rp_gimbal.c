/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v1.1		
  * @Author     : hwx			
  * @Date       : 2023年2月28日         
  * @Description:    
  *
  *
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rp_gimbal.h"
#include "9015_motor.h"
#include "dji_pid.h"
#include "remote.h"
#include "can_protocol.h"
#include "bmi.h"
/* Exported variables --------------------------------------------------------*/
extern motor_9015_t           motor_9015_structure;
extern rc_t                   rc_structure;
extern pid_t                  motor_9015_pid_spd;
extern pid_t                  motor_9015_pid_pos;
extern bmi_t                  bmi_structure;
/* Function  body --------------------------------------------------------*/
/**
  * @Name    Gimbal_Init
  * @brief   云台初始化
  * @param   
  * @retval 
  * @author  HWX
  * @Date    2023年2月28日
**/
void Gimbal_Init(gimbal_t* gimbal, motor_9015_t* motor )
{
	/*大YAW初始化*/
	gimbal->Yaw_9015 = motor;
	gimbal->target_speed     = 0;
	gimbal->target_yaw_angle = 0;
	gimbal->staus.work_sate  = GIMBAL_OFFLINE;
	
	/*左右头初始化*/
	gimbal->L_Head.staus = GIMBAL_OFFLINE;
	gimbal->L_Head.target_yaw_angle = 0;
	gimbal->L_Head.target_pit_angle = 0;
	
	gimbal->R_Head.staus = GIMBAL_OFFLINE;
	gimbal->R_Head.target_yaw_angle = 0;
	gimbal->R_Head.target_pit_angle = 0;
	
	uint32_t init_cnt = 0;
	while(gimbal->Yaw_9015->base_info->temperature != 0 )
	{
		Motor_9015_Off();
		if(init_cnt ++ >= 100000)
		{
			break;
		}
		
	}
	
	gimbal->target_yaw_angle = gimbal->Yaw_9015->base_info->encoder;
}

/**
  * @Name    Gimbal_BigYawSpeed
  * @brief   云台速度控制
  * @param   
  * @retval
  * @author  HWX
  * @Date    2023年2月28日
**/
void Gimbal_BigYawSpeed(gimbal_t* gimbal)
{
	Motor_9015_HeartBeat();
	
	if(gimbal->staus.work_sate == GIMBAL_ONLINE && \
	   rc_structure.info->status == REMOTE_ONLINE)
	{
		motor_9015_structure.base_info->target_speed = gimbal->target_speed;
		Motor_9015_Speed();
	}
	else
	{
		//卸力
		Motor_9015_Off();
	}
}

/**
  * @Name    Gimbal_BigYawPosition
  * @brief   云台位置控制
  * @param   
  * @retval
  * @author  HWX
  * @Date    2023年2月28日
**/
void Gimbal_BigYawPosition(gimbal_t* gimbal)
{
	Motor_9015_HeartBeat();
	
	if(gimbal->staus.work_sate == GIMBAL_ONLINE && \
	   rc_structure.info->status == REMOTE_ONLINE)
	{	
		motor_9015_structure.base_info->target_angle = gimbal->target_yaw_angle;
		Motor_9015_Position();
	}
	else
	{	
		//卸力
		Motor_9015_Off();
		
	}
}

/**
  * @Name    Gimbal_BigYawPosition
  * @brief   云台位置控制
  * @param   
  * @retval
  * @author  HWX
  * @Date    2023年2月28日
**/
void Gimbal_BigYawBmiPosition(gimbal_t* gimbal)
{
	Motor_9015_HeartBeat();
	
	if(gimbal->staus.work_sate == GIMBAL_ONLINE && \
	   rc_structure.info->status == REMOTE_ONLINE)
	{	
		motor_9015_structure.base_info->target_angle = gimbal->target_yaw_angle;
		Motor_9015_BmiPosition();
	}
	else
	{	
		//卸力
		Motor_9015_Off();
		
	}
}

void Gimbal_DynamicLimit(gimbal_t* gimbal)
{
	int16_t Left  = gimbal->L_Head.target_yaw_angle; //一定是电机的角度
	int16_t Right = gimbal->R_Head.target_yaw_angle; 
	
	/*数据标准化
	
		2048       |       2048
	 0	  L   4096 |  4096   R   0
		 8192      |        8192
	
	*/
	
	/*模式划分 
	正常模式： 范围定死
	超范模式： 动态限幅
	*/
	
	/*输入左头，输入右头的范围*/
	
	
	

}

/*yaw电机 角度*/
void Gimbal_BigYawAngleCheck(gimbal_t* gimbal)
{
	
    if(gimbal->target_yaw_angle >= 65536)//目标判断
    {
        gimbal->target_yaw_angle -= 65536 ;
    } 
	else if(gimbal->target_yaw_angle <= 0)
    {
        gimbal->target_yaw_angle += 65536 ;
    }

}

int16_t test1 = 1;
int16_t test0 = 400;
void Pitch_Auto_R(Master_Head_t* M2H)
{
	static uint8_t direction_p_R = Auto_Up;  //初始巡航方向
	
	if(M2H->Send_R_Head.target_pit > P_MOT_UPP_LIMIT - test1)
	{
		direction_p_R = Auto_Up;
	}
	else if(M2H->Send_R_Head.target_pit < P_MOT_LOW_LIMIT + test0)
	{
		direction_p_R = Auto_Down;
	}
	
	
	switch(direction_p_R)
	{
		case Auto_Up: M2H->Send_R_Head.target_pit -= PITCH_ROTATE_UNIT; break;       
		case Auto_Down: M2H->Send_R_Head.target_pit += PITCH_ROTATE_UNIT; break;
	}

	
}

int16_t test2 = 1;
int16_t test3 = 730;

void Pitch_Auto_L(Master_Head_t* M2H)
{
	static uint8_t direction_p_L = Auto_Up;  //初始巡航方向
	
	if(M2H->Send_L_Head.target_pit > P_MOT_UPP_LIMIT_ - test2)
	{
		direction_p_L = Auto_Up;
	}
	else if(M2H->Send_L_Head.target_pit < P_MOT_LOW_LIMIT_ + test3)
	{
		direction_p_L = Auto_Down;
	}
	switch(direction_p_L)
	{
		case Auto_Up: M2H->Send_L_Head.target_pit -= PITCH_ROTATE_UNIT; break;       
		case Auto_Down: M2H->Send_L_Head.target_pit += PITCH_ROTATE_UNIT; break;
	}
	
}

void Yaw_Auto_L(Master_Head_t* M2H)
{
	static uint8_t direction_L = Auto_Right;  //初始巡航方向
	static uint8_t cnt = 0;        //改变目标值的时间间隔
	
	if(M2H->Send_L_Head.target_yaw >= 405 &&  M2H->Send_L_Head.target_yaw<= 1944)
	{
		direction_L = Auto_Right;
	}
	else if(M2H->Send_L_Head.target_yaw <= 3483 && M2H->Send_L_Head.target_yaw>= 1944)
	{
		direction_L = Auto_Left;
	}

	switch(direction_L)
	{
		case Auto_Left: 
		{
			if(M2H->Send_L_Head.target_yaw > 0 && M2H->Send_L_Head.target_yaw <=450)
			{
				M2H->Send_L_Head.target_yaw += YAW_ROTATE_UNIT; 
			}
			else
			{
				M2H->Send_L_Head.target_yaw += YAW_ROTATE_UNIT; 
			}
			
			break;
		}
		case Auto_Right:
		{				
			if(M2H->Send_L_Head.target_yaw > 0 && M2H->Send_L_Head.target_yaw <=450)
			{
				M2H->Send_L_Head.target_yaw -= YAW_ROTATE_UNIT; 
			}
			else
			{
				M2H->Send_L_Head.target_yaw -= YAW_ROTATE_UNIT; 
			}
			break;
		}
	}
	
}

void Yaw_Auto_R(Master_Head_t* M2H)
{
	static uint8_t direction_R = Auto_Right;  //初始巡航方向
	static uint8_t cnt = 0;        //改变目标值的时间间隔
	
	if( M2H->Send_R_Head.target_yaw >= 3250 && M2H->Send_R_Head.target_yaw <= 4750)
	{
		direction_R = Auto_Left;
	}
	else if(M2H->Send_R_Head.target_yaw  <= 6200 && M2H->Send_R_Head.target_yaw  >= 4750)
	{
		direction_R = Auto_Right;
	}

	switch(direction_R)
	{
		case Auto_Left: 
		{
			if(M2H->Send_R_Head.target_yaw > 0 && M2H->Send_R_Head.target_yaw <=3250)
			{
				M2H->Send_R_Head.target_yaw -= YAW_ROTATE_UNIT; 
			}
			else
			{
				M2H->Send_R_Head.target_yaw -= YAW_ROTATE_UNIT; 
			}
			
			break;
		}
		case Auto_Right:
		{				
			if(M2H->Send_L_Head.target_yaw > 0 && M2H->Send_L_Head.target_yaw <=3250)
			{
				M2H->Send_R_Head.target_yaw += YAW_ROTATE_UNIT; 
			}
			else
			{
				M2H->Send_R_Head.target_yaw += YAW_ROTATE_UNIT; 
			}
			break;
		}
	}
	
}


int16_t MOTOR_9015_TO_BMI(gimbal_t* gimbal, uint16_t ammo_angle)
{  
	int32_t delta_angle = -gimbal->Yaw_9015->base_info->encoder + ammo_angle;
	int32_t rslt = bmi_structure.yaw_angle * 8 ;
	
		
	/*加上误差角度*/
	rslt += delta_angle;
	
	if(rslt<- 32768)
	{
		rslt=(1)*(65536+rslt);	
	}else if(rslt > 32768)
	{
		rslt=(1)*(-65536+rslt);
	}
	
	return rslt;
}


int16_t Dynamic_lim_L(int16_t x)
{
	return (int16_t)(-0.0007*x*x + 9.34*x - 29470);
}

int16_t Dynamic_lim_R(int16_t x) 
{
	return (int16_t)(0.0015*x*x - 1.59*x +6543.9);
}