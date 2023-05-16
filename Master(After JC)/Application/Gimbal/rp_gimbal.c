/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v1.1		
  * @Author     : hwx			
  * @Date       : 2023��2��28��         
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
  * @Name    GIMBAL_INIT
  * @brief   ��̨��ʼ��
  * @param   
  * @retval 
  * @author  HWX
  * @Date    2023��2��28��
**/
void GIMBAL_INIT(gimbal_t* gimbal, motor_9015_t* motor )
{
	/*��YAW��ʼ��*/
	gimbal->Yaw_9015 = motor;
	gimbal->target_speed     = 0;
	gimbal->target_yaw_angle = 0;
	gimbal->staus.work_sate  = GIMBAL_OFFLINE;
	
	/*����ͷ��ʼ��*/
	gimbal->L_Head.staus = GIMBAL_OFFLINE;
	gimbal->L_Head.target_yaw_angle = 0;
	gimbal->L_Head.target_pit_angle = 0;
	
	gimbal->R_Head.staus = GIMBAL_OFFLINE;
	gimbal->R_Head.target_yaw_angle = 0;
	gimbal->R_Head.target_pit_angle = 0;
	
	uint32_t init_cnt = 0;
	while(gimbal->Yaw_9015->base_info->temperature != 0 )
	{
		MOTOR_9015_OFF();
		if(init_cnt ++ >= 100000)
		{
			break;
		}
		
	}
	
	gimbal->target_yaw_angle = gimbal->Yaw_9015->base_info->encoder;
}

/**
  * @Name    Big_Yaw_Speed
  * @brief   ��̨�ٶȿ���
  * @param   
  * @retval
  * @author  HWX
  * @Date    2023��2��28��
**/
void Big_Yaw_Speed(gimbal_t* gimbal)
{
	MOTOR_9015_HEART();
	
	if(gimbal->staus.work_sate == GIMBAL_ONLINE && \
	   rc_structure.info->status == REMOTE_ONLINE)
	{
		motor_9015_structure.base_info->target_speed = gimbal->target_speed;
		MOTOR_9015_SPEED();
	}
	else
	{
		//ж��
		MOTOR_9015_OFF();
	}
}

/**
  * @Name    Big_Yaw_Position
  * @brief   ��̨λ�ÿ���
  * @param   
  * @retval
  * @author  HWX
  * @Date    2023��2��28��
**/
void Big_Yaw_Position(gimbal_t* gimbal)
{
	MOTOR_9015_HEART();
	
	if(gimbal->staus.work_sate == GIMBAL_ONLINE && \
	   rc_structure.info->status == REMOTE_ONLINE)
	{	
		motor_9015_structure.base_info->target_angle = gimbal->target_yaw_angle;
		MOTOR_9015_POSIT();
	}
	else
	{	
		//ж��
		MOTOR_9015_OFF();
		
	}
}

/**
  * @Name    Big_Yaw_Position
  * @brief   ��̨λ�ÿ���
  * @param   
  * @retval
  * @author  HWX
  * @Date    2023��2��28��
**/
void Big_Yaw_Position_Bmi(gimbal_t* gimbal)
{
	MOTOR_9015_HEART();
	
	if(gimbal->staus.work_sate == GIMBAL_ONLINE && \
	   rc_structure.info->status == REMOTE_ONLINE)
	{	
		motor_9015_structure.base_info->target_angle = gimbal->target_yaw_angle;
		MOTOR_9015_POSIT_BMI();
	}
	else
	{	
		//ж��
		MOTOR_9015_OFF();
		
	}
}

void Gimbal_Dynamic_Limit(gimbal_t* gimbal)
{
	int16_t Left  = gimbal->L_Head.target_yaw_angle; //һ���ǵ���ĽǶ�
	int16_t Right = gimbal->R_Head.target_yaw_angle; 
	
	/*���ݱ�׼��
	
		2048       |       2048
	 0	  L   4096 |  4096   R   0
		 8192      |        8192
	
	*/
	
	/*ģʽ���� 
	����ģʽ�� ��Χ����
	����ģʽ�� ��̬�޷�
	*/
	
	/*������ͷ��������ͷ�ķ�Χ*/
	
	
	

}

/*yaw��� �Ƕ�*/
void Big_Yaw_Angle_Check(gimbal_t* gimbal)
{
	
    if(gimbal->target_yaw_angle >= 65536)//Ŀ���ж�
    {
        gimbal->target_yaw_angle -= 65536 ;
    } 
	else if(gimbal->target_yaw_angle <= 0)
    {
        gimbal->target_yaw_angle += 65536 ;
    }

}

void Pitch_Auto_R(Master_Head_t* M2H)
{
	static uint8_t direction_p_R = Auto_Up;  //��ʼѲ������
	
	if(M2H->Send_R_Head.target_pit > P_MOT_UPP_LIMIT - 200)
	{
		direction_p_R = Auto_Up;
	}
	else if(M2H->Send_R_Head.target_pit < P_MOT_LOW_LIMIT + 700)
	{
		direction_p_R = Auto_Down;
	}
	
	
	switch(direction_p_R)
	{
		case Auto_Up: M2H->Send_R_Head.target_pit -= PITCH_ROTATE_UNIT; break;       
		case Auto_Down: M2H->Send_R_Head.target_pit += PITCH_ROTATE_UNIT; break;
	}

	
}

void Pitch_Auto_L(Master_Head_t* M2H)
{
	static uint8_t direction_p_L = Auto_Up;  //��ʼѲ������
	
	if(M2H->Send_L_Head.target_pit > P_MOT_UPP_LIMIT_ - 100)
	{
		direction_p_L = Auto_Up;
	}
	else if(M2H->Send_L_Head.target_pit < P_MOT_LOW_LIMIT_ + 1000)
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
	static uint8_t direction_L = Auto_Right;  //��ʼѲ������
	static uint8_t cnt = 0;        //�ı�Ŀ��ֵ��ʱ����
	
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
	static uint8_t direction_R = Auto_Right;  //��ʼѲ������
	static uint8_t cnt = 0;        //�ı�Ŀ��ֵ��ʱ����
	
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
	
		
	/*�������Ƕ�*/
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