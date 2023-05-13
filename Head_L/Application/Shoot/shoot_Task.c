#include "stm32F4xx_hal.h"
#include "3508_motor.h"
#include "rp_shoot.h"
#include "can_protocol.h"



extern shoot_t                       shoot_structure;
extern Master_Head_t                 Master_Head_structure;

uint8_t firc_wheel = 1;
uint8_t single_fire = 0;
uint8_t running_fire = 0;
int16_t test_shoot_speed = 0; //-3000
int16_t test_fric_speed =0;//-7800
int16_t test_cnt = 0;
int16_t test_cnt_max = 130;
int16_t stri_stop = 0;

void Shoot_Work()
{		

		
		if(Master_Head_structure.L_Head_status.status == DEV_ONLINE)
		{
			if(Master_Head_structure.Send_L_Head.shoot_mode != 0)
			{
				shoot_structure.base.firct_speed = -(5800 + Master_Head_structure.Send_L_Head.shoot_mode*30);
				
			}
			else
			{
				shoot_structure.base.firct_speed = 0;
			}


		}
		else
		{
			shoot_structure.base.firct_speed = 0;
		}
		

	
	/*摩擦轮工作*/
	Fric_Work(&shoot_structure);
		
	/*电机掉线检测*/
	MOTOR_3508_HEART(shoot_structure.fric_right);
	MOTOR_3508_HEART(shoot_structure.fric_left);
	
	
	if(shoot_structure.fric_left->info->status  == _DEV_OFFLINE  ||\
	   shoot_structure.fric_right->info->status == _DEV_OFFLINE)
	{
		/*卸力*/
		MOTOR_3508_CAN1_SENT_DATA(0,0,0,0);

		/*状态更新*/
		shoot_structure.base.status = Shoot_Offline;
	}
	else //未发生掉线
	{
		/*数据发送*/
		MOTOR_3508_CAN1_SENT_DATA(shoot_structure.fric_right->output_current,\
			  shoot_structure.fric_left->output_current,\
			  shoot_structure.stir_wheel->output_current,\
			  0);	
		/*状态更新*/
		shoot_structure.base.status = Shoot_Online;
	}
}	































