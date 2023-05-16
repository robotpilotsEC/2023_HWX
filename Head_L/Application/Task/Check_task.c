#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "driver.h"
#include "9015_motor.h"
#include "dji_pid.h"
#include "bmi.h"
#include "can_protocol.h"
#include "rp_shoot.h"
#include "rp_gimbal.h"

extern void Shoot_Work();

extern motor_9015_t           motor_9015_structure;
extern motor_9015_info_t      motor_9015_info_structure;
extern motor_9015_base_info_t motor_9015_base_info_structure;
extern motor_9015_pid_t       motor_9015_pid_structure;
extern pid_t                  pid_position_structure;
extern Master_Head_t          Master_Head_structure;
extern shoot_t                shoot_structure;
extern gimbal_t               gimbal_structure;

int32_t test_speed = 5000;
int32_t test_angle = 1000;
uint8_t test_flag  = 1;

/*红 -- 底盘
  绿 -- 云台
  蓝 -- 发射机构
  橙 -- 其他
             */


void CheckTask(void const * argument)
{
	while(1)
	{	
		  
		if(shoot_structure.base.status == Shoot_Offline && gimbal_structure.work.status == Gimbal_Offline)
		{
			LASER_TOGGLE();
			LED_RED_TOGGLE();
			LED_BLUE_TOGGLE();
			LED_GREEN_TOGGLE();
			osDelay(100);
		}
		else if(shoot_structure.base.status == Shoot_Offline)
		{
			LASER_TOGGLE();
			LED_RED_TOGGLE();
			LED_BLUE_TOGGLE();
			osDelay(500);
		}
		else if(gimbal_structure.work.status == Gimbal_Offline)
		{
			LASER_TOGGLE();
			LED_RED_TOGGLE();
			osDelay(1000);
		}
		else if(Master_Head_structure.L_Head_status.status == DEV_OFFLINE)
		{
			LASER_OFF();
		}
		else
		{
			LASER_ON();
		}
		
		
			
		osDelay(1);
	}


}

void BmiUpdateTask(void const * argument)
{
	while(1)
	{
		BMI_Updata();
	
		osDelay(1);
	}

}

int16_t test_test_angle;
int16_t test_pit_angle = 3913;//bmi:3913  motor:5680
int16_t test_yaw_angle = 3481;//bmi:3481  motor:6150

void GimbalTask(void const * argument)
{  
	while(1)
	{
		
		Gimbal_work();

		if(bmi_structure.state == BMI_INIT)
		{
			gimbal_structure.base.target_pit = Master_Head_structure.Send_L_Head.target_pit;
			gimbal_structure.base.target_yaw = Master_Head_structure.Send_L_Head.target_yaw;

			gimbal_work(&gimbal_structure);
		}
		else
		{
			if(gimbal_structure.base.mode == DATA_FROM_BMI_AND_MOTOR)
			{
				if( gimbal_structure.pitch->base_info->angle != 0 )
				{
				test_yaw_angle = gimbal_structure.yaw->base_info->angle;
				test_pit_angle = gimbal_structure.pitch->base_info->angle;
				}
				else
				{
					test_pit_angle = 5680;//bmi:3913  motor:5680
					test_yaw_angle = 6150;//bmi:3481  motor:6150
				}
			}
			else if(gimbal_structure.base.mode == DATA_FROM_BMI)
			{
				test_yaw_angle = bmi_structure.pit_angle;
				test_pit_angle = bmi_structure.yaw_angle;
			}
		}
		
		osDelay(3);
	}

}

void ShootTask(void const * argument)
{
	while(1)
	{
		
		Shoot_Work();
		osDelay(1);
	}

}

void H2MTask(void const * argument)
{
	while(1)
	{
		/*完成数据接收*/
		//中断中完成
		Master_Head_HEART(&Master_Head_structure);
		
		/*完成数据同步*/
		//gimbal_structure.base.target_pit  = Master_Head_structure.Send_L_Head.target_pit;
		//gimbal_structure.base.target_yaw  = Master_Head_structure.Send_L_Head.target_yaw;
		
		
		/*数据反馈*/
		if(gimbal_structure.work.status == Gimbal_Online)
		{
			Master_Head_structure.From_L_Head.gimbal_mode = 1;
		}
		else
		{
			Master_Head_structure.From_L_Head.gimbal_mode = 0;
		}
		
		if(shoot_structure.base.status == Shoot_Online)
		{
			Master_Head_structure.From_L_Head.shoot_mode = 1;
		}
		else
		{
			Master_Head_structure.From_L_Head.shoot_mode = 0;
		}
		
		/*将当前的云台反馈给Master*/
		Master_Head_structure.From_L_Head.measure_pit = gimbal_structure.pitch->base_info->angle;
		Master_Head_structure.From_L_Head.measure_yaw = gimbal_structure.yaw->base_info->angle;
		
		
		osDelay(1);//数据反馈可以慢一点 3436 
	}

}
