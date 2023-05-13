#include "rp_gimbal.h"
#include "remote.h"
#include "Car.h"
#include "vision.h"
#include "bmi.h"

extern bmi_t                  bmi_structure;
extern gimbal_t               Gimbal;
extern rc_t                   rc_structure;
extern car_t                  car_structure;
extern motor_9015_t           motor_9015_structure;
extern vision_t               vision_structure;

uint16_t                      first2vision = 0;
uint16_t                      first2follow = 0;
uint16_t                      first2spin = 0;


uint32_t                      motor_9015_offline_cnt = 0;
void Gimbal_Work()
{
	/*电机掉线检测*/
	MOTOR_9015_HEART();
	
	/*云台掉线检测*/
	if (Gimbal.Yaw_9015->info->status == _9015_ONLINE)
	{
		Gimbal.staus.work_sate = GIMBAL_ONLINE;
	}
	else
	{
		Gimbal.staus.work_sate = GIMBAL_OFFLINE;
//		if(motor_9015_offline_cnt++ %900 == 1)
//		{
//			MOTOR_9015_STAR();
//			motor_9015_offline_cnt = 0;
//		}
	}
	
	
	/*模式切换*/
	if(car_structure.ctrl == RC_CAR)
	{
		if(car_structure.mode == machine_CAR ||car_structure.mode == navigation_CAR)//机械模式
		{
			
			Gimbal.target_yaw_angle = MOROT_9015_MIDDLE;
			Big_Yaw_Angle_Check(&Gimbal);
			Big_Yaw_Position(&Gimbal);

			
		}
		else if(car_structure.mode == follow_CAR || car_structure.mode == two_CAR || car_structure.mode == patrol_CAR|| car_structure.mode ==  shake_CAR)//跟随模式
		{
			if(car_structure.mode == follow_CAR)
			{
				Gimbal.target_yaw_angle += (int32_t)(rc_structure.base_info->ch0*(-0.09));
			}
			
			if(first2follow == 0 )
			{
				Gimbal.target_yaw_angle = bmi_structure.yaw_angle * 8;
			}
			first2follow = 1;
			
			Big_Yaw_Angle_Check(&Gimbal);	
			Big_Yaw_Position_Bmi(&Gimbal);
		}
		else if(car_structure.mode == vision_CAR ||car_structure.mode == aim_CAR)//视觉模式
		{
			#if 1
			if(first2vision == 0)
			{
				Gimbal.target_yaw_angle = bmi_structure.yaw_angle;
			}
			first2vision = 1;
			Gimbal.target_yaw_angle = vision_structure.rx_pack->RxData.B_yaw_angle;
			motor_9015_structure.base_info->target_angle = Gimbal.target_yaw_angle;
			MOTOR_9015_POSIT_BMI_VISION(&Gimbal);
			#endif
			
			#if 0
			if(first2vision == 0)
			{
				Gimbal.target_yaw_angle = Gimbal.Yaw_9015->base_info->encoder;
			}
			first2vision = 1;
			//Gimbal.target_yaw_angle = vision_structure.rx_pack->RxData.B_yaw_angle * 8;
			Big_Yaw_Position(&Gimbal);
			
			#endif

		}
		else if(car_structure.mode == spin_CAR)//小陀螺模式
		{
			if(first2spin == 0 )
			{
				Gimbal.target_yaw_angle = bmi_structure.yaw_angle * 8;
			}
			first2spin = 1;
			
			Gimbal.target_yaw_angle += (int32_t)(rc_structure.base_info->ch0*(-0.09));
			Big_Yaw_Angle_Check(&Gimbal);
			Big_Yaw_Position_Bmi(&Gimbal);
		}
		else if(car_structure.mode == offline_CAR)//离线模式
		{
			MOTOR_9015_OFF();
		}
		
		/*标志位清零*/
		if(car_structure.mode != follow_CAR)
		{
			first2follow = 0;
		}
		if(car_structure.mode != spin_CAR)
		{
			first2spin = 0;
		}
		if(car_structure.mode != vision_CAR)
		{
			first2vision = 0;
		}
		
		
		
	}
	else if(car_structure.ctrl == MINIPC_CAR)
	{
		
	}
	else if(car_structure.ctrl == TEST_CAR)
	{

		
	}
	

}

