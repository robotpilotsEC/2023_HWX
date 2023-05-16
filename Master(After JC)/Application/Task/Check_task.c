#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bmi.h"
#include "main.h"
#include "cmsis_os.h"
#include "driver.h"
#include "9015_motor.h"
#include "dji_pid.h"
#include "rp_chassis.h"
#include "rp_gimbal.h"
#include "remote.h"
#include "can_protocol.h"
#include "vision.h"
#include "Car.h"
#include "rp_shoot.h"
#include "judge.h"
#include "vision.h"

extern judge_t                judge;
extern motor_9015_t           motor_9015_structure;
extern motor_9015_info_t      motor_9015_info_structure;
extern motor_9015_base_info_t motor_9015_base_info_structure;
extern motor_9015_pid_t       motor_9015_pid_structure;
extern pid_t                  pid_position_structure;
extern bmi_t                  bmi_structure;
extern chassis_t              Chassis; 
extern rc_t                   rc_structure;
extern Master_Head_t          Master_Head_structure;
extern gimbal_t               Gimbal;
extern vision_t               vision_structure;
extern car_t                  car_structure;
extern IWDG_HandleTypeDef     hiwdg;

extern void MODE_CHECK(void);
extern void Chassic_Mode_Work(void);
extern void Gimbal_Work(void);

int32_t test_speed = 5000;
int32_t test_angle = 1000;
uint8_t test_flag  = 1;

/*红 -- 底盘
  绿 -- 云台
  蓝 -- 遥控器
  橙 -- 其他
             */

void CheckTask(void const * argument)
{
	while(1)
	{
		if(Chassis.work_info.work_sate == CHASSIC_ONLINE)
		{
			LED_ORANGE_TOGGLE();
			osDelay(100);
		}
		else
		{
			LED_ORANGE_ON();
		}
		
		if(Master_Head_structure.L_Head_status.status == M2H_ONLINE && \
		   Master_Head_structure.R_Head_status.status == M2H_ONLINE)
		{	
			LED_BLUE_TOGGLE();
			osDelay(100);
		}
		else
		{
			LED_BLUE_ON();
		}
		if(Gimbal.staus.work_sate == GIMBAL_ONLINE)
		{
			LED_RED_TOGGLE();
			osDelay(100);
		}
		else
		{
			LED_RED_ON();
		}
		if(vision_structure.state.work_state == VISION_ONLINE)
		{
			LED_GREEN_TOGGLE();
			osDelay(100);
		}
		else
		{
			LED_GREEN_ON();
		}
		
		if(judge.info->status != JUDGE_ONLINE)
		{
			osDelay(500);
		}
		osDelay(1);
	}


}

void BmiUpdateTask(void const * argument)
{
	while(1)
	{
		
		/*喂狗*/
		HAL_IWDG_Refresh(&hiwdg);
		
		BMI_Updata();
		
		/*临时放着*/
		MODE_CHECK();
		RC_HEART(&rc_structure);
		judge_heart();
		
		osDelay(1);
	}

}
void GimbalTask(void const * argument)
{
	while(1)
	{
		Gimbal_Work();
		
		osDelay(1);
	}
}
void VisionTask(void const * argument)
{
	while(1)
	{
		vision_task();
		/*试一下4行不行*/
		osDelay(5);
	}

}

void ShootTask(void const * argument)
{
	while(1)
	{
		Shoot_Work();
		
		osDelay(5);
	}
}




/*底盘工作*/
void ChassisTask(void const * argument)
{
	while(1)
	{
		Chassic_Mode_Work();
		
		osDelay(1);
	}


}

uint8_t    gimbal_init_ok = 0;
extern uint8_t first2vision;
int16_t    L_Head_Limit = 650;
int16_t    R_Head_Limit = 6000;
uint16_t   first2ParVis = 0;

extern uint8_t  hurt_mode;
extern uint32_t hurt_cnt;

void M2HTask(void const * argument)
{
	while(1)
	{	
		/*云台通讯*/
		Master_Head_HEART(&Master_Head_structure);
		
		/*接收数据解析*/
		/*云台部分*/
		if(gimbal_init_ok == 0)
		{
			Master_Head_structure.Send_L_Head.target_pit = P_Init;
			Master_Head_structure.Send_L_Head.target_yaw = Y_Init;
			
			Master_Head_structure.Send_R_Head.target_pit = P_Init_;
			Master_Head_structure.Send_R_Head.target_yaw = Y_Init_;
			gimbal_init_ok = 1;
		}
		else if(rc_structure.info->status == REMOTE_OFFLINE)
		{
			gimbal_init_ok = 1;
		}
		/*发射机构部分*/

		/*发送数据处理*/
		if(car_structure.mode == two_CAR)
		{

			Master_Head_structure.Send_L_Head.target_pit  += (int32_t)(rc_structure.base_info->ch1*(-0.008));
			Master_Head_structure.Send_L_Head.target_yaw  += (int32_t)(rc_structure.base_info->ch0*(-0.008));
			
			Master_Head_structure.Send_R_Head.target_pit  += (int32_t)(rc_structure.base_info->ch3*(-0.008));
			Master_Head_structure.Send_R_Head.target_yaw  += (int32_t)(rc_structure.base_info->ch2*(-0.008));
			
			Gimbal.L_Head.target_pit_angle = Master_Head_structure.Send_L_Head.target_pit;
			Gimbal.L_Head.target_yaw_angle = Master_Head_structure.Send_L_Head.target_yaw;
			
			Gimbal.R_Head.target_pit_angle = Master_Head_structure.Send_R_Head.target_pit;
			Gimbal.R_Head.target_yaw_angle = Master_Head_structure.Send_R_Head.target_yaw;
			
			Master_Head_structure.Send_L_Head.gimbal_mode = 1;
			Master_Head_structure.Send_R_Head.gimbal_mode = 1;
			
		}
		else if(car_structure.mode == aim_CAR)
		{		
			    /*确认处于测试自瞄模式*/
				Master_Head_structure.Send_L_Head.target_pit  = vision_structure.rx_pack->RxData.L_pitch_angle;
				Master_Head_structure.Send_L_Head.target_yaw  = vision_structure.rx_pack->RxData.L_yaw_angle;
				 Master_Head_structure.Send_R_Head.target_pit  = vision_structure.rx_pack->RxData.R_pitch_angle;
				Master_Head_structure.Send_R_Head.target_yaw  = vision_structure.rx_pack->RxData.R_yaw_angle;
		}
		else if(car_structure.mode ==  shake_CAR)
		{
			/*摇头模式*/

			static uint16_t shake_cnt = 0;
			
			if(shake_cnt++ % 100 >= 50)
			{
					Master_Head_structure.Send_L_Head.target_pit  += 0;
					Master_Head_structure.Send_L_Head.target_yaw  -= 10;
					Master_Head_structure.Send_R_Head.target_pit  += 0;
					Master_Head_structure.Send_R_Head.target_yaw  += 10;
			}
			else
			{
					Master_Head_structure.Send_L_Head.target_pit  -= 0;
					Master_Head_structure.Send_L_Head.target_yaw  += 10;
					Master_Head_structure.Send_R_Head.target_pit  -= 0;
					Master_Head_structure.Send_R_Head.target_yaw  -= 10;	
			}
			
			Master_Head_structure.Send_L_Head.gimbal_mode = 1;
			Master_Head_structure.Send_R_Head.gimbal_mode = 1;
			
			
			
		}
		else if(car_structure.mode == vision_CAR)
		{
			if(vision_structure.state.work_state == VISION_ONLINE)
			{
				
				Master_Head_structure.Send_L_Head.target_pit  = vision_structure.rx_pack->RxData.L_pitch_angle;
				Master_Head_structure.Send_L_Head.target_yaw  = vision_structure.rx_pack->RxData.L_yaw_angle;

				Master_Head_structure.Send_R_Head.target_pit  = vision_structure.rx_pack->RxData.R_pitch_angle;
				Master_Head_structure.Send_R_Head.target_yaw  = vision_structure.rx_pack->RxData.R_yaw_angle;

				Master_Head_structure.Send_L_Head.gimbal_mode = 1;
				Master_Head_structure.Send_R_Head.gimbal_mode = 1;
			}
			else //保护视觉 
			{
					Master_Head_structure.Send_R_Head.target_pit  += (int32_t)(rc_structure.base_info->ch1*(-0.005));
					Master_Head_structure.Send_R_Head.target_yaw  += (int32_t)(rc_structure.base_info->ch0*(-0.005));
					Master_Head_structure.Send_L_Head.target_pit  += (int32_t)(rc_structure.base_info->ch3*(-0.005));
					Master_Head_structure.Send_L_Head.target_yaw  += (int32_t)(rc_structure.base_info->ch2*(-0.005));
				
					Master_Head_structure.Send_L_Head.gimbal_mode = 1;
					Master_Head_structure.Send_R_Head.gimbal_mode = 1;
				
			}
		}
		else if(car_structure.mode == offline_CAR)
		{
			Master_Head_structure.Send_L_Head.target_pit  = Master_Head_structure.From_L_Head.measure_pit;
			Master_Head_structure.Send_L_Head.target_yaw  = Master_Head_structure.From_L_Head.measure_yaw;
			
			Master_Head_structure.Send_R_Head.target_pit  = Master_Head_structure.From_R_Head.measure_pit;
			Master_Head_structure.Send_R_Head.target_yaw  = Master_Head_structure.From_R_Head.measure_yaw;
			
			Master_Head_structure.Send_L_Head.gimbal_mode = 0;
			Master_Head_structure.Send_R_Head.gimbal_mode = 0;
			
		}
		else if(car_structure.mode == patrol_CAR)
		{
			

			
			Yaw_Auto_L(&Master_Head_structure);
			Pitch_Auto_L(&Master_Head_structure);
			
			Yaw_Auto_R(&Master_Head_structure);
			Pitch_Auto_R(&Master_Head_structure);
			
			if(hurt_cnt <= 1000 &&  hurt_mode != 0)
			{
				Master_Head_structure.Send_L_Head.target_yaw = Y_Init_;
				Master_Head_structure.Send_R_Head.target_yaw = Y_Init;
			}
			
			Master_Head_structure.Send_L_Head.gimbal_mode = 1;
			Master_Head_structure.Send_R_Head.gimbal_mode = 1;
			
		}
		else if(car_structure.mode == machine_CAR)
		{
			Master_Head_structure.Send_L_Head.target_pit  = P_Init_;
			Master_Head_structure.Send_L_Head.target_yaw  = Y_Init_;
			
			Master_Head_structure.Send_R_Head.target_pit  = P_Init;
			Master_Head_structure.Send_R_Head.target_yaw  = Y_Init;
			
			Master_Head_structure.Send_L_Head.gimbal_mode = 1;
			Master_Head_structure.Send_R_Head.gimbal_mode = 1;
			
		}
		else
		{
//			Master_Head_structure.Send_L_Head.target_pit = Gimbal.L_Head.target_pit_angle;
//			Master_Head_structure.Send_L_Head.target_yaw = Gimbal.L_Head.target_yaw_angle;
//			
//			Master_Head_structure.Send_R_Head.target_pit = Gimbal.R_Head.target_pit_angle;
//			Master_Head_structure.Send_R_Head.target_yaw = Gimbal.R_Head.target_yaw_angle;
		}
		
		
		
		
		/*限位*/
		if(Master_Head_structure.Send_R_Head.target_yaw >= 8192)
		{
			Master_Head_structure.Send_R_Head.target_yaw -= 8192;	
		}
		else if(Master_Head_structure.Send_R_Head.target_yaw < 0)
		{
			Master_Head_structure.Send_R_Head.target_yaw += 8192;
		}
		
		if( Master_Head_structure.Send_R_Head.target_pit >= P_MOT_UPP_LIMIT)
		{
			Master_Head_structure.Send_R_Head.target_pit = P_MOT_UPP_LIMIT;
		}
		else if(Master_Head_structure.Send_R_Head.target_pit <= P_MOT_LOW_LIMIT)
		{
			Master_Head_structure.Send_R_Head.target_pit = P_MOT_LOW_LIMIT;
		}
		
		if(Master_Head_structure.Send_L_Head.target_yaw >= 8192)
		{
			Master_Head_structure.Send_L_Head.target_yaw -= 8192;	
		}
		else if(Master_Head_structure.Send_L_Head.target_yaw < 0 )
		{
			Master_Head_structure.Send_L_Head.target_yaw += 8192;
		}
		
		if( Master_Head_structure.Send_L_Head.target_pit >= P_MOT_UPP_LIMIT_)
		{
			Master_Head_structure.Send_L_Head.target_pit = P_MOT_UPP_LIMIT_;
		}
		else if(Master_Head_structure.Send_L_Head.target_pit <= P_MOT_LOW_LIMIT_)
		{
			Master_Head_structure.Send_L_Head.target_pit = P_MOT_LOW_LIMIT_;
		}
		
		
		/*动态限位*/
		
//		L_Head_Limit = Dynamic_lim_L(Master_Head_structure.From_R_Head.measure_yaw);
//		L_test       = Dynamic_lim_L(Master_Head_structure.From_R_Head.measure_yaw);
//		R_Head_Limit = Dynamic_lim_R(Master_Head_structure.From_L_Head.measure_yaw);
//		R_test       = Dynamic_lim_R(Master_Head_structure.From_L_Head.measure_yaw);
//		
//		if(L_Head_Limit > 650)
//		{
//			L_Head_Limit = 650;
//		}
//		
//		if(R_Head_Limit < 6000)
//		{
//			R_Head_Limit = 6000;
//		} 
		
		
		/*没头脑*/
		if( Master_Head_structure.Send_R_Head.target_yaw > 3250 &&  Master_Head_structure.Send_R_Head.target_yaw< 4750)
		{
			Master_Head_structure.Send_R_Head.target_yaw = 3250;
		}
		else if(Master_Head_structure.Send_R_Head.target_yaw <= R_Head_Limit && Master_Head_structure.Send_R_Head.target_yaw >= 4750)
		{
			Master_Head_structure.Send_R_Head.target_yaw = R_Head_Limit;
		}
		

		

		/*不高兴*/
		if( Master_Head_structure.Send_L_Head.target_yaw >= L_Head_Limit &&  Master_Head_structure.Send_L_Head.target_yaw<= 1944)
		{
			Master_Head_structure.Send_L_Head.target_yaw = L_Head_Limit;
		}
		else if(Master_Head_structure.Send_L_Head.target_yaw <= 3483 && Master_Head_structure.Send_L_Head.target_yaw >= 1944)
		{
			Master_Head_structure.Send_L_Head.target_yaw = 3483;
		}
		
		if(car_structure.mode != patrol_CAR && car_structure.mode != vision_CAR)
		{
			first2ParVis = 1;
		}

		/*卡头保护*/
		M2H_SENT_DATA(&Master_Head_structure);
		
		/*4*/
		osDelay(4);
	}
	
}



