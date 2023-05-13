#include "Car.h"
#include "remote.h"
#include "rp_chassis.h"
#include "rp_gimbal.h"
#include "can_protocol.h"
#include "rp_shoot.h"
#include "vision.h"
#include "rp_shoot.h"
#include "judge.h"
#include "FreeRTOS.h"
#include "can_protocol.h"

extern judge_t                judge;
extern car_t                  car_structure;
extern rc_t                   rc_structure;
extern chassis_t              Chassis; 
extern gimbal_t               Gimbal;
extern shoot_t                L_shoot_structure;
extern shoot_t                R_shoot_structure;
extern Master_Head_t          Master_Head_structure;
extern vision_t               vision_structure;



/*遥控器跳变检测变量*/
int8_t            s2_down_flag   = 1;
int8_t            s2_up_flag     = 1;
extern uint8_t shoot_1_over_speed_flag;
extern uint8_t shoot_2_over_speed_flag;
extern int16_t    test_shoot_speed;
void MODE_INIT(void)
{
	car_structure.ctrl = RC_CAR;
	car_structure.mode  = offline_CAR;

}


void MODE_CHECK(void)
{
    /*状态更新*/
    rc_wheel_status_interrupt_update(rc_structure.base_info);
	all_key_board_status_interrupt_update(rc_structure.base_info);
	all_key_board_status_update(rc_structure.base_info);
	
	/*检测跳变(检测向下拨动)*/
	/*检测右拨杆向下*/
	if(rc_structure.base_info->s2.value_last != rc_structure.base_info->s2.value \
			&& rc_structure.base_info->s2.value == 2)
	{
		s2_down_flag = s2_down_flag*(-1);
	}

	/*检测右拨杆向上*/
	if(rc_structure.base_info->s2.value_last != rc_structure.base_info->s2.value \
			&& rc_structure.base_info->s2.value == 1)
	{
		s2_up_flag = s2_up_flag*(-1);
	}
	
	if(rc_structure.info->status == REMOTE_ONLINE)
	{
		/*模块状态更新*/

		
		/*按键映射*/
		if(S1_DOWN)//底盘运动控制
		{

			/*发射机构轴状态更新*/
			L_shoot_structure.status = Stop_Shoot;
			R_shoot_structure.status = Stop_Shoot;
			
			
			if(S2_UP)
			{
				/*小陀螺*/
				car_structure.mode = spin_CAR;
			}
		   else if(S2_DOWN)
			{
				/*调整两个头*/
				car_structure.mode = machine_CAR;
				
				if(rc_structure.base_info->thumbwheel.status == down_R)
				{
						car_structure.mode = offline_CAR;
					
						osDelay(100);
					
						__set_FAULTMASK(1);
						NVIC_SystemReset();
				}
				else if(rc_structure.base_info->thumbwheel.status == up_R)
				{
						car_structure.mode = offline_CAR;
					
						osDelay(100);
					
						__set_FAULTMASK(1);
						NVIC_SystemReset();			
				}
			}
			else if(S2_MIDDLE)
			{	
				/*这样写能默认进入跟随模式？*/
				static car_move_mode_e s2_down_mode  = follow_CAR;
				car_structure.mode = s2_down_mode;
				
				if(rc_structure.base_info->thumbwheel.value == -660)
				{
					/*跟随模式*/
					s2_down_mode = follow_CAR;
				}
				else if(rc_structure.base_info->thumbwheel.value == 660)
				{
					/*跟随模式*/
					s2_down_mode = navigation_CAR;				
				}
			}

		}
		else if(S1_MIDDLE)//云台控制模式
		{
			
			if(S2_UP)
			{
				/*巡逻模式*/
				//car_structure.mode = patrol_CAR;
				car_structure.mode = shake_CAR;
				/*发射机构轴状态更新*/
				L_shoot_structure.status = Stop_Shoot;
				R_shoot_structure.status = Stop_Shoot;

			}
			else if(S2_MIDDLE)
			{
				/*调整双头模式*/
				car_structure.mode = two_CAR;
				
				if(rc_structure.base_info->thumbwheel.value == 660)
				{
					L_shoot_structure.status = Running_Shoot;
					R_shoot_structure.status = Running_Shoot;
				}
				
				if(rc_structure.base_info->thumbwheel.value == -660)
				{
					L_shoot_structure.status = Stop_Shoot;
					R_shoot_structure.status = Stop_Shoot;
				}


			}
			else if(S2_DOWN)
			{
				/*自瞄模式*/
				car_structure.mode = aim_CAR;
				
				/*发射机构轴状态更新*/
				L_shoot_structure.status = Stop_Shoot;
				R_shoot_structure.status = Stop_Shoot;
			}

			

		}
		else if(S1_UP)//上拨码
		{

				
			if(S2_MIDDLE)
			{	
				/*视觉模式*/
				if(vision_structure.rx_pack->FrameHeader.cmd_id == 1)
				{
					car_structure.mode = vision_CAR;
				}
				else if(vision_structure.rx_pack->FrameHeader.cmd_id == 2)
				{
					car_structure.mode = shake_CAR;
				}
				else
				{
					car_structure.mode = patrol_CAR;
				}
				//car_structure.mode = vision_CAR;
				L_shoot_structure.status = Stop_Shoot;
				R_shoot_structure.status = Stop_Shoot;
				
				/*在vision.c中更新*/
				/*在视觉模式下，根据视觉发的标志位选择连发还是单发*/
				/*做好异常数据剔除*/
//				if(L_shoot_structure.cnt.shoot_cnt != L_shoot_structure.cnt.last_shoot_cnt)
//				{
//					uint8_t L_err =abs(L_shoot_structure.cnt.last_shoot_cnt - L_shoot_structure.cnt.shoot_cnt);
//					if(L_err >= 5)
//					{
//						L_err = 0;
//					}
//					
//					L_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864*(L_err);
//				}
//				if(R_shoot_structure.cnt.shoot_cnt != R_shoot_structure.cnt.last_shoot_cnt)
//				{
//					uint8_t R_err =abs(L_shoot_structure.cnt.last_shoot_cnt - L_shoot_structure.cnt.shoot_cnt);
//					if(R_err >= 5)
//					{
//						R_err = 0;
//					}
//					
//					R_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864*((R_err));
//				}
				

			}
			else if(S2_UP)
			{
				/*视觉模式*/
				
				car_structure.mode = vision_CAR;
				
				if(judge.base_info->game_progress == 4)
				{
					L_shoot_structure.status = Visin_Shoot;
					R_shoot_structure.status = Visin_Shoot;
					/*若比赛开始则强制进入*/
					if(vision_structure.rx_pack->FrameHeader.cmd_id == 1)
					{
						car_structure.mode = vision_CAR;
					}
					else if(vision_structure.rx_pack->FrameHeader.cmd_id == 2)
					{
						car_structure.mode = shake_CAR;
					}
					else
					{
						car_structure.mode = patrol_CAR;
					}
					
				} 
				else
				{
					car_structure.mode = patrol_CAR;
					L_shoot_structure.status = Stop_Shoot;
					R_shoot_structure.status = Stop_Shoot;
					
				}
				/*两边向上，比赛模式*/
				/*发射机构 ： 单发、连发模式*/
				/*底盘：视觉控制模式*/
				/*云台：视觉控制模式 */
			}
			else if(S2_DOWN)
			{
		
				/*视觉模式*/
				car_structure.mode = vision_CAR;
				L_shoot_structure.status = Single_Shoot;
				R_shoot_structure.status = Single_Shoot;
				/*单发*/
				if(rc_structure.base_info->thumbwheel.status == down_R)
				{
					L_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
				}
				else if(rc_structure.base_info->thumbwheel.status == up_R)
				{
					R_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
				}
			}
		}	
	}
	else
	{
		/*卸力*/
		/*底盘卸力在rp_chassis.c中完成*/
		/*云台卸力需要通讯*/
		Master_Head_structure.Send_L_Head.shoot_mode = 0;
		Master_Head_structure.Send_R_Head.shoot_mode = 0;
		L_shoot_structure.status = Stop_Shoot;
		R_shoot_structure.status = Stop_Shoot;
		car_structure.mode = offline_CAR;
		
	}
	
	





}

















