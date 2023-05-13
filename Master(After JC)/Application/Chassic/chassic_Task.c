#include "rp_chassis.h"
#include "remote.h"
#include "Car.h"
#include "vision.h"
#include "bmi.h"
#include "rp_gimbal.h"
#include "judge.h"

extern chassis_t              Chassis; 
extern rc_t                   rc_structure;
extern car_t                  car_structure;
extern vision_t               vision_structure;
extern bmi_t                  bmi_structure;
extern gimbal_t               Gimbal;
extern judge_t                judge;
extern pid_t                  chassis_pid_follow_structure;


#define change_p              (400)
#define pi                    (3.14159265354)
uint32_t                      sin_cnt = 0;
float                         cita = 0;  
int16_t                       spin_speed = 0;

uint32_t                      sin_tick = 0; 
uint32_t                      cos_tick = 0; 
uint32_t                      hurt_change = 2000;


uint8_t                       game_star = 0;
uint8_t                       position_ok =0;

uint16_t                      right_cnt = 0;
uint16_t                      fornt_cnt = 0;

uint16_t                      right_time = 0;

uint16_t                      fornt_time = 0;

uint16_t                       aerial_time = 0; //云台手指令运行时间

#define AERIAL_W 1
#define AERIAL_A 2
#define AERIAL_S 3
#define AERIAL_D 4
#define AERIAL_NO 5

uint8_t chassis_aerial_cmd = AERIAL_NO;


void Chassic_Mode_Work()
{
	
	if(car_structure.ctrl == RC_CAR)
	{
		/*机械模式*/
		if(car_structure.mode == machine_CAR)
		{
			/*底盘控制部分*/
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.cycle_speed = (float)rc_structure.base_info->ch0 / 660.f * Chassis.work_info.config.speed_max;
			
			/*状态位清除*/
			Chassis.config_top_calc(CHASSIS_TOP_CALC_OFF);
			sin_cnt = 0;
			
			/*调整速度限制*/
			if(rc_structure.base_info->thumbwheel.value == -660)
			{
				if(Chassis.work_info.config.speed_max ++ >= 10000)
				{
					Chassis.work_info.config.speed_max = 10000;
				}
			}
			else if(rc_structure.base_info->thumbwheel.value == 660)
			{
				if(Chassis.work_info.config.speed_max -- <= 0)
				{
					Chassis.work_info.config.speed_max = 0;
				}					
			}

			
			
		}
		else if(car_structure.mode == navigation_CAR)
		{
			Chassis.base_info.target.front_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_right *change_p;//chassis_front 视觉正 前移
			Chassis.base_info.target.right_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_front*change_p; //视觉正 右移
			
			Chassis.base_info.target.cycle_speed = 0;
			
						
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
			
			//Chassis.base_info.target.cycle_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_cycle*change_p;//视觉正 逆时针
			
		}
		else if(car_structure.mode == spin_CAR )
		{
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)CHASSIS_SPEED_MAX / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)CHASSIS_SPEED_MAX / 660.f;
			Chassis.base_info.target.cycle_speed = spin_speed;
			
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
			
			/*调整速度限制*/
			if(rc_structure.base_info->thumbwheel.value == -660)
			{
				if(spin_speed ++ >= 7000)
				{
					spin_speed = 7000;
				}
			}
			else if(rc_structure.base_info->thumbwheel.value == 660)
			{
				if(spin_speed -- <= 0)
				{
					spin_speed = 0;
				}					 
			}
			
		}
		else if(car_structure.mode == two_CAR)
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 0;

			
		}
		else if(car_structure.mode == follow_CAR)
		{
			/*底盘控制部分*/
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.cycle_speed = pid_calc_err_9015(&chassis_pid_follow_structure,
																	  Gimbal.Yaw_9015->base_info->encoder,\
																	  MOROT_9015_MIDDLE); //由pid计算给出
			
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			if(Gimbal.Yaw_9015->info->status == _9015_OFFLINE)
			{
				Chassis.base_info.target.cycle_speed = 0;	
			}
			Chassis.config_top_calc(CHASSIS_TOP_CALC_OFF);
			

			
		}
		else if(car_structure.mode == vision_CAR)
		{
			
			/*PC数据转换*/
			//do something 
			
			/*赋值*/
//			Chassis.base_info.target.front_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_right *change_p;//chassis_front 视觉正 前移
//			Chassis.base_info.target.right_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_front*change_p; //视觉正 右移
//			Chassis.base_info.target.cycle_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_cycle*change_p;//视觉正 逆时针
//			
//			Chassis.config_top_calc(CHASSIS_TOP_CALC_OFF);
//			sin_cnt = 0;
			
			
//			if(judge.base_info->game_progress == 4)
//			{
//				game_star = 1;
//				position_ok = 1;
//				if(game_star == 1 && position_ok == 1)
//				{
//					/*变速小陀螺*/
//					Chassis.base_info.target.cycle_speed = 4000;
//					Chassis.base_info.target.right_speed = 0;
//					Chassis.base_info.target.front_speed = 0;
//				}
//				else
//				{
//					/*出发*/
//					if(fornt_cnt++ <= fornt_time_ )
//					{
//						Chassis.base_info.target.front_speed = -1000;
//						Chassis.base_info.target.right_speed = 0;
//						Chassis.base_info.target.cycle_speed = 0;
//					}
//					else
//					{
//						fornt_cnt = fornt_time_ + 500;
//						
//						position_ok = 1;
//					}
//					
//				}
//			}
//			else
//			{
//				/*底盘控制部分*/
//				Chassis.base_info.target.cycle_speed = 0;
//			}
				
			if(judge.base_info->last_HP - judge.base_info->remain_HP >= 9)
			{
				/*状态改变*/
				
				static int8_t direction = Auto_Up;
				
				if(hurt_change<= 3000)
				{
					direction = Auto_Down;
				}
				else if(hurt_change>= 1000)
				{
					direction = Auto_Up;
				}

				switch(direction)
				{
					case Auto_Up: 
					{
						hurt_change += 200;
						break;
					}
					case Auto_Down:
					{				
						hurt_change -= 200;
						break;
					}
				}
			}
			
			//Chassis.base_info.target.cycle_speed = 0*sin( ((sin_tick++)%1300)*2*3.14f ) +4000;
			if(judge.base_info->game_progress == 4 && judge.base_info->friendly_outposts_HP == 0)
			{
				
				Chassis.base_info.target.cycle_speed = 4000;
			}
			else
			{
				Chassis.base_info.target.cycle_speed = 0;
			}
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
		}
		else if(car_structure.mode == patrol_CAR || car_structure.mode ==  shake_CAR )
		{
		
			
			/*变速小陀螺*/
			if(judge.base_info->game_progress == 4 && judge.base_info->friendly_outposts_HP == 0)
			{
				Chassis.base_info.target.cycle_speed = 4000;
			}
			else
			{
				Chassis.base_info.target.cycle_speed = 0;
			}
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
			
			
			if(judge.base_info->last_HP - judge.base_info->remain_HP >= 9)
			{
				/*状态改变*/
				
				static int8_t direction = Auto_Up;
				
				if(hurt_change<= 3000)
				{
					direction = Auto_Down;
				}
				else if(hurt_change>= 1000)
				{
					direction = Auto_Up;
				}

				switch(direction)
				{
					case Auto_Up: 
					{
						hurt_change += 200;
						break;
					}
					case Auto_Down:
					{				
						hurt_change -= 200;
						break;
					}
				}
			}

			/*受击变速*/
//			if(judge.base_info->hurt_type == ammo_HURT)
//			{
//				/*清零*/
//				judge.base_info->hurt_type = no_HURT;
//				
//				
//				/*状态改变*/
//				
//				static int8_t direction = Auto_Up;
//				
//				if(hurt_change<= 3000)
//				{
//					direction = Auto_Down;
//				}
//				else if(hurt_change>= 1000)
//				{
//					direction = Auto_Up;
//				}

//				switch(direction)
//				{
//					case Auto_Up: 
//					{
//						hurt_change += 200;
//						break;
//					}
//					case Auto_Down:
//					{				
//						hurt_change -= 200;
//						break;
//					}
//				}
//			}
			

			
//			Chassis.base_info.target.cycle_speed = 1000;
//			cita = ((sin_cnt++%5000)/5000.0)*2*pi;
//			Chassis.base_info.target.front_speed = (-1)*sin(cita)*(float)1000;
//			Chassis.base_info.target.right_speed = (-1)*cos(cita)*(float)1000;
			
		}
		else 
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 0;
		}
		
	}
	else if(car_structure.ctrl == MINIPC_CAR)
	{
		/*PC数据转换*/
		//do something 
		
		/*赋值*/
		Chassis.base_info.target.front_speed = (float)vision_structure.rx_pack->RxData.chassis_front;
		Chassis.base_info.target.right_speed = (float)vision_structure.rx_pack->RxData.chassis_right;
		Chassis.base_info.target.cycle_speed = (float)vision_structure.rx_pack->RxData.chassis_cycle;
		
	}
	else if(car_structure.ctrl == TEST_CAR)
	{
		Chassis.base_info.target.front_speed = (float)rc_structure.base_info->ch3 * (float)CHASSIS_SPEED_MAX / 660.f;
		Chassis.base_info.target.right_speed = (float)rc_structure.base_info->ch2 * (float)CHASSIS_SPEED_MAX / 660.f;
		Chassis.base_info.target.cycle_speed = (float)rc_structure.base_info->ch0 / 660.f * CHASSIS_SPEED_MAX;
		
	}
	
	/*云台手指令*/
	if(judge.info->status == JUDGE_ONLINE)
	{
		if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->last_commond == 'W') //Z
		{
			chassis_aerial_cmd = AERIAL_W;
		}
		else if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->last_commond == 'A') //X
		{
			chassis_aerial_cmd = AERIAL_A;
		}
		else if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->last_commond == 'S') //X
		{
			chassis_aerial_cmd = AERIAL_S;
		}
		else if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->last_commond == 'D') //X
		{
			chassis_aerial_cmd = AERIAL_D;
		}
	}
	else
	{
		chassis_aerial_cmd = AERIAL_NO;
	}
	
	if(chassis_aerial_cmd == AERIAL_W)
	{
		if(aerial_time++ >= 500)
		{
			chassis_aerial_cmd = AERIAL_NO;
			aerial_time = 0;
		}
		else
		{
			Chassis.base_info.target.front_speed = -1000;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 2000;
				
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
				
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_A)
	{
		if(aerial_time++ >= 500)
		{
			chassis_aerial_cmd = AERIAL_NO;
			aerial_time = 0;
		}
		else
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = -1000;
			Chassis.base_info.target.cycle_speed = 2000;
				
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
				
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_D)
	{
		if(aerial_time++ >= 500)
		{
			chassis_aerial_cmd = AERIAL_NO;
			aerial_time = 0;
		}
		else
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 1000;
			Chassis.base_info.target.cycle_speed = 2000;
				
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
				
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_S)
	{
		if(aerial_time++ >= 500)
		{
			chassis_aerial_cmd = AERIAL_NO;
			aerial_time = 0;
		}
		else
		{
			Chassis.base_info.target.front_speed = 1000;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 2000;
				
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
				
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	
	Chassis_Work(&Chassis);

}

