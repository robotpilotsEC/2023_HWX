#include "stm32F4xx_hal.h"
#include "3508_motor.h"
#include "rp_shoot.h"
#include "remote.h"
#include "vision.h"
#include "Car.h"
#include "judge.h" 
#include "can_protocol.h"


extern judge_t                       judge;
extern car_t                         car_structure;
extern shoot_t                       L_shoot_structure;
extern shoot_t                       R_shoot_structure;
extern motor_2006_t                  L_motor_2006_PLUCK_structure;
extern motor_2006_t                  R_motor_2006_PLUCK_structure;
extern vision_t                      vision_structure;
extern rc_t                          rc_structure;
extern Master_Head_t                 Master_Head_structure;

/*超射速处理*/
extern uint8_t                       shoot_1_over_speed_flag;
extern uint8_t                       shoot_2_over_speed_flag;


float L_shoot_speed = 0;
float R_shoot_speed = 0;

#define AERIAL_SHOOT 1
#define AERIAL_STOP_SHOOT 0

uint8_t shoot_aerial_cmd = AERIAL_SHOOT;

void Shoot_Work(shoot_t* shoot)
{
	/*比赛开始视觉*/
	if(rc_structure.info->status == REMOTE_ONLINE )
	{
		if(judge.base_info->game_progress == 4) //若比赛开始则强制将打蛋控制权给视觉
		{
			L_shoot_structure.status = Visin_Shoot;
			R_shoot_structure.status = Visin_Shoot;
			
		}
		else if(judge.base_info->game_progress == 5)//比赛结束强制关闭发射机构
		{
			L_shoot_structure.status = Stop_Shoot;
			R_shoot_structure.status = Stop_Shoot;
		}
	}
	else
	{
		L_shoot_structure.status = Stop_Shoot;
		R_shoot_structure.status = Stop_Shoot;
	}
	
	/*云台手命令处理*/
	if(judge.info->status == JUDGE_ONLINE)
	{
		if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->last_commond == 'Z') //Z
		{
			shoot_aerial_cmd = AERIAL_STOP_SHOOT;
		}
		else if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->last_commond == 'X') //X
		{
			shoot_aerial_cmd = AERIAL_SHOOT;
		}
	}
	/*裁判系统掉线处理*/
	else
	{
		shoot_aerial_cmd = AERIAL_STOP_SHOOT;
	}
	/*视觉射速处理*/
	
	L_shoot_speed = -((vision_structure.rx_pack->RxData.L_shoot_speed/10.0)/8.0)*60*36;
	R_shoot_speed = -((vision_structure.rx_pack->RxData.R_shoot_speed/10.0)/8.0)*60*36;
	
	
	/*枪管热量限制*/
	if(judge.base_info->shooter_id1_cooling_heat >= 240- 40 ||shoot_aerial_cmd == AERIAL_STOP_SHOOT)
	{
		L_shoot_speed = 0;
		L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
	}
	
	if(judge.base_info->shooter_id2_cooling_heat >= 240- 40 ||shoot_aerial_cmd == AERIAL_STOP_SHOOT)
	{
		R_shoot_speed = 0;
		R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
	}
	
	
	/*掉线检测*/
	MOTOR_2006_HEART(&L_motor_2006_PLUCK_structure);
	MOTOR_2006_HEART(&R_motor_2006_PLUCK_structure);
	
	/*堵转检测*/
	Done_Check(&L_shoot_structure);
	Done_Check(&R_shoot_structure);
	
	
	/*左拨蛋轮*/
	/*连发模式*/
	if(L_shoot_structure.status == Running_Shoot)
	{
		if(L_shoot_structure.flag.locked == 0)
		{
			if(judge.base_info->shooter_id1_cooling_heat >= 240- 50)
			{
				L_shoot_structure.base.shoot_speed = 0;
			}
			else
			{
				L_shoot_structure.base.shoot_speed = -2760;
			}
			/*连发测试*/
			Running_Fire(&L_shoot_structure);

			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*单发模式*/
	else if (L_shoot_structure.status == Single_Shoot)
	{
		

		if(L_shoot_structure.flag.locked == 0)
		{
			//Car.C中处理
			//L_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			
			/*角度控制*/
			Single_Fire(&L_shoot_structure);
			
			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*视觉控制*/
	else if(L_shoot_structure.status == Visin_Shoot )
	{

		
		if(L_shoot_structure.flag.locked == 0)
		{
			
			//Car.C中处理
			//L_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			
			#if 0
			if(L_shoot_structure.base.last_shoot_speed != L_shoot_structure.base.shoot_speed && \
			   L_shoot_structure.base.last_shoot_speed == 0)
			{
				/*为了快速打出第一发*/
				L_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
			}
			/*检测第一发是否打出*/
			if(abs(L_shoot_structure.stir_wheel->base_info->angle_sum - L_shoot_structure.stir_wheel->base_info->target_angle_sum) <= 2000)
			{
				L_shoot_structure.flag.vision_first_shoot = 1;
			}
			
			/*发射速度为零就清零标志位*/
			if(L_shoot_structure.base.shoot_speed == 0)
			{
				L_shoot_structure.flag.vision_first_shoot = 0;
			}
			
			/*若第一发打出则切换为速度控制*/
			if(L_shoot_structure.flag.vision_first_shoot == 1)//打出第一发
			{
				Running_Fire(&L_shoot_structure);
			}
			else
			{
				Single_Fire(&L_shoot_structure);
			}
			#endif
			
			#if 1
			/*数据预处理*/
			L_shoot_structure.base.last_shoot_speed = L_shoot_structure.base.shoot_speed;
			L_shoot_structure.base.shoot_speed = L_shoot_speed;
			
			Running_Fire(&L_shoot_structure);
			#endif
			
			/*堵转清零*/
			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*发射机构关闭模式*/
	else
	{	
		/*停止转动*/
		L_shoot_structure.base.shoot_speed = 0;
		Running_Fire(&L_shoot_structure);
		
		/*角度和清零*/
		L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		/*堵转标志位清零*/
		L_shoot_structure.cnt.done_time = 0;
		L_shoot_structure.flag.locked = 0;
	}
		
		

	/*右拨蛋轮*/
	/*连发模式*/
	if(R_shoot_structure.status == Running_Shoot)
	{
		if(R_shoot_structure.flag.locked == 0)
		{
			if(judge.base_info->shooter_id1_cooling_heat >= 240- 50)
			{
				R_shoot_structure.base.shoot_speed = 0;
			}
			else
			{
				R_shoot_structure.base.shoot_speed = -2760;
			}

			/*连发测试*/
			Running_Fire(&R_shoot_structure);

			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*单发模式*/
	else if (R_shoot_structure.status == Single_Shoot)
	{
		
		if(R_shoot_structure.flag.locked == 0)
		{
			//Car.C中处理
			//R_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			/*角度控制*/
			Single_Fire(&R_shoot_structure);
			
			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*视觉模式*/
	else if(R_shoot_structure.status == Visin_Shoot)
	{
		
		
		if(R_shoot_structure.flag.locked == 0)
		{
			//Car.C中处理
			//L_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			#if 0
			if(R_shoot_structure.base.last_shoot_speed != R_shoot_structure.base.shoot_speed && \
			   R_shoot_structure.base.last_shoot_speed == 0)
			{
				/*为了快速打出第一发*/
				R_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
			}
			/*检测第一发是否打出*/
			if(abs(R_shoot_structure.stir_wheel->base_info->angle_sum - R_shoot_structure.stir_wheel->base_info->target_angle_sum) <= 2000)
			{
				R_shoot_structure.flag.vision_first_shoot = 1;
			}
			
			/*发射速度为零就清零标志位*/
			if(R_shoot_structure.base.shoot_speed == 0)
			{
				R_shoot_structure.flag.vision_first_shoot = 0;
			}
			
			/*若第一发打出则切换为速度控制*/
			if(R_shoot_structure.flag.vision_first_shoot == 1)//打出第一发
			{
				Running_Fire(&R_shoot_structure);
			}
			else
			{
				Single_Fire(&R_shoot_structure);
			}
			#endif
				
			#if 1
			/*数据预处理*/
			R_shoot_structure.base.last_shoot_speed = R_shoot_structure.base.shoot_speed;
			R_shoot_structure.base.shoot_speed = R_shoot_speed;
			
			Running_Fire(&R_shoot_structure);
			#endif
			/*堵转清零*/
			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*发射机构关闭模式*/
	else
	{
		R_shoot_structure.base.shoot_speed = 0;
		Running_Fire(&R_shoot_structure);
		/*角度和清零*/
		R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		/*堵转标志位清零*/
		R_shoot_structure.cnt.done_time = 0;
		R_shoot_structure.flag.locked = 0;
	}

	
	/*防止堵转处理导致电机过热，防止单次堵转次数过高*/
	/*警钟长鸣！！！！！！！！！！！！！！！！！！！！*/
	if(L_shoot_structure.cnt.deal_done_cnt >= DEAL_TIME_MAX)
	{
		L_shoot_structure.cnt.deal_done_cnt = DEAL_TIME_MAX;
		L_shoot_structure.stir_wheel->output_current = 0;
	}
	
	if(R_shoot_structure.cnt.deal_done_cnt >= DEAL_TIME_MAX)
	{
		R_shoot_structure.cnt.deal_done_cnt = DEAL_TIME_MAX;
		R_shoot_structure.stir_wheel->output_current = 0;
	}
	/*警钟长鸣！！！！！！！！！！！！！！！！！！！！*/
	
	/*摩擦轮状态更新*/
	if(L_shoot_structure.status == Visin_Shoot || \
	   L_shoot_structure.status == Single_Shoot || \
	   L_shoot_structure.status == Running_Shoot)
	{
		Master_Head_structure.Send_L_Head.shoot_mode = 1 + shoot_2_over_speed_flag;
	}
	else
	{
		Master_Head_structure.Send_L_Head.shoot_mode = 0;
	}
	
	if(R_shoot_structure.status == Visin_Shoot || \
	   R_shoot_structure.status == Single_Shoot || \
	   R_shoot_structure.status == Running_Shoot)
	{
		Master_Head_structure.Send_R_Head.shoot_mode = 1 + shoot_1_over_speed_flag;
	}
	else
	{
		Master_Head_structure.Send_R_Head.shoot_mode = 0;
	}
	/*电机数据发送（单个电机掉线单独清零，不能一个掉线另一个也不发射）*/
	
	/*掉线保护*/
	
	/*不高兴g了*/
//	L_shoot_structure.stir_wheel->output_current = 0;
//	Master_Head_structure.Send_L_Head.shoot_mode = 0;
	/*不高兴g了*/
	
	if( rc_structure.info->status == REMOTE_OFFLINE	)
	{
		int16_t L_current = L_shoot_structure.stir_wheel->output_current;
		int16_t R_current = R_shoot_structure.stir_wheel->output_current;
		
		if(L_shoot_structure.stir_wheel->info->status  == _DEV_OFFLINE)
		{
			/*状态更新*/
			L_current = 0;
			L_shoot_structure.base.status = Shoot_Offline;
		}
			
		if(R_shoot_structure.stir_wheel->info->status  == _DEV_OFFLINE)
		{
			/*状态更新*/
			R_current = 0;
			R_shoot_structure.base.status = Shoot_Offline;
		}
		
		/*卸力*/
		MOTOR_3508_CAN2_SENT_DATA(L_current,R_current,0,0);
			
	}
	/*正常工作*/
	else 
	{
		if(Master_Head_structure.L_Head_status.status != M2H_ONLINE)
		{
			L_shoot_structure.stir_wheel->output_current = 0;
		}
		if(Master_Head_structure.R_Head_status.status != M2H_ONLINE)
		{
			R_shoot_structure.stir_wheel->output_current = 0;
		}
		/*数据发送*/
		MOTOR_3508_CAN2_SENT_DATA(L_shoot_structure.stir_wheel->output_current,\
								  R_shoot_structure.stir_wheel->output_current,\
								  0,\
								  0);
		/*状态更新*/
		L_shoot_structure.base.status = Shoot_Online;
		R_shoot_structure.base.status = Shoot_Online;
	}
	
	
	

}	































