#include "my_gimbal.h"
#include "my_chassis.h"
#include <string.h>
#include <stdbool.h>
#include "car.h"
#include "dji_pid.h"
#include "remote.h"
#include "bmi.h"
#include "rp_math.h"



extern chassis_t              chassis_structure;


extern gimbal_work_info_t     gimbal_work_info_structure;
extern s_pitch_work_info_t    s_pitch_work_info_structure;
extern s_yaw_work_info_t      s_yaw_work_info_structure;
extern gimbal_info_t          gimbal_info_structure;
extern gimbal_config_t gimbal_config_structure;
extern gimbal_t gimbal_structure ;


extern motor_6020_t           motor_6020_YAW_structure;
extern motor_6020_base_info_t motor_6020_YAW_base_info;
extern motor_6020_info_t      motor_6020_YAW_info;


extern motor_6020_t           motor_6020_PIT_structure;
extern motor_6020_base_info_t motor_6020_PIT_base_info;
extern motor_6020_info_t      motor_6020_PIT_info;

extern car_t                  car_structure;

extern pid_t                  chassis_pid_position_structure;
extern pid_t                  chassis_pid_speed_structure;


extern pid_t                  yaw_lock_pid_speed_structure;
extern pid_t                  yaw_pid_speed_structure;
extern pid_t                  yaw_lock_pid_position_structure;
extern pid_t                  yaw_head_pid_position_structure;


extern pid_t                  yaw_follow_pid_position_structure;
extern pid_t                  pit_pid_position_structure;
extern pid_t                  pit_pid_speed_structure;

extern rc_t                   rc_structure;

extern car_t                  car_structure;

extern bmi_t                  bmi_structure;

       uint8_t                first2follow = 1;
/**
  * @brief  云台初始化
  * @param   
  * @retval  
  */
void gimbal_init (gimbal_t *gimbal)
{
	/* 电机初始化 */
	MOTOR_6020_INIT( &motor_6020_YAW_structure, &motor_6020_YAW_base_info , &motor_6020_YAW_info );
	MOTOR_6020_INIT( &motor_6020_PIT_structure, &motor_6020_PIT_base_info , &motor_6020_PIT_info );
	
	
	/* PID初始化 */
	
	  /*机械模式YAW位置环*/
	PID_struct_init( &yaw_lock_pid_position_structure,  POSITION_PID,20000,    0,  10,    0, 0.5);
	  /*机械模式YAW速度环*/
	PID_struct_init( &yaw_lock_pid_speed_structure,     POSITION_PID,20000, 5000,  20, 0.01,   0);
	
	
	  /*跟随模式YAW位置环（地盘跟头）*/
	PID_struct_init( &yaw_follow_pid_position_structure,POSITION_PID,20000,    0,   4,    0,   0);
	  /*跟随模式YAW速度环（总速度环）*/ 
	PID_struct_init( &yaw_pid_speed_structure,          POSITION_PID,20000, 2000,   3,    0,   0);
	  /*跟随模式YAW位置环（云台转动）*/
   PID_struct_init( &yaw_head_pid_position_structure,  POSITION_PID,20000,    0,   3,    0,   0);
	
	
	  /*PITCH的双环*/
	PID_struct_init( &pit_pid_speed_structure,          POSITION_PID,20000, 5000,  15,  0.4,   0);
	PID_struct_init( &pit_pid_position_structure,       POSITION_PID,20000,    0,  15,    0, 0.1); 
	
	 /*换头标志位*/
	gimbal->front_or_back = 0;
	
	/* 云台信息初始化 */
	gimbal_info_init(gimbal->info);
	
	/* 云台命令初始化 */
	gimbal_commond_init(gimbal);
	
	
}
/**
  * @brief  云台信息初始化
  * @param  
  * @retval 
  */
void gimbal_info_init(gimbal_info_t *info)
{
	memset(info, 0, sizeof(gimbal_info_t));
	info->target_pitch_angle =  6550;
}

/**
  * @brief  云台命令初始化
  * @param  
  * @retval 
  */
void gimbal_commond_init(gimbal_t *gimbal)
{
//	gimbal_to_check = false;
//	gimbal_out_check = false;
//  暂时不管
}

//初始化完成

/**
  * @brief  云台控制任务
  * @param  
  * @retval 
  */void gimbal_ctrl_task(gimbal_t *gimbal)
{
	/* 云台模式更新 */
  gimbal_mode_update(gimbal);
	
	/*云台陀螺仪赋值*/
	gimbal_yaw_can_update(gimbal);
	
	/* 云台命令响应 */
  gimbal_commond_respond(gimbal);
	
	/* 云台工作 */
  gimbal_work(gimbal);
	
	/* 云台命令初始化 */
	gimbal_commond_init(gimbal);
	
	/*云台电机数据发送*/
}

/**
  * @brief  云台模式更新
  * @param  
  * @retval 
  */
void gimbal_mode_update(gimbal_t *gimbal)
{

   if(car_structure.move_mode_status ==machine_CAR)
		{
			gimbal->work->status = G_G_lock;//机械
			
		}else if(car_structure.move_mode_status ==follow_CAR)
		{
			gimbal->work->status = G_G_follow;//跟随
		}else if(car_structure.move_mode_status == offline_CAR)
		{
			gimbal->work->status = G_G_offline;
		}else if(car_structure.move_mode_status == spin_CAR)
		{
			gimbal->work->status = G_G_spin;
		}
		
}

/**
  * @brief  云台命令响应
  * @param  
  * @retval 
  */
void gimbal_commond_respond(gimbal_t *gimbal)
{
	
}





void gimbal_work(gimbal_t *gimbal)
{
	gimbal_work_info_t *work = gimbal->work;
	gimbal_config_t *config = gimbal->config;
	gimbal_info_t *info = gimbal->info;
	s_pitch_work_info_t *pitch_work = gimbal->pitch_work;
	s_yaw_work_info_t *yaw_work = gimbal->yaw_work;
	watch_work_info_t *watch_work = gimbal->watch_work;
	
	static uint8_t G_W_flag;
	
	switch(work->status)
	{ 
		case G_G_offline:
			pitch_work->status = G_P_offline;
			yaw_work->status = G_Y_offline;
			watch_work->status = G_W_offline;
			work->work_status = 0;
			work->init_Y_O_N = 0;
		
			break;
			
			
		case G_G_follow:
			pitch_work->status = G_P_normal;
			yaw_work->status = G_Y_angle;
			watch_work->status = G_W_retract;
			break;
		
		case G_G_lock:
			pitch_work->status =  G_P_lock;
			yaw_work->status   =  G_Y_lock;
			watch_work->status =  G_W_retract;
			break;
		case G_G_spin:
			pitch_work->status =  G_P_normal;
			yaw_work->status   =  G_Y_spin;
			watch_work->status =  G_W_retract;
			break;
		
		default:
			break;
	}
	gimbal_pitch_work(gimbal);
	gimbal_yaw_work(gimbal);
}




void gimbal_pitch_work(gimbal_t *gimbal)
{
	gimbal_config_t *config = gimbal->config;
	gimbal_info_t *info = gimbal->info;
	s_pitch_work_info_t *work = gimbal->pitch_work;
	
	switch(work->status)
	{
		case G_P_offline:
			//info->target_pitch_speed = 0;
			//gimbal_pitch_speed_ctrl(gimbal);
		motor_6020_PIT_structure.output_current = 0;
		motor_6020_YAW_structure.output_current = 0;
			break;
		
		case G_P_normal:
			switch(car_structure.ctrl_mode)
			{
				case RC_CAR:
				  info->target_pitch_angle = info->target_pitch_angle + (int16_t)(rc_structure.base_info->ch1 * (-0.01));
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					info->target_pitch_angle -= rc_structure.info->mouse_y_K / 10.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_angle_ctrl(gimbal);
					break;
				default:
					break;
			}
			break;
			
		
	 case G_P_lock:
			switch(car_structure.ctrl_mode)
			{
				case RC_CAR:
					info->target_pitch_angle = info->target_pitch_angle + (int16_t)(rc_structure.base_info->ch1 * (-0.01));
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					//
					break;
				default:
					break;
			}
			break;	
	}
}







void gimbal_yaw_work(gimbal_t *gimbal)
{
	s_yaw_work_info_t *work = gimbal->yaw_work;
	gimbal_config_t *config = gimbal->config;
	gimbal_info_t *info = gimbal->info;
	
	switch(work->status)
	{
		case G_Y_offline:
			info->target_yaw_speed = 0;
		  motor_6020_YAW_structure.output_current = 0;
			//gimbal_yaw_speed_ctrl(gimbal);
			break;
	
		case G_Y_angle:
			//增加头尾判断
			if( gimbal->yaw->base_info->angle    <= 52 \
				&& gimbal->yaw->base_info->angle >= 4148)
			{
				gimbal->front_or_back = -1;
			}
			else
			{
				gimbal->front_or_back = 1;
			}
			/*避免模式切换时，转半天*/
			if(first2follow){
				gimbal->info->target_yaw_angle = bmi_structure.yaw_angle;
				first2follow                   = 0;
			}
			gimbal->info->target_yaw_angle = gimbal->info->target_yaw_angle + (int16_t)(rc_structure.base_info->ch0*(-0.01)) ;
			gimbal_yaw_angle_check(gimbal);
			gimbal_yaw_angle_ctrl(gimbal);		
			break;
		
		case G_Y_lock:
			gimbal->info->target_yaw_angle = 2100;
		    first2follow = 1;
			gimbal_yaw_angle_check(gimbal);
			gimbal_yaw_angle_lock_ctrl(gimbal);		
			break;
		
		case G_Y_spin:
			gimbal->info->target_yaw_angle = gimbal->info->target_yaw_angle + (int16_t)(rc_structure.base_info->ch0*(-0.01)) ;
			gimbal_yaw_angle_check(gimbal);
			gimbal_yaw_angle_ctrl(gimbal);	
			break;
			
	}
}

void gimbal_watch_work(gimbal_t *gimbal)
{

}



/*pitch电机数据更新 6020*/
void gimbal_pitch_can_update(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	motor_6020_base_info_t *base_info = gimbal->pitch->base_info;
	
	info->pitch_speed = - base_info->speed;
	info->pitch_angle -= base_info->angle_add;
}


void gimbal_watch_can_update(gimbal_t *gimbal)
{

}

/*pitch电机 角度限制 6020*/
void gimbal_pitch_angle_check(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	gimbal_config_t *config = gimbal->config;
	
	if(info->target_pitch_angle > 7300)
	{
		info->target_pitch_angle = 7300;
	}
	if(info->target_pitch_angle < 5800)
	{
		info->target_pitch_angle = 5800;
	}
	
}





/*pitch电机 速度环控制（内环）  6020*/
void gimbal_pitch_speed_ctrl(gimbal_t *gimbal)
{
	gimbal->pitch->output_current = pid_calc( &pit_pid_speed_structure,\
																				 bmi_structure.pit_gro,\
																				 gimbal->info->target_pitch_speed);
	
	
 motor_6020_PIT_structure.output_current = pit_pid_speed_structure.pos_out;
	
}

/*pitch电机 位置环控制（外环）  6020*/
void gimbal_pitch_angle_ctrl(gimbal_t *gimbal)
{
	
	
	gimbal->info->target_pitch_speed = pid_calc(&pit_pid_position_structure,\
																						  gimbal->pitch->base_info->angle,\
	                                            gimbal->info->target_pitch_angle);
	gimbal_pitch_speed_ctrl(gimbal);
	
}



/*yaw电机数据更新 6020*/
void gimbal_yaw_can_update(gimbal_t *gimbal)
{
	
	
	/*速度用陀螺仪 角度用自带的*/
	/*不管了先全都用返回来的数据，写出来先再改陀螺仪*/
	gimbal_info_t *info = gimbal->info;
	gimbal_config_t *config = gimbal->config;
	motor_6020_base_info_t *base_info = gimbal->yaw->base_info;
	
	
	/*改用陀螺仪*/
	gimbal->info->yaw_angle = bmi_structure.yaw_angle;
	
	/*
	 *机械模式下直接用电机返回的数据
	 *跟随模式用陀螺仪的欧拉角*/	
	gimbal_yaw_angle_check(gimbal);
	
	
	
}

/*yaw电机 角度限幅*/
void gimbal_yaw_angle_check(gimbal_t *gimbal)
{
    	gimbal_info_t *info = gimbal->info;
	
	if(info->target_yaw_angle > 8191)//目标判断
	{
		info->target_yaw_angle -= 8192 ;
		
	}else if(info->target_yaw_angle < 0)
	{
		info->target_yaw_angle += 8192 ;
	}
	
}



/*yaw电机 跟随位置环控制（外环）  6020*/
void gimbal_yaw_angle_ctrl(gimbal_t *gimbal)
{
	//小陀螺模式下不需要
	if(gimbal->yaw_work->status != G_Y_spin)
	{
	/*地盘跟头*/
	chassis_structure.info->target_cycle_speed= pid_calc_err(&yaw_follow_pid_position_structure,\
															 gimbal->yaw->base_info->angle,\
		                                                     gimbal->front_or_back == 1? 2100:6196);//换头
	}
	
	/*锁头*//*现在问题就集中在这里*/
	gimbal->info->target_yaw_speed = pid_calc_err(&yaw_head_pid_position_structure,\
												   bmi_structure.yaw_angle,\
	                                               gimbal->info->target_yaw_angle);
	//位置环参数给到底盘
	gimbal_yaw_speed_ctrl(gimbal);
}



/*yaw电机机械 位置环控制（外环）  6020*/
void gimbal_yaw_angle_lock_ctrl(gimbal_t *gimbal)
{
	
	/*机械模式*/
	gimbal->info->target_yaw_speed = pid_calc_err(&yaw_lock_pid_position_structure,\
										           gimbal->yaw->base_info->angle,\
	                                               2100);

	gimbal_yaw_lock_speed_ctrl(gimbal);
}




/*yaw电机 跟随速度环控制（内环）  6020*/
void gimbal_yaw_speed_ctrl(gimbal_t *gimbal)
{
		
	  gimbal->yaw->output_current = pid_calc( &yaw_pid_speed_structure,\
																					   bmi_structure.yaw_gro,\
																						 gimbal->info->target_yaw_speed);	
	
	 motor_6020_YAW_structure.output_current = yaw_pid_speed_structure.pos_out;
}





/*yaw电机机械 速度环控制（内环）  6020*/
void gimbal_yaw_lock_speed_ctrl(gimbal_t *gimbal)
{
		
	  gimbal->yaw->output_current = pid_calc( &yaw_lock_pid_speed_structure,\
																					   bmi_structure.yaw_gro,\
																						 gimbal->info->target_yaw_speed);	
	
	 motor_6020_YAW_structure.output_current = yaw_lock_pid_speed_structure.pos_out;
}













/*暂时没用*/
void gimbal_watch_speed_ctrl(gimbal_t *gimbal)
{

}
void gimbal_watch_angle_ctrl(gimbal_t *gimbal)
{

	
}



