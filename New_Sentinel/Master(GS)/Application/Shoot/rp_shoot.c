/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v1.1		
  * @Author     : hwx			
  * @Date       : 2022-11-06         
  * @Description:    
  *
  *
  ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "rp_shoot.h"


/* Exported variables --------------------------------------------------------*/
//左右波蛋轮结构体
shoot_t                       L_shoot_structure;
shoot_t                       R_shoot_structure;

/*波胆点击*/
motor_2006_t           L_motor_2006_PLUCK_structure;
motor_2006_base_info_t L_motor_2006_PLUCK_base_info;
motor_2006_info_t      L_motor_2006_PLUCK_info;

motor_2006_t           R_motor_2006_PLUCK_structure;
motor_2006_base_info_t R_motor_2006_PLUCK_base_info;
motor_2006_info_t      R_motor_2006_PLUCK_info;


/*左电机*/
pid_t                  L_pid_s_stir_sin; /*单发位置环*/
pid_t                  L_pid_p_stir_sin; /*单发速度环*/
pid_t                  L_pid_p_stir_done;/*堵转速度环*/
pid_t                  L_pid_s_stir_run; /*连发速度环*/


/*右电机*/
pid_t                  R_pid_s_stir_sin; /*单发位置环*/
pid_t                  R_pid_p_stir_sin; /*单发速度环*/
pid_t                  R_pid_p_stir_done;/*堵转速度环*/
pid_t                  R_pid_s_stir_run; /*连发速度环*/



void SHOOT_INIT()
{
	
	//左
	Motor_2006_Init(&L_motor_2006_PLUCK_structure,  &L_motor_2006_PLUCK_base_info,&L_motor_2006_PLUCK_info );
    /*单发位置环*/
    PID_struct_init(&L_pid_s_stir_sin,POSITION_PID,SPID_STRI_OUTPUT_MAX,SPID_STRI_INTEGRA_MAX,SPID_STRI_KP,SPID_STRI_KI,SPID_STRI_KD);
    PID_struct_init(&L_pid_p_stir_sin,POSITION_PID,PPID_STRI_OUTPUT_MAX,PPID_STRI_INTEGRA_MAX,PPID_STRI_KP,PPID_STRI_KI,PPID_STRI_KD);
    /*连发速度环*/
    
	PID_struct_init(&L_pid_s_stir_run,POSITION_PID,TPID_STRI_OUTPUT_MAX,TPID_STRI_INTEGRA_MAX,TPID_STRI_KP,TPID_STRI_KI,TPID_STRI_KD);
	/*堵转处理位置环*/
	PID_struct_init(&L_pid_p_stir_done,POSITION_PID,PID_DONE_OUTPUT_MAX,PID_DONE_INTEGRA_MAX,PID_DONE_KP,PID_DONE_KI,PID_DONE_KD);
	
	L_shoot_structure.base.status = Shoot_Offline;
	
	L_shoot_structure.pid_s_stir_sin  = &L_pid_s_stir_sin;
	L_shoot_structure.pid_p_stir_sin  = &L_pid_p_stir_sin;
	L_shoot_structure.pid_p_stir_done = &L_pid_p_stir_done;
	L_shoot_structure.pid_s_stir_run  = &L_pid_s_stir_run;
	
	L_shoot_structure.stir_wheel = &L_motor_2006_PLUCK_structure;
	L_shoot_structure.status = Stop_Shoot;
	
	//右
	Motor_2006_Init(&R_motor_2006_PLUCK_structure,  &R_motor_2006_PLUCK_base_info,&R_motor_2006_PLUCK_info );
	 /*单发位置环*/
    PID_struct_init(&R_pid_s_stir_sin,POSITION_PID,SPID_STRI_OUTPUT_MAX,SPID_STRI_INTEGRA_MAX,SPID_STRI_KP,SPID_STRI_KI,SPID_STRI_KD);
    PID_struct_init(&R_pid_p_stir_sin,POSITION_PID,PPID_STRI_OUTPUT_MAX,PPID_STRI_INTEGRA_MAX,PPID_STRI_KP,PPID_STRI_KI,PPID_STRI_KD);
    /*连发速度环*/
    PID_struct_init(&R_pid_s_stir_run,POSITION_PID,TPID_STRI_OUTPUT_MAX,TPID_STRI_INTEGRA_MAX,TPID_STRI_KP,TPID_STRI_KI,TPID_STRI_KD);
	/*堵转处理位置环*/
	PID_struct_init(&R_pid_p_stir_done,POSITION_PID,PID_DONE_OUTPUT_MAX,PID_DONE_INTEGRA_MAX,PID_DONE_KP,PID_DONE_KI,PID_DONE_KD);
	
	R_shoot_structure.base.status = Shoot_Offline;
	
	R_shoot_structure.pid_s_stir_sin  = &R_pid_s_stir_sin;
	R_shoot_structure.pid_p_stir_sin  = &R_pid_p_stir_sin;
	R_shoot_structure.pid_p_stir_done = &R_pid_p_stir_done;
	R_shoot_structure.pid_s_stir_run  = &R_pid_s_stir_run;
	
	R_shoot_structure.stir_wheel = &R_motor_2006_PLUCK_structure;
	
	
	R_shoot_structure.status = Stop_Shoot;
	
	
}



void Shoot_Heart(shoot_t * shoot)
{

}


/*连发*/
void Running_Fire(shoot_t * shoot)
{
	/*波蛋轮速度环PID*/
	shoot->stir_wheel->output_current  = pid_calc(shoot->pid_s_stir_run,\
										shoot->stir_wheel->base_info->speed,\
										shoot->base.shoot_speed);
	//can_send_data
}

/*单发*/
void Single_Fire(shoot_t * shoot)
{
	
	/*波蛋轮位置环PID*/
	shoot->base.shoot_speed           = pid_calc(shoot->pid_p_stir_sin,\
										shoot->stir_wheel->base_info->angle_sum,\
										shoot->stir_wheel->base_info->target_angle_sum);
	/*波蛋轮速度环PID*/
	shoot->stir_wheel->output_current  = pid_calc(shoot->pid_s_stir_sin,\
										shoot->stir_wheel->base_info->speed,\
										shoot->base.shoot_speed);
}





/*堵转检测*/
void Done_Check(shoot_t * shoot)
{
	/*单发堵转*/
	if(shoot->status == Single_Shoot)
	{
		if( abs(shoot->stir_wheel->base_info->target_angle_sum - shoot->stir_wheel->base_info->angle_sum) >=  1500)
		{
			
			if( shoot->cnt.done_time ++ >= SIX_SHOOT_DONE_TIME_MAX)//堵转时间达到能忍受的最大值
			{
				shoot->flag.locked = 1 ;//表示正在处理堵转（思考如何判断解除堵转）

			}
			else
			{
				/*进行堵转次数清零*/
				shoot->cnt.deal_done_cnt = 0;
				//shoot->cnt.done_time;
			}		
		
		}
		else
		{
				shoot->cnt.done_time  = 0;//连发模式未发生堵转
				shoot->flag.locked    = 0;//结束堵转
		}
	}
	/*连发堵转*/
	else if(shoot->status == Running_Shoot ||shoot->status == Visin_Shoot)
	{
		if( abs(shoot->stir_wheel->base_info->speed - shoot->base.shoot_speed) >=  500)
		{
			
			if( shoot->cnt.done_time ++ >= SIX_SHOOT_DONE_TIME_MAX)//堵转时间达到能忍受的最大值
			{
				shoot->flag.locked = 1 ;//表示正在处理堵转（思考如何判断解除堵转）

			}
			else
			{
				//shoot->cnt.done_time;
			}		
		
		}
		else
		{
				shoot->cnt.done_time  = 0;//连发模式未发生堵转
				shoot->flag.locked    = 0;//结束堵转
		}
	}
	/*停转处理*/
	else if(shoot->status ==Stop_Shoot)
	{
		L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		
		R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
	}
	
}

/**
  * @brief  堵转处理
  * @param
  * @retval 应该要一个堵转时间计数器
  */

void DONE_WORK(shoot_t* shoot)
{
	if( ( (shoot->cnt.deal_time++) %DEAL_DONE_ROUND_TIME)  >= DEAL_DONE_ROUND_HALF_TIME)
	{
		 /*波蛋轮速度环PID*/
        shoot->stir_wheel->output_current = pid_calc(shoot->pid_p_stir_done,\
														shoot->stir_wheel->base_info->speed,\
														DEAL_DONE_SPEED);	//反转
	}
	else
	{
		/*波蛋轮速度环PID*/
        shoot->stir_wheel->output_current = pid_calc(shoot->pid_p_stir_done,\
														shoot->stir_wheel->base_info->speed,\
														(-1)*DEAL_DONE_SPEED);//正转
	}
	
	if(shoot->cnt.deal_time == DEAL_DONE_ROUND_TIME)
	{
		shoot->cnt.deal_done_cnt++;
		shoot->cnt.deal_time = 0;
		shoot->cnt.done_time = 0;//结束堵转
		shoot->flag.locked = 0;
	}	
	
}



