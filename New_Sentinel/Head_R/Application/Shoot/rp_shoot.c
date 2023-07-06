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
/*波胆点击*/
motor_2006_t           motor_2006_PLUCK_structure;
motor_2006_base_info_t motor_2006_PLUCK_base_info;
motor_2006_info_t      motor_2006_PLUCK_info;

/*摩擦轮*/
motor_3508_t           motor_3508_FIRC_L_structure;
motor_3508_base_info_t motor_3508_FIRC_L_base_info;
motor_3508_info_t      motor_3508_FIRC_L_info;

motor_3508_t           motor_3508_FIRC_R_structure;
motor_3508_base_info_t motor_3508_FIRC_R_base_info;
motor_3508_info_t      motor_3508_FIRC_R_info;


pid_t                  pid_s_left;
pid_t                  pid_s_righ;

pid_t                  pid_s_stir_sin;
pid_t                  pid_p_stir_sin;

pid_t                  pid_p_stir_done;

pid_t                  pid_s_stir_run;



void SHOOT_INIT(shoot_t * shoot)
{
    /*电机们 初始化*/
    MOTOR_3508_INIT(&motor_3508_FIRC_L_structure, &motor_3508_FIRC_L_base_info,&motor_3508_FIRC_L_info );
    MOTOR_3508_INIT(&motor_3508_FIRC_R_structure, &motor_3508_FIRC_R_base_info,&motor_3508_FIRC_R_info );
    MOTOR_2006_INIT(&motor_2006_PLUCK_structure,  &motor_2006_PLUCK_base_info,&motor_2006_PLUCK_info );
	
	shoot->stir_wheel = &motor_2006_PLUCK_structure;
	shoot->fric_left  = &motor_3508_FIRC_L_structure;
	shoot->fric_right = &motor_3508_FIRC_R_structure;

    /*摩擦轮速度环*/
    PID_struct_init(&pid_s_left,POSITION_PID,PID_FIRC_OUTPUT_MAX,PID_FIRC_INTEGRA_MAX,PID_FIRC_KP,PID_FIRC_KI,PID_FIRC_KD);
    PID_struct_init(&pid_s_righ,POSITION_PID,PID_FIRC_OUTPUT_MAX,PID_FIRC_INTEGRA_MAX,PID_FIRC_KP,PID_FIRC_KI,PID_FIRC_KD);

    /*单发位置环*/
    PID_struct_init(&pid_s_stir_sin,POSITION_PID,SPID_STRI_OUTPUT_MAX,SPID_STRI_INTEGRA_MAX,SPID_STRI_KP,SPID_STRI_KI,SPID_STRI_KD);
    PID_struct_init(&pid_p_stir_sin,POSITION_PID,PPID_STRI_OUTPUT_MAX,PPID_STRI_INTEGRA_MAX,PPID_STRI_KP,PPID_STRI_KI,PPID_STRI_KD);

    /*连发速度环*/
    PID_struct_init(&pid_s_stir_run,POSITION_PID,TPID_STRI_OUTPUT_MAX,TPID_STRI_INTEGRA_MAX,TPID_STRI_KP,TPID_STRI_KI,TPID_STRI_KD);
	
	/*堵转处理位置环*/
	PID_struct_init(&pid_p_stir_done,POSITION_PID,PID_DONE_OUTPUT_MAX,PID_DONE_INTEGRA_MAX,PID_DONE_KP,PID_DONE_KI,PID_DONE_KD);
	
	
	shoot->pid_s_left      = &pid_s_left;
	shoot->pid_s_righ      = &pid_s_righ;
	shoot->pid_s_stir_sin  = &pid_s_stir_sin;
	shoot->pid_p_stir_sin  = &pid_p_stir_sin;
	shoot->pid_p_stir_done = &pid_p_stir_done;
	shoot->pid_s_stir_run  = &pid_s_stir_run;
	
	
}

void Shoot_Heart(shoot_t * shoot)
{
	if(shoot->fric_left->info->status  == _DEV_OFFLINE || \
	   shoot->fric_right->info->status == _DEV_OFFLINE)
	{
		shoot->base.status = Shoot_Offline;
	}
	else
	{
		shoot->base.status = Shoot_Online;
	}

}

/*重装载*/
void Shoot_Reload(shoot_t * shoot)
{
	uint8_t cnt = 0;
	
	while(cnt++ < RELOAD_CNT || shoot->flag.locked == 0)
	{
		Check_One_Shoot(shoot);
		
		if(shoot->flag.one_shoot_done == 1)//上次装载完成
		{
			shoot->stir_wheel->base_info->target_angle_sum-= (int32_t)8191*4.5;//单发
		}
	}
	//can_send_data
	
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
	shoot->stir_wheel->base_info->target_speed         = pid_calc(shoot->pid_p_stir_sin,\
																	shoot->stir_wheel->base_info->done_angle,\
																	shoot->stir_wheel->base_info->target_angle_sum);
	/*波蛋轮速度环PID*/
	shoot->stir_wheel->output_current  = pid_calc(shoot->pid_s_stir_sin,\
										shoot->stir_wheel->base_info->speed,\
										shoot->stir_wheel->base_info->target_speed);
	//can_send_data
}



/*检测单发上弹是否完成*/
void Check_One_Shoot(shoot_t * shoot)
{
	if(shoot->flag.one_shoot_done == 0)
	{
		if(abs(shoot->stir_wheel->base_info->angle_sum -shoot->stir_wheel->base_info->target_angle_sum) <= 20)//shoot_structure.stir_wheel->base_info->angle_sum -shoot_structure.stir_wheel->base_info->target_angle_sum
		{
			shoot->flag.one_shoot_done = 1;
		}
	}
}

uint8_t init_ok = 1;

/*堵转检测*/
void Done_Check(shoot_t * shoot)
{
	if( init_ok &&abs(shoot->stir_wheel->base_info->speed - shoot->base.shoot_speed) >=  500)
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


void Fric_Work(shoot_t * shoot)
{
	
	
	/*摩擦轮速度环PID*/
	shoot->fric_left->output_current  = pid_calc(shoot->pid_s_left,\
										shoot->fric_left->base_info->speed,\
										(-1)*shoot->base.firct_speed);


	shoot->fric_right->output_current = pid_calc(shoot->pid_s_righ,\
										shoot->fric_right->base_info->speed,\
										shoot->base.firct_speed);

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
        shoot->stir_wheel->output_current = pid_calc(&pid_p_stir_done,\
														shoot->stir_wheel->base_info->speed,\
														DEAL_DONE_SPEED);	//反转
	}
	else
	{
		/*波蛋轮速度环PID*/
        shoot->stir_wheel->output_current = pid_calc(&pid_p_stir_done,\
														shoot->stir_wheel->base_info->speed,\
														(-1)*DEAL_DONE_SPEED);//正转
	}
	
	if(shoot->cnt.deal_time == DEAL_DONE_ROUND_TIME)
	{
		shoot->cnt.deal_time = 0;
		shoot->cnt.done_time = 0;//结束堵转
		shoot->flag.locked = 0;
	}	
	
}

