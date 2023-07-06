/**
  ******************************************************************************
  * @file           : launcher.c/h
  * @brief          : 
  * @note           : finish 2022-3-3 16:25:32
	*                   2022-3-12 更新dial_work函数（改为拨盘是否静止判断卡弹）
	*                   2022-3-12 增加dial_work_to_keep函数
	*                   2022-3-17 增加滴答任务，识别左摩擦轮失联状态
	*                   2022-3-17 更新launcher_mode_update函数，若左摩擦轮失联，
	*                         则停止工作
	*                   2022-3-19 限位轮工作与拨盘工作之间增加延时
  ******************************************************************************
  */

#include "launcher.h"
#include "launcher_motor.h"
#include "config.h"
#include "config_launcher.h"
#include "car.h"
#include "main.h"
#include "judge.h"
#include "math_support.h"

dial_work_info_t dial_work_info;
launcher_config_t launcher_config = 
{
	.friction_motor_work_speed = friction_motor_work_speed_init,
	.position_motor_work_speed = position_motor_work_speed_init,
	.position_motor_work_cnt = position_motor_work_cnt_init,
	.dial_motor_work_delay_cnt = dial_motor_work_delay_cnt_init,
	.dial_front_output_max = dial_front_output_max_init,
	.dial_back_output_max = dial_back_output_max_init,
	.dial_keep_output_max = dial_keep_output_max_init,
	.auto_shoot_T = auto_shoot_T_init,
	.Reload_Times =       Reload_Times_init,
	.F_lock_Times =       F_lock_Times_init,
	.Unload_Times =       Unload_Times_init,
	.B_lock_Times =       B_lock_Times_init,
	.Reload_angle_check = Reload_angle_check_init,
	.Reload_angle =       Reload_angle_init,
	.Unload_angle_check = Unload_angle_check_init,
	.Unload_angle =       Unload_angle_init,
	.F_lock_angle_check = F_lock_angle_check_init,
	.F_lock_angle =       F_lock_angle_init,
	.B_lock_angle_check = B_lock_angle_check_init,
	.B_lock_angle =       B_lock_angle_init,
	.Reload_cnt =         Reload_cnt_init,
	.F_lock_cnt =         F_lock_cnt_init,
	.B_lock_cnt =         B_lock_cnt_init,
	.Unload_cnt =         Unload_cnt_init,
};
launcher_info_t launcher_info;
launcher_work_info_t launcher_work_info = 
{
	.dial_work_info = &dial_work_info,
};
launcher_t launcher = 
{
	.dial = &dial_motor,
	.friction_left = &friction_left_motor,
	.friction_right = &friction_right_motor,
	.position = &position_motor,
	.config = &launcher_config,
	.info = &launcher_info,
	.work_info = &launcher_work_info,
};/* 发射机构 */

/**
  * @brief  发射机构初始化
  * @param  
  * @retval 
  */
void launcher_init(launcher_t *launcher)
{
	/* 发射机构电机初始化 */
	launcher->dial->init(launcher->dial);
	launcher->friction_left->init(launcher->friction_left);
	launcher->friction_right->init(launcher->friction_right);
	launcher->position->init(launcher->position);
	
	/* 发射机构信息初始化 */
  launcher->info->mode = L_stop;
	
	/* 发射机构工作信息初始化 */
	launcher_work_info_init(launcher->work_info);
	
	/* 发射机构命令初始化 */
	launcher_commond_init(launcher);
}

/**
  * @brief  发射机构工作信息初始化
  * @param  
  * @retval 
  */
void launcher_work_info_init(launcher_work_info_t *info)
{
	/* 拨盘工作初始化 */
	dial_work_init(info->dial_work_info);
	
	info->commond = Done_L;
	info->status = Done_L;
	info->Ready_OR_Not = 0;
}

/**
  * @brief  拨盘工作初始化
  * @param  
  * @retval 
  */
void dial_work_init(dial_work_info_t *info)
{
	info->commond = Keep_D;
	info->status = Keep_D;
}

/**
  * @brief  发射机构命令初始化
  * @param  
  * @retval 
  */
void launcher_commond_init(launcher_t *launcher)
{
  Launcher_Shoot = false;
	Launcher_Relax = false;
	Launcher_Stop = false;
	dial_reload = false;
	dial_unload = false;
}

/**
  * @brief  发射机构控制任务
  * @param  
  * @retval 
  */
void launcher_ctrl_task(launcher_t *launcher)
{
	/* 发射机构模式更新 */
	launcher_mode_update(launcher);
	
	/* 裁判系统限制 */
	judge_limit(launcher);
	
	/* 发射机构命令响应 */
	launcher_commond_respond(launcher);
	
	/* 发射机构工作 */
	#ifndef LAUNCHER_PART_OFF
	launcher_work(launcher);
	#endif
	
	/* 拨盘工作 */
	#ifndef DIAL_PART_OFF
	dial_work(launcher->work_info->dial_work_info);
	#endif
	
	/* 发射机构命令初始化 */
	launcher_commond_init(launcher);
}

/**
  * @brief  发射机构模式更新
  * @param  
  * @retval 
  */
void launcher_mode_update(launcher_t *launcher)
{
	switch(car.move_mode_status)
	{
		case gyro_CAR:
		case high_shoot_CAR:
		case machine_CAR:
			launcher->info->mode = L_work;
			break;
		default:
			launcher->info->mode = L_stop;
			break;
	}
	
	if(launcher->friction_left->info->status == DEV_OFFLINE)
	{
		launcher->info->mode = L_stop;
		launcher->work_info->commond = Stop_L;
		launcher->work_info->dial_work_info->commond = Keep_D;
	}
}

/**
  * @brief  发射机构命令响应
  * @param  
  * @retval 
  */
void launcher_commond_respond(launcher_t *launcher)
{
	if(Launcher_Shoot == true)
	{
		launcher->work_info->commond = Shoot_L;
	}
	if(Launcher_Relax == true)
	{
		launcher->work_info->commond = Relax_L;
	}
	if(Launcher_Stop == true)
	{
		launcher->work_info->commond = Stop_L;
	}
	if(dial_reload == true)
	{
		launcher->work_info->dial_work_info->commond = Reload_D;
	}
	if(dial_unload == true)
	{
		launcher->work_info->dial_work_info->commond = Unload_D;
	}
}

/**
  * @brief  发射机构工作
  * @param  
  * @retval 
  */
void launcher_work(launcher_t *launcher)
{
	if(launcher->info->mode == L_stop)
	{
		launcher->work_info->commond = Stop_L;
	}
	
	switch (launcher->work_info->status)
	{
		case Stop_L:
			launcher->work_info->Ready_OR_Not = 0;
			launcher->friction_left->pid_speed->info->target = 0;
			launcher->friction_right->pid_speed->info->target = 0;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);//关激光
			launcher->work_info->status = Done_L;
			launcher->work_info->cnt = 0;
			break;
		case Ready_L:
			launcher->work_info->dial_work_info->commond = Reload_D;
			launcher->friction_left->pid_speed->info->target = - launcher->config->friction_motor_work_speed;
			launcher->friction_right->pid_speed->info->target = launcher->config->friction_motor_work_speed;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);//开激光
			if((launcher->friction_left->pid_speed->info->measure <= -launcher->config->friction_motor_work_speed + 500)&&\
				 (launcher->friction_right->pid_speed->info->target >= launcher->config->friction_motor_work_speed - 500)&&\
			   (launcher->work_info->dial_work_info->status == Keep_D))
			{
				launcher->work_info->status = Done_L;
				launcher->work_info->Ready_OR_Not = 1;
				launcher->work_info->cnt = 0;
			}
			if(launcher->work_info->commond == Stop_L)
			{
				launcher->work_info->status = Stop_L;
				launcher->work_info->cnt = 0;
			}
			break;
		case Shoot_L:
			launcher->position->pid_speed->info->target = - launcher->config->position_motor_work_speed;
			launcher->work_info->cnt ++;
			if(launcher->work_info->cnt == launcher->config->dial_motor_work_delay_cnt)
			{
				launcher->work_info->dial_work_info->commond = Reload_D;
			}
			if(launcher->work_info->cnt >= launcher->config->position_motor_work_cnt)
			{
				launcher->position->pid_speed->info->target = 0;
				launcher->work_info->status = Done_L;
				launcher->work_info->cnt = 0;
			}
			if(launcher->work_info->commond == Stop_L)
			{
				launcher->work_info->status = Stop_L;
				launcher->work_info->cnt = 0;
			}
			break;
		case Relax_L:
			switch(launcher->work_info->commond)
			{
				case Stop_L:
					launcher->work_info->status = Stop_L;
					launcher->work_info->cnt = 0;
					break;
				case Shoot_L:
					if(launcher->work_info->Ready_OR_Not == 1)
					{
						launcher->work_info->status = Shoot_L;
						launcher->work_info->cnt = 0;
					}
					else 
					{
						launcher->work_info->status = Ready_L;
						launcher->work_info->cnt = 0;
					}
					break;
				default:
					break;
			}
			break;
		case Done_L:
			/* 自动射击部分 */
//			launcher->work_info->cnt ++;
//			if(launcher->work_info->cnt >= launcher->config->auto_shoot_T)
//			{
//				launcher->work_info->status = Shoot_L;
//				launcher->work_info->cnt = 0;
//			}
			switch(launcher->work_info->commond)
			{
				case Stop_L:
					launcher->work_info->status = Stop_L;
					launcher->work_info->cnt = 0;
					break;
				case Relax_L:
					launcher->work_info->status = Relax_L;
					launcher->work_info->cnt = 0;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	launcher->friction_left->ctrl(launcher->friction_left);
	launcher->friction_right->ctrl(launcher->friction_right);
	launcher->position->ctrl(launcher->position);
}

/**
  * @brief  拨盘工作
  * @param  
  * @retval 
  */
void dial_work(dial_work_info_t *info)
{
	launcher_config_t *config = launcher.config;
	
	int32_t angle_measure, angle_target, speed_measure;
	
	angle_measure = launcher.dial->base_info->angle_sum;
	angle_target = launcher.dial->info->target_angle_sum;
	speed_measure = launcher.dial->base_info->speed;
	
	switch (info->status)
	{
		case Reload_D:
			if(angle_measure > angle_target - config->Reload_angle_check)
			{//到位
				if(info->work_times >= config->Reload_Times)
				{//超次数（无弹丸）
					angle_target = angle_measure;
					dial_work_to_keep(info);
				}
				else 
				{//继续补弹
					angle_target += config->Reload_angle;
					info->work_times++;
					info->cnt = 0;
				}
			}
			else 
			{//未到位
				if(abs(speed_measure) <= 2)
				{//拨盘不动
					info->cnt++;
					if(info->cnt >= config->Reload_cnt)
					{//卡弹
						info->lock_times++;
						if(info->lock_times >= config->F_lock_Times)
						{//卡弹超次数（补满弹丸）
							angle_target = angle_measure;
							dial_work_to_keep(info);
						}
						else 
						{//卡弹未超次数（卡弹处理）
							info->status = F_lock_D;
							info->angle_temp = angle_target - angle_measure + config->F_lock_angle;
							angle_target = angle_measure - config->F_lock_angle;
							info->cnt = 0;
						}
					}
				}
				else 
				{//拨盘在动
					info->cnt = 0;
				}
			}
			if(info->commond == Unload_D)
			{
				info->status = Unload_D;
				info->commond = Keep_D;
				angle_target = angle_measure - config->Unload_angle;
				launcher.dial->pid_speed->info->out_max = config->dial_back_output_max;
				info->work_times = 1;
				info->cnt = 0;
			}
			break;
		case F_lock_D:
			if(angle_measure < angle_target + config->F_lock_angle_check)
			{//到位
				info->status = Reload_D;
				angle_target += info->angle_temp;
				info->cnt = 0;
			}
			else 
			{//未到位
				if(abs(speed_measure) <= 2)
				{//拨盘不动
					info->cnt++;
					if(info->cnt >= config->F_lock_cnt)
					{//卡住
						info->status = Reload_D;
						angle_target += info->angle_temp;
						info->cnt = 0;
					}
				}
				else 
				{//拨盘在动
					info->cnt = 0;
				}
			}
			break;
		case Unload_D:
			launcher.work_info->Ready_OR_Not = 0;
			if(angle_measure < angle_target + config->Unload_angle_check)
			{//到位
				if(info->work_times >= config->Unload_Times)//超次数（弹丸最多退4个）
				{
					angle_target = angle_measure;
					dial_work_to_keep(info);
				}
				else 
				{//继续退弹
					angle_target -= config->Unload_angle;
					info->work_times++;
					info->cnt = 0;
				}
			}
			else 
			{//未到位
				if(abs(speed_measure) <= 2)
				{//拨盘不动
					info->cnt++;
					if(info->cnt >= config->B_lock_cnt)
					{//卡弹
						info->lock_times++;
						if(info->lock_times >= config->B_lock_Times)
						{//卡弹超次数
							angle_target = angle_measure;
							dial_work_to_keep(info);
						}
						else 
						{//卡弹未超次数（卡弹处理）
							info->status = B_lock_D;
							info->angle_temp = angle_target - angle_measure - config->B_lock_angle;
							angle_target = angle_measure + config->B_lock_angle;
							info->cnt = 0;
						}
					}
				}
			}
			if(info->commond == Reload_D)
			{
				info->status = Reload_D;
				angle_target = angle_measure + config->Reload_angle;
				info->cnt = 0;
				info->work_times = 1;
				info->lock_times = 0;
				info->commond = Keep_D;
				launcher.dial->pid_speed->info->out_max = config->dial_front_output_max;
			}
			break;
		case B_lock_D:
			if(angle_measure > angle_target - config->B_lock_angle_check)
			{//到位
				info->status = Unload_D;
				angle_target -= info->angle_temp;
				info->cnt = 0;
			}
			else 
			{//未到位
				if(abs(speed_measure) <= 2)
				{//拨盘不动
					info->cnt++;
					if(info->cnt >= config->B_lock_cnt)
					{//卡住
						info->status = Unload_D;
						angle_target -= info->angle_temp;
						info->cnt = 0;
					}
				}
				else 
				{//拨盘在动
					info->cnt = 0;
				}
			}
			break;
		case Keep_D:
			launcher.dial->pid_speed->info->out_max = config->dial_keep_output_max;
			angle_target = angle_measure;
			switch (info->commond)
			{
				case Reload_D:
					info->status = Reload_D;
					angle_target = angle_measure + config->Reload_angle;
					info->cnt = 0;
					info->work_times = 1;
					info->lock_times = 0;
					info->commond = Keep_D;
					launcher.dial->pid_speed->info->out_max = config->dial_front_output_max;
					break;
				case Unload_D:
					info->status = Unload_D;
					angle_target = angle_measure - config->Unload_angle;
					info->cnt = 0;
					info->work_times = 1;
					info->lock_times = 0;
					info->commond = Keep_D;
					launcher.dial->pid_speed->info->out_max = config->dial_back_output_max;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	launcher.dial->info->target_angle_sum = angle_target;
	
	launcher.dial->ctrl(launcher.dial);
}

void dial_work_to_keep(dial_work_info_t *info)
{
	info->status = Keep_D;
	info->work_times = 0;
	info->lock_times = 0;
	info->angle_temp = 0;
}

/**
  * @brief  裁判系统限制
  * @param  
  * @retval 
  */
void judge_limit(launcher_t *launcher)
{
	if(judge.info->status == DEV_ONLINE)
	{
		if(judge.base_info->shooter_cooling_limit - judge.base_info->shooter_cooling_heat < 100)
		{
			Launcher_Shoot = false;
			dial_reload = false;
		}
	}
}

/**
  * @brief  发射机构电机转速修改函数
  * @param  
  * @retval 
  */
void launcher_motor_speed_change(uint16_t speed)
{
	launcher.config->friction_motor_work_speed = speed;
}

/**
  * @brief  发射机构滴答任务
  * @param  
  * @retval 
  */
void launcher_tick_task(launcher_t *launcher)
{
	motor_3508_info_t *info = launcher->friction_left->info;
	
	info->offline_cnt++;
	if(info->offline_cnt >= info->offline_cnt_max)
	{
		info->status = DEV_OFFLINE;
	}
}
