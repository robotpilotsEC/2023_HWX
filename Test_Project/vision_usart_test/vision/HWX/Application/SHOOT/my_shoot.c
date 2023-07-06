
#include "my_shoot.h"
#include "car.h"
#include "dji_pid.h"
#include "REMOTE.h"
#include "drv_tim.h"
#include "judge_infantrypotocol.h"
#include "judge_sensor.h"

/*波胆点击*/
extern motor_2006_t           motor_2006_PLUCK_structure;
extern motor_2006_base_info_t motor_2006_PLUCK_base_info;
extern motor_2006_info_t      motor_2006_PLUCK_info;

/*摩擦轮*/
extern motor_3508_t           motor_3508_FIRC_L_structure;
extern motor_3508_base_info_t motor_3508_FIRC_L_base_info;
extern motor_3508_info_t      motor_3508_FIRC_L_info;

extern motor_3508_t           motor_3508_FIRC_R_structure;
extern motor_3508_base_info_t motor_3508_FIRC_R_base_info;
extern motor_3508_info_t      motor_3508_FIRC_R_info;

/*PID发射*/
extern pid_t                  FIRC_L_speed_structure;
extern pid_t                  FIRC_R_speed_structure;
extern pid_t                  STRI_three_speed_structure;
extern pid_t                  STRI_position_structure;
extern pid_t                  STRI_one_speed_structure;
extern pid_t                  shoot_deal_done_structure;
extern pid_t                  shoot_judge_structure;
/*车状态*/
extern car_t                  car_structure;

/*发射机构*/
extern shoot_t                shoot_structure; 
extern int8_t                 fric_work;

/*裁判系统*/
extern judge_sensor_t         judge_sensor;
/*遥控器*/
extern rc_t                   rc_structure;
extern int8_t                 s2_down_flag;
extern int8_t                 s2_up_flag;
	   int8_t                 s2_up_last_flag = 1;
       int16_t                shoot_offline_cnt = 0;

       int8_t                 init_ok = 0;//连发模式下拨码轮是否到达目标转速
       int8_t	              deal_done = 0;//堵转正在处理
	   int8_t                 over_heat = 0;
extern int8_t           	  one_shoot_done;
extern bool                   hight_speed_shoot;

/**
  * @Name    Cover
  * @brief  
  * @param   弹舱开关 
  * @retval
  * @author  HWX
  * @Date    2022-10-20
**/
void Cover_Open(void)
{
	COVER_PwmOut(50);//180
}


void Cover_Close(void)
{
	COVER_PwmOut(113);//250
}



/**
  * @Name    SHOOT_INIT
  * @brief  
  * @param   shoot: [输入/出] 
  * @retval
  * @author  HWX
  * @Date    2022-10-20
**/
void SHOOT_INIT(shoot_t * shoot)
{
    /*电机们 初始化*/
    MOTOR_3508_INIT(&motor_3508_FIRC_L_structure, &motor_3508_FIRC_L_base_info,&motor_3508_FIRC_L_info );
    MOTOR_3508_INIT(&motor_3508_FIRC_R_structure, &motor_3508_FIRC_R_base_info,&motor_3508_FIRC_R_info );
    MOTOR_2006_INIT(&motor_2006_PLUCK_structure,  &motor_2006_PLUCK_base_info,&motor_2006_PLUCK_info );

    /*摩擦轮速度环*/
    PID_struct_init(&FIRC_L_speed_structure,POSITION_PID,PID_FIRC_OUTPUT_MAX,PID_FIRC_INTEGRA_MAX,PID_FIRC_KP,PID_FIRC_KI,PID_FIRC_KD);
    PID_struct_init(&FIRC_R_speed_structure,POSITION_PID,PID_FIRC_OUTPUT_MAX,PID_FIRC_INTEGRA_MAX,PID_FIRC_KP,PID_FIRC_KI,PID_FIRC_KD);

    /*单发位置环*/
    PID_struct_init(&STRI_one_speed_structure,POSITION_PID,SPID_STRI_OUTPUT_MAX,SPID_STRI_INTEGRA_MAX,SPID_STRI_KP,SPID_STRI_KI,SPID_STRI_KD);
    PID_struct_init(&STRI_position_structure,POSITION_PID,PPID_STRI_OUTPUT_MAX,PPID_STRI_INTEGRA_MAX,PPID_STRI_KP,PPID_STRI_KI,PPID_STRI_KD);

    /*连发速度环*/
    PID_struct_init(&STRI_three_speed_structure,POSITION_PID,TPID_STRI_OUTPUT_MAX,TPID_STRI_INTEGRA_MAX,TPID_STRI_KP,TPID_STRI_KI,TPID_STRI_KD);
	
	/*堵转处理位置环*/
	PID_struct_init(&shoot_deal_done_structure,POSITION_PID,PID_DONE_OUTPUT_MAX,PID_DONE_INTEGRA_MAX,PID_DONE_KP,PID_DONE_KI,PID_DONE_KD);
	
	/*裁判系统速度环*/
	 PID_struct_init(&shoot_judge_structure,POSITION_PID,TPID_STRI_OUTPUT_MAX,TPID_STRI_INTEGRA_MAX,TPID_STRI_KP,TPID_STRI_KI,TPID_STRI_KD);
	
	
    shoot->fric_left  = &motor_3508_FIRC_L_structure;
    shoot->fric_right = &motor_3508_FIRC_R_structure;
    shoot->stir_wheel = &motor_2006_PLUCK_structure;
    shoot->work_sta   = S_S_offline;
    shoot->cnt_three_done   = SHOOT_OFFLINE_CNT_MAX;
    shoot->cnt_right  = SHOOT_OFFLINE_CNT_MAX;
    shoot-> cnt_stri  = SHOOT_OFFLINE_CNT_MAX;
	shoot->stri_mode  = NOT_SHOOT;
    shoot->shoot_time = 0;
    shoot->shoot_speed= 0;
	shoot->box_open   = 0;
	shoot->done_time  = 0; 
	shoot->six_done_time  = 0; 
    shoot->init  = SHOOT_INIT;
    shoot->check = DONE_CHECK;
    shoot->update= MODE_UPDATE;
    shoot->data  = DATA_UPDATE;
    shoot->work  = SHOOR_WORK;
    shoot->she_ji= SHE_JI;

}



/**
  * @brief  模式更新
  * @param
  * @retval
  */

void MODE_UPDATE(shoot_t* shoot)
{

	if(car_structure.ctrl_mode == RC_CAR)
	{
		/*车辆模式匹配*/
		switch (car_structure.move_mode_status)
		{
		case offline_CAR:
		{

			shoot->work_sta = S_S_offline;

			/*PID目标值清零*/
			shoot->stir_wheel->base_info->target_speed = 0;
			shoot->shoot_speed = 0;
			shoot->stir_wheel->base_info->target_angle_sum = 0;
			/*标志位清零*/
			s2_up_flag      = 1;
			s2_down_flag    = 1;
			s2_up_last_flag = 1;
			shoot->stir_wheel->base_info->circle_cnt = 0;
			shoot->stir_wheel->base_info->angle_sum    = 0;

			/*泄力*/
			shoot_offline_cnt++;
			
			/*弹舱舵机卸力*/
			COVER_PwmOut(230);//很神奇230没力

			break;

		}

		case machine_CAR:
		{
			shoot->work_sta = S_S_oneshot;

			if(!deal_done)//若正在处理堵转，堵转程序完全获得对拨码轮速度的操作权
			{
				/*拨码轮转动一格*/
				if(s2_up_last_flag != s2_up_flag )
				{

					shoot->stir_wheel->base_info->target_angle_sum -= (int32_t)8191*4.5; //8191*4.5  36860

				}
			}

			s2_up_last_flag = s2_up_flag;

			/*摩擦轮开关*/
			if(s2_down_flag != 1  || shoot->box_open == 1)
			{
				Cover_Open();//开启弹舱
			}
			else
			{
				Cover_Close();//关闭弹舱
			}

			break;
		}

		case follow_CAR:
		{

			shoot->work_sta = S_S_threeshot;

			if(!deal_done)//若正在处理堵转，堵转程序完全获得对拨码轮速度的操作权
			{
				/*波蛋轮开关*/
				if(s2_up_flag != 1)
				{
					shoot->stir_wheel->base_info->target_speed = SHOOT_STRI_SPEED;//开启波蛋轮

					/*init_ok标志位更新*/
					if(shoot->stir_wheel->base_info->speed >= (SHOOT_STRI_SPEED - SHOOT_STRI_SPEED_ERR))
					{
						init_ok = 1;//一旦大于就 = 1，直到拨码轮关闭再清零
					}
				}
				else
				{
					shoot->stir_wheel->base_info->target_speed = 0;//关闭波蛋轮
					init_ok = 0;
				}
			}

			/*摩擦轮开关*/
			if(s2_down_flag != 1  )
			{
				shoot->shoot_speed = SHOOT_STRI_SPEED;//开启摩擦轮

			}
			else
			{
				shoot->shoot_speed = 0;//关闭摩擦轮
			}
			
			/*陀螺仪模式关弹舱*/
			Cover_Close();

			break;
		}
		}

		rc_structure.base_info->s2.value_last = rc_structure.base_info->s2.value;

		SHOOR_WORK(&shoot_structure);
	}
	/*键盘模式控制发射机构*/
	else if(car_structure.ctrl_mode == KEY_CAR)
	{
		
		/*跟随模式一直开启摩擦轮?*/
		if(car_structure.move_mode_status == machine_CAR)
		{
			shoot->shoot_speed = 0;//关闭摩擦轮
			shoot->stir_wheel->base_info->target_speed = 0;
			SHOOR_WORK(&shoot_structure);
		}
		else if(car_structure.move_mode_status == offline_CAR)
		{
			shoot->work_sta = S_S_offline;

			/*PID目标值清零*/
			shoot->stir_wheel->base_info->target_speed = 0;
			shoot->shoot_speed = 0;
			shoot->stir_wheel->base_info->target_angle_sum = 0;
			shoot->stir_wheel->base_info->angle_sum = 0;
			/*标志位清零*/
			s2_up_flag      = 1;
			s2_down_flag    = 1;
			s2_up_last_flag = 1;
			shoot->stir_wheel->base_info->circle_cnt = 0;
			shoot->stir_wheel->base_info->angle      = 0;
			

			/*泄力*/
			shoot_offline_cnt++;
			
			/*弹舱舵机卸力*/
			COVER_PwmOut(230);//很神奇230没力
		}
		else
		{
			//shoot->shoot_speed = SHOOT_SPEED *fric_work;//开启摩擦轮
			shoot->shoot_speed = SHOOT_STRI_SPEED *fric_work;//开启摩擦轮
			
			if(deal_done == 0)
			{
				
				if(shoot->stir_wheel->base_info->speed >= SHOOT_STRI_SPEED )
					{
							init_ok = 1;//波蛋轮初始化完成
					}
				/*波蛋轮控制*/
				switch(shoot->stri_mode)
				{
					case ONE_SHOOT:
						//car.c控制
					break;
					
					case NOT_SHOOT:
						
						shoot->stir_wheel->base_info->target_speed = 0;
						init_ok = 0;//重置波蛋轮状态
					
					break;
					
					case SIX_SHOOT:
						shoot->stir_wheel->base_info->target_speed = SHOOT_STRI_SPEED;
						
					
					break;
				
				}
				
				
				SHOOR_WORK(&shoot_structure);
			}
			/*堵转处理*/
			else if(deal_done == 1)
			{
				DONE_WORK(&shoot_structure);
			}
		}
		
		
		
		/*弹舱开关*/
		if(shoot->box_open == 1)
		{
			Cover_Open();//开启弹舱
		}
		else
		{
			Cover_Close();//关闭弹舱
		}		
	}
	
}



/**
  * @brief  发射机构工作
  * @param
  * @retval
  */
void SHOOR_WORK(shoot_t* shoot)
{

    //我看过曲线图，确实是会有一一点点的突变，不知道是为什么，所以采取角度差分是好一点的，但是那样不准确？
    if(shoot->work_sta == S_S_oneshot || shoot->stri_mode == ONE_SHOOT)
    {
        /*波蛋轮位置环PID*/
        shoot->stir_wheel->base_info->target_speed = pid_calc(&STRI_position_structure,\
                shoot->stir_wheel->base_info->angle_sum,\
                shoot->stir_wheel->base_info->target_angle_sum);

        /*波蛋轮速度环PID*/ //转那么多圈过什么零点
        shoot->stir_wheel->output_current          = pid_calc(&STRI_one_speed_structure,\
                shoot->stir_wheel->base_info->speed,\
                shoot->stir_wheel->base_info->target_speed);

    }

    else if(shoot->work_sta == S_S_threeshot || shoot->stri_mode == SIX_SHOOT || shoot->stri_mode == NOT_SHOOT )
    {


        /*波蛋轮速度环PID*/
        shoot->stir_wheel->output_current = pid_calc(&STRI_three_speed_structure,\
                                            shoot->stir_wheel->base_info->speed,\
                                            shoot->stir_wheel->base_info->target_speed);
    }
	
	
		/*摩擦轮速度环PID*/
	shoot->fric_left->output_current  = pid_calc(&FIRC_L_speed_structure,\
										shoot->fric_left->base_info->speed,\
										(-1)*shoot->shoot_speed);


	shoot->fric_right->output_current = pid_calc(&FIRC_R_speed_structure,\
										shoot->fric_right->base_info->speed,\
										shoot->shoot_speed);
}


/**
  * @brief  堵转检测
  * @param  abs(shoot->stir_wheel->base_info->speed) <= SHOOT_DONE_SPEED_MAX
  * @retval init_ok
  */
/*
首先是要检测到堵转（保证启动的时候不会发生检测，所以需要一个shoot_init_ok标志位）
其次是处理，发生小范围的抖动？还是反转，我觉得可以将波蛋轮的控制权给一个新的位置环pid，进行往返运动
每运动一个周期，就检测是否堵转结束，堵转应该是发射机构优先级最高的状态


*/
void DONE_CHECK(shoot_t* shoot)//能转动就清零？
{
	
	/*连发堵转*/
    if(shoot->work_sta != S_S_offline || shoot_structure.stri_mode == SIX_SHOOT)
    {
        if((init_ok && abs(shoot->stir_wheel->base_info->speed) <= SHOOT_DONE_SPEED_MAX))//初始化完成 且堵转
        {
            if(shoot->cnt_three_done >= SHOOT_DONE_TIME_MAX)//堵转时间达到能忍受的最大值
            {
                deal_done = 1;//表示正在处理堵转（思考如何判断解除堵转）
				//进入堵转处理位置环 //shoot->stir_wheel->base_info->target_speed *= (-1);//速度取反
            }
            else
            {
                shoot->cnt_three_done++;
            }

        }
        else
        {
			if(shoot->stir_wheel->base_info->speed <=(SHOOT_STRI_SPEED - SHOOT_STRI_SPEED_ERR))
			{
				shoot->cnt_three_done  = 0;//连发模式未发生堵转
				deal_done              = 0;//结束堵转
				shoot->done_time       = 0;//堵转时间计数器清零
			}

        }
    }
	else if(shoot_structure.stri_mode == ONE_SHOOT)
	{
		if(abs(shoot->stir_wheel->base_info->angle_sum - shoot->stir_wheel->base_info->target_angle_sum) >=  5000)
		{
			
			if( shoot->six_done_time >= SIX_SHOOT_DONE_TIME_MAX)//堵转时间达到能忍受的最大值
            {
                deal_done = 1;//表示正在处理堵转（思考如何判断解除堵转）
				//进入堵转处理位置环 //shoot->stir_wheel->base_info->target_speed *= (-1);//速度取反
            }
            else
            {
                shoot->six_done_time++;;
            }		
		
		}
        else
        {
			if( abs(shoot->stir_wheel->base_info->angle_sum - shoot->stir_wheel->base_info->target_angle_sum) <=(SHOOT_STRI_SPEED - SHOOT_STRI_SPEED_ERR))
			{
				shoot->six_done_time  = 0;//连发模式未发生堵转
				deal_done              = 0;//结束堵转
				shoot->done_time       = 0;//堵转时间计数器清零
			}

        }
	
	
	}
	
	
	/*摩擦轮速度环PID（堵转处理别忘了处理摩擦轮）*/
	shoot->fric_left->output_current  = pid_calc(&FIRC_L_speed_structure,\
										shoot->fric_left->base_info->speed,\
										(-1)*shoot->shoot_speed);


	shoot->fric_right->output_current = pid_calc(&FIRC_R_speed_structure,\
										shoot->fric_right->base_info->speed,\
										shoot->shoot_speed);

}


/**
  * @brief  发射机构总任务
  * @param
  * @retval 
  */
void shoot_task()
{
    /*发射模式更新*/
    MODE_UPDATE( &shoot_structure);//在这决定发射数据

    /*发射数据更新*/
    DATA_UPDATE(&shoot_structure);

	/*裁判系统数据更新*/
    DONE_CHECK(&shoot_structure);
	

    /*卸力*/
    if(shoot_offline_cnt >= 100)
    {
        shoot_offline_cnt = 100;

        MOTOR_3508_CAN2_SENT_DATA(0,0,0,0);//注意顺序，最后一个一定是0
    }
    else
    {
        /*电机数据发送*/
        MOTOR_3508_CAN2_SENT_DATA(motor_3508_FIRC_R_structure.output_current,\
                                  motor_3508_FIRC_L_structure.output_current,\
                                  motor_2006_PLUCK_structure.output_current,\
                                  0);//注意顺序，最后一个一定是0
    }
}


/**
  * @brief  堵转处理
  * @param
  * @retval 应该要一个堵转时间计数器
  */
void DONE_WORK(shoot_t* shoot)
{
	if( ( (shoot->done_time++) %DEAL_DONE_ROUND_TIME)  >= DEAL_DONE_ROUND_HALF_TIME)
	{
		 /*波蛋轮速度环PID*/
        shoot->stir_wheel->output_current = pid_calc(&shoot_deal_done_structure,\
														shoot->stir_wheel->base_info->speed,\
														DEAL_DONE_SPEED);	//反转
	}
	else
	{
		/*波蛋轮速度环PID*/
        shoot->stir_wheel->output_current = pid_calc(&shoot_deal_done_structure,\
														shoot->stir_wheel->base_info->speed,\
														(-1)*DEAL_DONE_SPEED);//正转
	}
	
	if(shoot->done_time == DEAL_DONE_ROUND_TIME)
	{
		deal_done = 0;//结束堵转
		shoot->cnt_three_done  = 0;//连发模式未发生堵转
		shoot->cnt_three_done  = 0;//连发模式未发生堵转
		shoot->done_time       = 0;//堵转时间计数器清零
	}	
	
}




/**
  * @brief  裁判系统数据更新
  * @param	hight_speed_shoot 1 表示开启
  * @retval 高速 打蛋模式
  */
void DATA_UPDATE(shoot_t* shoot)
{
	
	   shoot->cooling_heat  = judge_sensor.info->power_heat_data.shooter_id1_17mm_cooling_heat;
	   shoot->cooling_limit = judge_sensor.info->game_robot_status.shooter_id1_17mm_cooling_limit;
		
		if(shoot->cooling_heat >= shoot->cooling_limit - 9)
		{
			 /*波蛋轮速度环PID*/
			shoot->stir_wheel->output_current = pid_calc(&shoot_judge_structure,\
													      shoot->stir_wheel->base_info->speed,\
														  0);
			over_heat = 1;
		}
		else
		{
			over_heat = 0;
		}
		
	
}
/**
  * @brief  热量限制射击
  * @param
  * @retval
  */
void SHE_JI(shoot_t* shoot)
{
	

}




