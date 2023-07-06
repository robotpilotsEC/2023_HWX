#include "my_gimbal.h"
#include "my_chassis.h"
#include <string.h>
#include <stdbool.h>
#include "car.h"
#include "dji_pid.h"
#include "remote.h"
#include "bmi.h"
#include "rp_math.h"
#include "vision.h"
#include "my_shoot.h"



extern chassis_t              chassis_structure;


extern gimbal_work_info_t     gimbal_work_info_structure;
extern s_pitch_work_info_t    s_pitch_work_info_structure;
extern s_yaw_work_info_t      s_yaw_work_info_structure;
extern gimbal_info_t          gimbal_info_structure;
extern gimbal_config_t        gimbal_config_structure;
extern gimbal_t               gimbal_structure ;


extern motor_6020_t           motor_6020_YAW_structure;
extern motor_6020_base_info_t motor_6020_YAW_base_info;
extern motor_6020_info_t      motor_6020_YAW_info;


extern motor_6020_t           motor_6020_PIT_structure;
extern motor_6020_base_info_t motor_6020_PIT_base_info;
extern motor_6020_info_t      motor_6020_PIT_info;

extern car_t                  car_structure;
extern shoot_t                shoot_structure;
extern pid_t                  chassis_pid_position_structure;
extern pid_t                  chassis_pid_speed_structure;


extern pid_t                  yaw_lock_pid_speed_structure;
extern pid_t                  yaw_pid_speed_structure;
extern pid_t                  yaw_lock_pid_position_structure;
extern pid_t                  yaw_head_pid_position_structure;


extern pid_t                  yaw_follow_pid_position_structure;

extern pid_t                  pit_pid_position_structure;
extern pid_t                  pit_pid_speed_structure;

extern pid_t                  pit_pid_bmispeed_structure;
extern pid_t                  pit_pid_bmiposition_structure;


extern rc_t                   rc_structure;

extern car_t                  car_structure;

extern bmi_t                  bmi_structure;

extern vision_t               vision_structure;

       uint8_t                YAW_first2follow = 1;
	   uint8_t                PIT_first2follow = 1;
	   uint8_t                PIT_first2lock   = 1;
	   uint8_t                PIT_vision2lock  = 1;
	   uint8_t                YAW_lock2spin    = 1;
 	   
	   
extern int8_t                 switch_head;
  
/**
  * @brief  云台初始化
  * @param
  * @retval
  */
void gimbal_init (gimbal_t *gimbal)
{
    /* 电机初始化 */
    MOTOR_6020_INIT( &motor_6020_YAW_structure, &motor_6020_YAW_base_info, &motor_6020_YAW_info );
    MOTOR_6020_INIT( &motor_6020_PIT_structure, &motor_6020_PIT_base_info, &motor_6020_PIT_info );


    /* PID初始化 */

    /*机械模式YAW位置环*/
    PID_struct_init( &yaw_lock_pid_position_structure,  POSITION_PID,20000,    0,  10,    0, 0.5);
    /*机械模式YAW速度环*/
    PID_struct_init( &yaw_lock_pid_speed_structure,     POSITION_PID,20000, 5000,  20, 0.01,   0);


    /*跟随模式YAW位置环（地盘跟头）*/
    PID_struct_init( &yaw_follow_pid_position_structure,POSITION_PID,20000,    0,   3.5,   0,   0);
    /*跟随模式YAW速度环（总速度环）*/
    PID_struct_init( &yaw_pid_speed_structure,          POSITION_PID,20000, 2000,   10,  0.1,   0);
    /*跟随模式YAW位置环（云台转动）*/
    PID_struct_init( &yaw_head_pid_position_structure,  POSITION_PID,20000,    0,    7,    0,   0);


    /*PITCH的双环*/
    PID_struct_init( &pit_pid_speed_structure,          POSITION_PID,20000, 5000,  15,    0,   0);
    PID_struct_init( &pit_pid_position_structure,       POSITION_PID,20000,    0,  15,    0, 0.1);
	
	/*PITCH的双环(follow)*/
    PID_struct_init( &pit_pid_bmispeed_structure,          POSITION_PID,20000, 5000,  9,    0,   0);
    PID_struct_init( &pit_pid_bmiposition_structure,       POSITION_PID,20000,    0,  10,    0, 0.1);
	
	

    /*换头标志位*/
    gimbal->front_or_back = 1;

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
    info->target_pitch_angle =  PIT_MIDDLE;
	gimbal_structure.angle_45 = 0;
}

/**
  * @brief  云台命令初始化
  * @param
  * @retval
  */
void gimbal_commond_init(gimbal_t *gimbal)
{

}


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
    } 
	else if(car_structure.move_mode_status ==follow_CAR)
    {
        gimbal->work->status = G_G_follow;//跟随
    } 
	else if(car_structure.move_mode_status == offline_CAR)
    {
        gimbal->work->status = G_G_offline;//卸力
    } 
	else if(car_structure.move_mode_status == spin_CAR)
    {
        gimbal->work->status = G_G_spin;//小陀螺
    }
	else if(car_structure.move_mode_status == vision_CAR)
	{
		gimbal->work->status = G_G_vision;
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


    s_pitch_work_info_t *pitch_work = gimbal->pitch_work;
    s_yaw_work_info_t *yaw_work = gimbal->yaw_work;
    watch_work_info_t *watch_work = gimbal->watch_work;



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
        break;

    case G_G_lock:
        pitch_work->status =  G_P_lock;
        yaw_work->status   =  G_Y_lock;
        break;
	
    case G_G_spin:
        pitch_work->status =  G_P_normal;
        yaw_work->status   =  G_Y_spin;
        break;
	
	case G_G_vision:
	    pitch_work->status =  G_P_vision;
        yaw_work->status   =  G_Y_vision;
	
	
    default:
        break;
    }
    gimbal_pitch_work(gimbal);
    gimbal_yaw_work(gimbal);
}




void gimbal_pitch_work(gimbal_t *gimbal)
{

    gimbal_info_t *info = gimbal->info;
    s_pitch_work_info_t *work = gimbal->pitch_work;

    switch(work->status)
    {
    case G_P_offline:
        motor_6020_PIT_structure.output_current = 0;
        motor_6020_YAW_structure.output_current = 0;
		PIT_first2follow                        = 1;
        break;

    case G_P_normal:
		if(PIT_first2follow) 
		{
			info->target_pitch_angle = bmi_structure.pit_angle;//保证不乱动
			PIT_first2follow               = 0;
		}
		
        switch(car_structure.ctrl_mode)
        {		
        case RC_CAR:
            info->target_pitch_angle = info->target_pitch_angle + (int16_t)(rc_structure.base_info->ch1 * (-0.01));
            gimbal_pitch_angle_check(gimbal);
            gimbal_pitch_angle_ctrl(gimbal);
            break;
        case KEY_CAR:
            info->target_pitch_angle += rc_structure.info->mouse_y_K / 30.f;
            gimbal_pitch_angle_check(gimbal);
            gimbal_pitch_angle_ctrl(gimbal);
            break;
        default:
            break;
        }
        break;


    case G_P_lock:
		
		if(PIT_first2follow == 0)
		{
			info->target_pitch_angle = gimbal->pitch->base_info->angle;//直接目标值等于机械角度就好
		}
		PIT_first2follow = 1;
		
		if(PIT_vision2lock == 1 )
		{
			info->target_pitch_angle = gimbal->pitch->base_info->angle;//直接目标值等于机械角度就好
		}
		PIT_vision2lock = 0;
        switch(car_structure.ctrl_mode)
        {
        case RC_CAR:
            info->target_pitch_angle = info->target_pitch_angle + (int16_t)(rc_structure.base_info->ch1 * (-0.01));
			gimbal_pitch_angle_check(gimbal);
            gimbal_pitch_angle_ctrl(gimbal);
            break;
		
		case KEY_CAR:
			if(!shoot_structure.box_open)
			{
				info->target_pitch_angle += rc_structure.info->mouse_y_K / 20.f;
			}
			else
			{
				info->target_pitch_angle = PIT_MIDDLE;
			}
			gimbal_pitch_angle_check(gimbal);
            gimbal_pitch_angle_ctrl(gimbal);
			
        default:
            break;
        }
        break;
		
	case G_P_vision:
		info->target_pitch_angle = vision_structure.rx_pack->RxData.pitch_angle + 4000;
		PIT_first2follow         = 1;
		PIT_vision2lock          = 1;
		//vision_task();//测试用
		gimbal_pitch_angle_check(gimbal);
        gimbal_pitch_angle_ctrl(gimbal);
		break;
    }
}







void gimbal_yaw_work(gimbal_t *gimbal)
{
    s_yaw_work_info_t *work = gimbal->yaw_work;

    gimbal_info_t *info = gimbal->info;

    switch(work->status)
    {
    case G_Y_offline:
        info->target_yaw_speed = 0;
        motor_6020_YAW_structure.output_current = 0;
		YAW_first2follow                        = 1;
        break;
	
    case G_Y_angle:
		/*只有进跟随模式需要头尾，头尾检测在 car.c中完成*/
        /*避免模式切换时，转半天*/
        if(YAW_first2follow) {
            gimbal->info->target_yaw_angle = bmi_structure.yaw_angle;//从机械切跟随的时候，头不会乱动
			YAW_first2follow                   = 0;
        }
		switch(car_structure.ctrl_mode)
		{	
			case RC_CAR:
				gimbal->info->target_yaw_angle = gimbal->info->target_yaw_angle + (int16_t)(rc_structure.base_info->ch0*(-0.02)) ;
				gimbal_yaw_angle_check(gimbal);
				gimbal_yaw_angle_ctrl(gimbal);
			break;
			
			case KEY_CAR:
				//info->target_pitch_angle -= rc_structure.info->mouse_x_K / 3.f;
				info->target_yaw_angle -= (!switch_head)*rc_structure.info->mouse_x_K /10.f;
				gimbal_yaw_angle_check(gimbal);
				gimbal_yaw_angle_ctrl(gimbal);
				
			break;
		}
        break;

    case G_Y_lock://锁住不用键盘
        gimbal->info->target_yaw_angle = YAW_MIDDLE_FOUNT;
        YAW_first2follow = 1;
		YAW_lock2spin    = 1;
        gimbal_yaw_angle_check(gimbal);
        gimbal_yaw_angle_lock_ctrl(gimbal);
        break;

    case G_Y_spin://switch_head_cnt++ >= SWITCH_HEAD_TIME_MAX  #define SWITCH_HEAD_TIME_MAX  2000
		if(YAW_lock2spin) {
            gimbal->info->target_yaw_angle = bmi_structure.yaw_angle;//从机械切跟随的时候，头不会乱动
			YAW_lock2spin                  = 0;
        }
		
		switch(car_structure.ctrl_mode)
		{	
			case RC_CAR:
				gimbal->info->target_yaw_angle = gimbal->info->target_yaw_angle + (int16_t)(rc_structure.base_info->ch0*(-0.01)) ;
				gimbal_yaw_angle_check(gimbal);
				gimbal_yaw_angle_ctrl(gimbal);
			break;

			case KEY_CAR:
				info->target_yaw_angle -= rc_structure.info->mouse_x_K / 10.f;
				gimbal_yaw_angle_check(gimbal);
				gimbal_yaw_angle_ctrl(gimbal);
			break;

		}
        break;
	
	case G_Y_vision://锁住不用键盘
		info->target_yaw_angle = vision_structure.rx_pack->RxData.yaw_angle;
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

	if(gimbal->work->status ==G_G_vision )
	{	
		if(info->target_pitch_angle > 8191)//目标判断
		{
			info->target_pitch_angle -= 8192 ;

		} else if(info->target_pitch_angle < 0)
		{
			info->target_pitch_angle += 8192 ;
		}
	}
	else if( gimbal->work->status ==G_G_follow || gimbal->work->status ==G_G_spin )//解决小陀螺低头问题
	{
		if(info->target_pitch_angle > PIT_B_UP_RANGE)
		{
			info->target_pitch_angle = PIT_B_UP_RANGE;
		}
		if(info->target_pitch_angle < PIT_B_DOWN_RANGE)
		{
			info->target_pitch_angle = PIT_B_DOWN_RANGE;
		}
	}
	else
	{
		if(info->target_pitch_angle > PIT_M_DOWN_RANGE)
		{
			info->target_pitch_angle = PIT_M_DOWN_RANGE;
		}
		if(info->target_pitch_angle < PIT_M_UP_RANGE)
		{
			info->target_pitch_angle = PIT_M_UP_RANGE;
		}
	}
	
	

}





/*pitch电机 速度环控制（内环）  6020*/
void gimbal_pitch_speed_ctrl(gimbal_t *gimbal)
{	
	//其实只是用不同的PID
	if(gimbal->work->status !=G_G_follow && \
	   gimbal->work->status !=G_G_vision && \
	   gimbal->work->status !=G_G_spin)
	{
		gimbal->pitch->output_current = pid_calc( &pit_pid_speed_structure,\
												   bmi_structure.pit_gro,\
												   gimbal->info->target_pitch_speed);
		 motor_6020_PIT_structure.output_current = pit_pid_speed_structure.pos_out;
	}
	else
	{
		gimbal->pitch->output_current = pid_calc( &pit_pid_bmispeed_structure,\
												   bmi_structure.pit_gro,\
												   gimbal->info->target_pitch_speed);
		 motor_6020_PIT_structure.output_current = pit_pid_bmispeed_structure.pos_out;
	}


   

}



/*pitch电机 位置环控制（外环）  6020*/
void gimbal_pitch_angle_ctrl(gimbal_t *gimbal)
{
	//使用陀螺仪角度的有 跟随(小陀螺) 视觉
	if(gimbal->work->status !=G_G_vision && \
	   gimbal->work->status !=G_G_follow && \
	   gimbal->work->status !=G_G_spin)
	{

		gimbal->info->target_pitch_speed = pid_calc(&pit_pid_position_structure,\
													 gimbal->pitch->base_info->angle,\
													 gimbal->info->target_pitch_angle);
	}
	else
	{
		gimbal->info->target_pitch_speed = pid_calc( &pit_pid_bmiposition_structure,\
													 bmi_structure.pit_angle,\
													 gimbal->info->target_pitch_angle );
	}

    
    gimbal_pitch_speed_ctrl(gimbal);

}



/*yaw电机数据更新 6020*/
void gimbal_yaw_can_update(gimbal_t *gimbal)
{


    /*速度用陀螺仪 角度用自带的*/
    /*不管了先全都用返回来的数据，写出来先再改陀螺仪*/


   


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

    } else if(info->target_yaw_angle < 0)
    {
        info->target_yaw_angle += 8192 ;
    }

}



/*yaw电机 跟随位置环控制（外环）  6020*/
void gimbal_yaw_angle_ctrl(gimbal_t *gimbal)
{
    //小陀螺模式下不需要
    if(gimbal->yaw_work->status != G_Y_spin && switch_head == 0)
    {
        /*地盘跟头*/
        chassis_structure.info->target_cycle_speed= pid_calc_err(&yaw_follow_pid_position_structure,\
													gimbal->yaw->base_info->angle,\
													(gimbal->front_or_back == 1? YAW_MIDDLE_FOUNT:YAW_MIDDLE_BACK)\
													+ gimbal_structure.angle_45 *HALF_HALF_CIRLE);//换头 + 侧向45度
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
                                     gimbal->front_or_back == 1? YAW_MIDDLE_FOUNT:YAW_MIDDLE_BACK);//机械模式下的头也要锁定

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



