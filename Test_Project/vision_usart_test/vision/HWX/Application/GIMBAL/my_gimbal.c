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
  * @brief  ��̨��ʼ��
  * @param
  * @retval
  */
void gimbal_init (gimbal_t *gimbal)
{
    /* �����ʼ�� */
    MOTOR_6020_INIT( &motor_6020_YAW_structure, &motor_6020_YAW_base_info, &motor_6020_YAW_info );
    MOTOR_6020_INIT( &motor_6020_PIT_structure, &motor_6020_PIT_base_info, &motor_6020_PIT_info );


    /* PID��ʼ�� */

    /*��еģʽYAWλ�û�*/
    PID_struct_init( &yaw_lock_pid_position_structure,  POSITION_PID,20000,    0,  10,    0, 0.5);
    /*��еģʽYAW�ٶȻ�*/
    PID_struct_init( &yaw_lock_pid_speed_structure,     POSITION_PID,20000, 5000,  20, 0.01,   0);


    /*����ģʽYAWλ�û������̸�ͷ��*/
    PID_struct_init( &yaw_follow_pid_position_structure,POSITION_PID,20000,    0,   3.5,   0,   0);
    /*����ģʽYAW�ٶȻ������ٶȻ���*/
    PID_struct_init( &yaw_pid_speed_structure,          POSITION_PID,20000, 2000,   10,  0.1,   0);
    /*����ģʽYAWλ�û�����̨ת����*/
    PID_struct_init( &yaw_head_pid_position_structure,  POSITION_PID,20000,    0,    7,    0,   0);


    /*PITCH��˫��*/
    PID_struct_init( &pit_pid_speed_structure,          POSITION_PID,20000, 5000,  15,    0,   0);
    PID_struct_init( &pit_pid_position_structure,       POSITION_PID,20000,    0,  15,    0, 0.1);
	
	/*PITCH��˫��(follow)*/
    PID_struct_init( &pit_pid_bmispeed_structure,          POSITION_PID,20000, 5000,  9,    0,   0);
    PID_struct_init( &pit_pid_bmiposition_structure,       POSITION_PID,20000,    0,  10,    0, 0.1);
	
	

    /*��ͷ��־λ*/
    gimbal->front_or_back = 1;

    /* ��̨��Ϣ��ʼ�� */
    gimbal_info_init(gimbal->info);

    /* ��̨�����ʼ�� */
    gimbal_commond_init(gimbal);


}
/**
  * @brief  ��̨��Ϣ��ʼ��
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
  * @brief  ��̨�����ʼ��
  * @param
  * @retval
  */
void gimbal_commond_init(gimbal_t *gimbal)
{

}


/**
  * @brief  ��̨��������
  * @param
  * @retval
  */void gimbal_ctrl_task(gimbal_t *gimbal)
{
    /* ��̨ģʽ���� */
    gimbal_mode_update(gimbal);

    /*��̨�����Ǹ�ֵ*/
    gimbal_yaw_can_update(gimbal);

    /* ��̨������Ӧ */
    gimbal_commond_respond(gimbal);

    /* ��̨���� */
    gimbal_work(gimbal);

    /* ��̨�����ʼ�� */
    gimbal_commond_init(gimbal);

    /*��̨������ݷ���*/
}

/**
  * @brief  ��̨ģʽ����
  * @param
  * @retval
  */
void gimbal_mode_update(gimbal_t *gimbal)
{

    if(car_structure.move_mode_status ==machine_CAR)
    {
        gimbal->work->status = G_G_lock;//��е
    } 
	else if(car_structure.move_mode_status ==follow_CAR)
    {
        gimbal->work->status = G_G_follow;//����
    } 
	else if(car_structure.move_mode_status == offline_CAR)
    {
        gimbal->work->status = G_G_offline;//ж��
    } 
	else if(car_structure.move_mode_status == spin_CAR)
    {
        gimbal->work->status = G_G_spin;//С����
    }
	else if(car_structure.move_mode_status == vision_CAR)
	{
		gimbal->work->status = G_G_vision;
	}

}

/**
  * @brief  ��̨������Ӧ
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
			info->target_pitch_angle = bmi_structure.pit_angle;//��֤���Ҷ�
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
			info->target_pitch_angle = gimbal->pitch->base_info->angle;//ֱ��Ŀ��ֵ���ڻ�е�ǶȾͺ�
		}
		PIT_first2follow = 1;
		
		if(PIT_vision2lock == 1 )
		{
			info->target_pitch_angle = gimbal->pitch->base_info->angle;//ֱ��Ŀ��ֵ���ڻ�е�ǶȾͺ�
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
		//vision_task();//������
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
		/*ֻ�н�����ģʽ��Ҫͷβ��ͷβ����� car.c�����*/
        /*����ģʽ�л�ʱ��ת����*/
        if(YAW_first2follow) {
            gimbal->info->target_yaw_angle = bmi_structure.yaw_angle;//�ӻ�е�и����ʱ��ͷ�����Ҷ�
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

    case G_Y_lock://��ס���ü���
        gimbal->info->target_yaw_angle = YAW_MIDDLE_FOUNT;
        YAW_first2follow = 1;
		YAW_lock2spin    = 1;
        gimbal_yaw_angle_check(gimbal);
        gimbal_yaw_angle_lock_ctrl(gimbal);
        break;

    case G_Y_spin://switch_head_cnt++ >= SWITCH_HEAD_TIME_MAX  #define SWITCH_HEAD_TIME_MAX  2000
		if(YAW_lock2spin) {
            gimbal->info->target_yaw_angle = bmi_structure.yaw_angle;//�ӻ�е�и����ʱ��ͷ�����Ҷ�
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
	
	case G_Y_vision://��ס���ü���
		info->target_yaw_angle = vision_structure.rx_pack->RxData.yaw_angle;
		gimbal_yaw_angle_check(gimbal);
        gimbal_yaw_angle_ctrl(gimbal);
		break;

    }
}

void gimbal_watch_work(gimbal_t *gimbal)
{

}



/*pitch������ݸ��� 6020*/
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

/*pitch��� �Ƕ����� 6020*/
void gimbal_pitch_angle_check(gimbal_t *gimbal)
{
    gimbal_info_t *info = gimbal->info;

	if(gimbal->work->status ==G_G_vision )
	{	
		if(info->target_pitch_angle > 8191)//Ŀ���ж�
		{
			info->target_pitch_angle -= 8192 ;

		} else if(info->target_pitch_angle < 0)
		{
			info->target_pitch_angle += 8192 ;
		}
	}
	else if( gimbal->work->status ==G_G_follow || gimbal->work->status ==G_G_spin )//���С���ݵ�ͷ����
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





/*pitch��� �ٶȻ����ƣ��ڻ���  6020*/
void gimbal_pitch_speed_ctrl(gimbal_t *gimbal)
{	
	//��ʵֻ���ò�ͬ��PID
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



/*pitch��� λ�û����ƣ��⻷��  6020*/
void gimbal_pitch_angle_ctrl(gimbal_t *gimbal)
{
	//ʹ�������ǽǶȵ��� ����(С����) �Ӿ�
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



/*yaw������ݸ��� 6020*/
void gimbal_yaw_can_update(gimbal_t *gimbal)
{


    /*�ٶ��������� �Ƕ����Դ���*/
    /*��������ȫ���÷����������ݣ�д�������ٸ�������*/


   


    /*����������*/
    gimbal->info->yaw_angle = bmi_structure.yaw_angle;

    /*
     *��еģʽ��ֱ���õ�����ص�����
     *����ģʽ�������ǵ�ŷ����*/
    gimbal_yaw_angle_check(gimbal);



}

/*yaw��� �Ƕ��޷�*/
void gimbal_yaw_angle_check(gimbal_t *gimbal)
{
    gimbal_info_t *info = gimbal->info;

    if(info->target_yaw_angle > 8191)//Ŀ���ж�
    {
        info->target_yaw_angle -= 8192 ;

    } else if(info->target_yaw_angle < 0)
    {
        info->target_yaw_angle += 8192 ;
    }

}



/*yaw��� ����λ�û����ƣ��⻷��  6020*/
void gimbal_yaw_angle_ctrl(gimbal_t *gimbal)
{
    //С����ģʽ�²���Ҫ
    if(gimbal->yaw_work->status != G_Y_spin && switch_head == 0)
    {
        /*���̸�ͷ*/
        chassis_structure.info->target_cycle_speed= pid_calc_err(&yaw_follow_pid_position_structure,\
													gimbal->yaw->base_info->angle,\
													(gimbal->front_or_back == 1? YAW_MIDDLE_FOUNT:YAW_MIDDLE_BACK)\
													+ gimbal_structure.angle_45 *HALF_HALF_CIRLE);//��ͷ + ����45��
    }

    /*��ͷ*//*��������ͼ���������*/
    gimbal->info->target_yaw_speed = pid_calc_err(&yaw_head_pid_position_structure,\
                                     bmi_structure.yaw_angle,\
                                     gimbal->info->target_yaw_angle);
    //λ�û�������������
    gimbal_yaw_speed_ctrl(gimbal);
}



/*yaw�����е λ�û����ƣ��⻷��  6020*/
void gimbal_yaw_angle_lock_ctrl(gimbal_t *gimbal)
{

    /*��еģʽ*/
    gimbal->info->target_yaw_speed = pid_calc_err(&yaw_lock_pid_position_structure,\
                                     gimbal->yaw->base_info->angle,\
                                     gimbal->front_or_back == 1? YAW_MIDDLE_FOUNT:YAW_MIDDLE_BACK);//��еģʽ�µ�ͷҲҪ����

    gimbal_yaw_lock_speed_ctrl(gimbal);
}




/*yaw��� �����ٶȻ����ƣ��ڻ���  6020*/
void gimbal_yaw_speed_ctrl(gimbal_t *gimbal)
{

    gimbal->yaw->output_current = pid_calc( &yaw_pid_speed_structure,\
                                            bmi_structure.yaw_gro,\
                                            gimbal->info->target_yaw_speed);

    motor_6020_YAW_structure.output_current = yaw_pid_speed_structure.pos_out;
}





/*yaw�����е �ٶȻ����ƣ��ڻ���  6020*/
void gimbal_yaw_lock_speed_ctrl(gimbal_t *gimbal)
{

	gimbal->yaw->output_current = pid_calc( &yaw_lock_pid_speed_structure,\
                                            bmi_structure.yaw_gro,\
                                            gimbal->info->target_yaw_speed);

    motor_6020_YAW_structure.output_current = yaw_lock_pid_speed_structure.pos_out;
}













/*��ʱû��*/
void gimbal_watch_speed_ctrl(gimbal_t *gimbal)
{

}
void gimbal_watch_angle_ctrl(gimbal_t *gimbal)
{


}



