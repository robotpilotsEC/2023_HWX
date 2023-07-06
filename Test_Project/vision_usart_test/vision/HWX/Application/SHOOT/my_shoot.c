
#include "my_shoot.h"
#include "car.h"
#include "dji_pid.h"
#include "REMOTE.h"
#include "drv_tim.h"
#include "judge_infantrypotocol.h"
#include "judge_sensor.h"

/*�������*/
extern motor_2006_t           motor_2006_PLUCK_structure;
extern motor_2006_base_info_t motor_2006_PLUCK_base_info;
extern motor_2006_info_t      motor_2006_PLUCK_info;

/*Ħ����*/
extern motor_3508_t           motor_3508_FIRC_L_structure;
extern motor_3508_base_info_t motor_3508_FIRC_L_base_info;
extern motor_3508_info_t      motor_3508_FIRC_L_info;

extern motor_3508_t           motor_3508_FIRC_R_structure;
extern motor_3508_base_info_t motor_3508_FIRC_R_base_info;
extern motor_3508_info_t      motor_3508_FIRC_R_info;

/*PID����*/
extern pid_t                  FIRC_L_speed_structure;
extern pid_t                  FIRC_R_speed_structure;
extern pid_t                  STRI_three_speed_structure;
extern pid_t                  STRI_position_structure;
extern pid_t                  STRI_one_speed_structure;
extern pid_t                  shoot_deal_done_structure;
extern pid_t                  shoot_judge_structure;
/*��״̬*/
extern car_t                  car_structure;

/*�������*/
extern shoot_t                shoot_structure; 
extern int8_t                 fric_work;

/*����ϵͳ*/
extern judge_sensor_t         judge_sensor;
/*ң����*/
extern rc_t                   rc_structure;
extern int8_t                 s2_down_flag;
extern int8_t                 s2_up_flag;
	   int8_t                 s2_up_last_flag = 1;
       int16_t                shoot_offline_cnt = 0;

       int8_t                 init_ok = 0;//����ģʽ�²������Ƿ񵽴�Ŀ��ת��
       int8_t	              deal_done = 0;//��ת���ڴ���
	   int8_t                 over_heat = 0;
extern int8_t           	  one_shoot_done;
extern bool                   hight_speed_shoot;

/**
  * @Name    Cover
  * @brief  
  * @param   ���տ��� 
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
  * @param   shoot: [����/��] 
  * @retval
  * @author  HWX
  * @Date    2022-10-20
**/
void SHOOT_INIT(shoot_t * shoot)
{
    /*����� ��ʼ��*/
    MOTOR_3508_INIT(&motor_3508_FIRC_L_structure, &motor_3508_FIRC_L_base_info,&motor_3508_FIRC_L_info );
    MOTOR_3508_INIT(&motor_3508_FIRC_R_structure, &motor_3508_FIRC_R_base_info,&motor_3508_FIRC_R_info );
    MOTOR_2006_INIT(&motor_2006_PLUCK_structure,  &motor_2006_PLUCK_base_info,&motor_2006_PLUCK_info );

    /*Ħ�����ٶȻ�*/
    PID_struct_init(&FIRC_L_speed_structure,POSITION_PID,PID_FIRC_OUTPUT_MAX,PID_FIRC_INTEGRA_MAX,PID_FIRC_KP,PID_FIRC_KI,PID_FIRC_KD);
    PID_struct_init(&FIRC_R_speed_structure,POSITION_PID,PID_FIRC_OUTPUT_MAX,PID_FIRC_INTEGRA_MAX,PID_FIRC_KP,PID_FIRC_KI,PID_FIRC_KD);

    /*����λ�û�*/
    PID_struct_init(&STRI_one_speed_structure,POSITION_PID,SPID_STRI_OUTPUT_MAX,SPID_STRI_INTEGRA_MAX,SPID_STRI_KP,SPID_STRI_KI,SPID_STRI_KD);
    PID_struct_init(&STRI_position_structure,POSITION_PID,PPID_STRI_OUTPUT_MAX,PPID_STRI_INTEGRA_MAX,PPID_STRI_KP,PPID_STRI_KI,PPID_STRI_KD);

    /*�����ٶȻ�*/
    PID_struct_init(&STRI_three_speed_structure,POSITION_PID,TPID_STRI_OUTPUT_MAX,TPID_STRI_INTEGRA_MAX,TPID_STRI_KP,TPID_STRI_KI,TPID_STRI_KD);
	
	/*��ת����λ�û�*/
	PID_struct_init(&shoot_deal_done_structure,POSITION_PID,PID_DONE_OUTPUT_MAX,PID_DONE_INTEGRA_MAX,PID_DONE_KP,PID_DONE_KI,PID_DONE_KD);
	
	/*����ϵͳ�ٶȻ�*/
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
  * @brief  ģʽ����
  * @param
  * @retval
  */

void MODE_UPDATE(shoot_t* shoot)
{

	if(car_structure.ctrl_mode == RC_CAR)
	{
		/*����ģʽƥ��*/
		switch (car_structure.move_mode_status)
		{
		case offline_CAR:
		{

			shoot->work_sta = S_S_offline;

			/*PIDĿ��ֵ����*/
			shoot->stir_wheel->base_info->target_speed = 0;
			shoot->shoot_speed = 0;
			shoot->stir_wheel->base_info->target_angle_sum = 0;
			/*��־λ����*/
			s2_up_flag      = 1;
			s2_down_flag    = 1;
			s2_up_last_flag = 1;
			shoot->stir_wheel->base_info->circle_cnt = 0;
			shoot->stir_wheel->base_info->angle_sum    = 0;

			/*й��*/
			shoot_offline_cnt++;
			
			/*���ն��ж��*/
			COVER_PwmOut(230);//������230û��

			break;

		}

		case machine_CAR:
		{
			shoot->work_sta = S_S_oneshot;

			if(!deal_done)//�����ڴ����ת����ת������ȫ��öԲ������ٶȵĲ���Ȩ
			{
				/*������ת��һ��*/
				if(s2_up_last_flag != s2_up_flag )
				{

					shoot->stir_wheel->base_info->target_angle_sum -= (int32_t)8191*4.5; //8191*4.5  36860

				}
			}

			s2_up_last_flag = s2_up_flag;

			/*Ħ���ֿ���*/
			if(s2_down_flag != 1  || shoot->box_open == 1)
			{
				Cover_Open();//��������
			}
			else
			{
				Cover_Close();//�رյ���
			}

			break;
		}

		case follow_CAR:
		{

			shoot->work_sta = S_S_threeshot;

			if(!deal_done)//�����ڴ����ת����ת������ȫ��öԲ������ٶȵĲ���Ȩ
			{
				/*�����ֿ���*/
				if(s2_up_flag != 1)
				{
					shoot->stir_wheel->base_info->target_speed = SHOOT_STRI_SPEED;//����������

					/*init_ok��־λ����*/
					if(shoot->stir_wheel->base_info->speed >= (SHOOT_STRI_SPEED - SHOOT_STRI_SPEED_ERR))
					{
						init_ok = 1;//һ�����ھ� = 1��ֱ�������ֹر�������
					}
				}
				else
				{
					shoot->stir_wheel->base_info->target_speed = 0;//�رղ�����
					init_ok = 0;
				}
			}

			/*Ħ���ֿ���*/
			if(s2_down_flag != 1  )
			{
				shoot->shoot_speed = SHOOT_STRI_SPEED;//����Ħ����

			}
			else
			{
				shoot->shoot_speed = 0;//�ر�Ħ����
			}
			
			/*������ģʽ�ص���*/
			Cover_Close();

			break;
		}
		}

		rc_structure.base_info->s2.value_last = rc_structure.base_info->s2.value;

		SHOOR_WORK(&shoot_structure);
	}
	/*����ģʽ���Ʒ������*/
	else if(car_structure.ctrl_mode == KEY_CAR)
	{
		
		/*����ģʽһֱ����Ħ����?*/
		if(car_structure.move_mode_status == machine_CAR)
		{
			shoot->shoot_speed = 0;//�ر�Ħ����
			shoot->stir_wheel->base_info->target_speed = 0;
			SHOOR_WORK(&shoot_structure);
		}
		else if(car_structure.move_mode_status == offline_CAR)
		{
			shoot->work_sta = S_S_offline;

			/*PIDĿ��ֵ����*/
			shoot->stir_wheel->base_info->target_speed = 0;
			shoot->shoot_speed = 0;
			shoot->stir_wheel->base_info->target_angle_sum = 0;
			shoot->stir_wheel->base_info->angle_sum = 0;
			/*��־λ����*/
			s2_up_flag      = 1;
			s2_down_flag    = 1;
			s2_up_last_flag = 1;
			shoot->stir_wheel->base_info->circle_cnt = 0;
			shoot->stir_wheel->base_info->angle      = 0;
			

			/*й��*/
			shoot_offline_cnt++;
			
			/*���ն��ж��*/
			COVER_PwmOut(230);//������230û��
		}
		else
		{
			//shoot->shoot_speed = SHOOT_SPEED *fric_work;//����Ħ����
			shoot->shoot_speed = SHOOT_STRI_SPEED *fric_work;//����Ħ����
			
			if(deal_done == 0)
			{
				
				if(shoot->stir_wheel->base_info->speed >= SHOOT_STRI_SPEED )
					{
							init_ok = 1;//�����ֳ�ʼ�����
					}
				/*�����ֿ���*/
				switch(shoot->stri_mode)
				{
					case ONE_SHOOT:
						//car.c����
					break;
					
					case NOT_SHOOT:
						
						shoot->stir_wheel->base_info->target_speed = 0;
						init_ok = 0;//���ò�����״̬
					
					break;
					
					case SIX_SHOOT:
						shoot->stir_wheel->base_info->target_speed = SHOOT_STRI_SPEED;
						
					
					break;
				
				}
				
				
				SHOOR_WORK(&shoot_structure);
			}
			/*��ת����*/
			else if(deal_done == 1)
			{
				DONE_WORK(&shoot_structure);
			}
		}
		
		
		
		/*���տ���*/
		if(shoot->box_open == 1)
		{
			Cover_Open();//��������
		}
		else
		{
			Cover_Close();//�رյ���
		}		
	}
	
}



/**
  * @brief  �����������
  * @param
  * @retval
  */
void SHOOR_WORK(shoot_t* shoot)
{

    //�ҿ�������ͼ��ȷʵ�ǻ���һһ����ͻ�䣬��֪����Ϊʲô�����Բ�ȡ�ǶȲ���Ǻ�һ��ģ�����������׼ȷ��
    if(shoot->work_sta == S_S_oneshot || shoot->stri_mode == ONE_SHOOT)
    {
        /*������λ�û�PID*/
        shoot->stir_wheel->base_info->target_speed = pid_calc(&STRI_position_structure,\
                shoot->stir_wheel->base_info->angle_sum,\
                shoot->stir_wheel->base_info->target_angle_sum);

        /*�������ٶȻ�PID*/ //ת��ô��Ȧ��ʲô���
        shoot->stir_wheel->output_current          = pid_calc(&STRI_one_speed_structure,\
                shoot->stir_wheel->base_info->speed,\
                shoot->stir_wheel->base_info->target_speed);

    }

    else if(shoot->work_sta == S_S_threeshot || shoot->stri_mode == SIX_SHOOT || shoot->stri_mode == NOT_SHOOT )
    {


        /*�������ٶȻ�PID*/
        shoot->stir_wheel->output_current = pid_calc(&STRI_three_speed_structure,\
                                            shoot->stir_wheel->base_info->speed,\
                                            shoot->stir_wheel->base_info->target_speed);
    }
	
	
		/*Ħ�����ٶȻ�PID*/
	shoot->fric_left->output_current  = pid_calc(&FIRC_L_speed_structure,\
										shoot->fric_left->base_info->speed,\
										(-1)*shoot->shoot_speed);


	shoot->fric_right->output_current = pid_calc(&FIRC_R_speed_structure,\
										shoot->fric_right->base_info->speed,\
										shoot->shoot_speed);
}


/**
  * @brief  ��ת���
  * @param  abs(shoot->stir_wheel->base_info->speed) <= SHOOT_DONE_SPEED_MAX
  * @retval init_ok
  */
/*
������Ҫ��⵽��ת����֤������ʱ�򲻻ᷢ����⣬������Ҫһ��shoot_init_ok��־λ��
����Ǵ�������С��Χ�Ķ��������Ƿ�ת���Ҿ��ÿ��Խ������ֵĿ���Ȩ��һ���µ�λ�û�pid�����������˶�
ÿ�˶�һ�����ڣ��ͼ���Ƿ��ת��������תӦ���Ƿ���������ȼ���ߵ�״̬


*/
void DONE_CHECK(shoot_t* shoot)//��ת�������㣿
{
	
	/*������ת*/
    if(shoot->work_sta != S_S_offline || shoot_structure.stri_mode == SIX_SHOOT)
    {
        if((init_ok && abs(shoot->stir_wheel->base_info->speed) <= SHOOT_DONE_SPEED_MAX))//��ʼ����� �Ҷ�ת
        {
            if(shoot->cnt_three_done >= SHOOT_DONE_TIME_MAX)//��תʱ��ﵽ�����ܵ����ֵ
            {
                deal_done = 1;//��ʾ���ڴ����ת��˼������жϽ����ת��
				//�����ת����λ�û� //shoot->stir_wheel->base_info->target_speed *= (-1);//�ٶ�ȡ��
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
				shoot->cnt_three_done  = 0;//����ģʽδ������ת
				deal_done              = 0;//������ת
				shoot->done_time       = 0;//��תʱ�����������
			}

        }
    }
	else if(shoot_structure.stri_mode == ONE_SHOOT)
	{
		if(abs(shoot->stir_wheel->base_info->angle_sum - shoot->stir_wheel->base_info->target_angle_sum) >=  5000)
		{
			
			if( shoot->six_done_time >= SIX_SHOOT_DONE_TIME_MAX)//��תʱ��ﵽ�����ܵ����ֵ
            {
                deal_done = 1;//��ʾ���ڴ����ת��˼������жϽ����ת��
				//�����ת����λ�û� //shoot->stir_wheel->base_info->target_speed *= (-1);//�ٶ�ȡ��
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
				shoot->six_done_time  = 0;//����ģʽδ������ת
				deal_done              = 0;//������ת
				shoot->done_time       = 0;//��תʱ�����������
			}

        }
	
	
	}
	
	
	/*Ħ�����ٶȻ�PID����ת��������˴���Ħ���֣�*/
	shoot->fric_left->output_current  = pid_calc(&FIRC_L_speed_structure,\
										shoot->fric_left->base_info->speed,\
										(-1)*shoot->shoot_speed);


	shoot->fric_right->output_current = pid_calc(&FIRC_R_speed_structure,\
										shoot->fric_right->base_info->speed,\
										shoot->shoot_speed);

}


/**
  * @brief  �������������
  * @param
  * @retval 
  */
void shoot_task()
{
    /*����ģʽ����*/
    MODE_UPDATE( &shoot_structure);//���������������

    /*�������ݸ���*/
    DATA_UPDATE(&shoot_structure);

	/*����ϵͳ���ݸ���*/
    DONE_CHECK(&shoot_structure);
	

    /*ж��*/
    if(shoot_offline_cnt >= 100)
    {
        shoot_offline_cnt = 100;

        MOTOR_3508_CAN2_SENT_DATA(0,0,0,0);//ע��˳�����һ��һ����0
    }
    else
    {
        /*������ݷ���*/
        MOTOR_3508_CAN2_SENT_DATA(motor_3508_FIRC_R_structure.output_current,\
                                  motor_3508_FIRC_L_structure.output_current,\
                                  motor_2006_PLUCK_structure.output_current,\
                                  0);//ע��˳�����һ��һ����0
    }
}


/**
  * @brief  ��ת����
  * @param
  * @retval Ӧ��Ҫһ����תʱ�������
  */
void DONE_WORK(shoot_t* shoot)
{
	if( ( (shoot->done_time++) %DEAL_DONE_ROUND_TIME)  >= DEAL_DONE_ROUND_HALF_TIME)
	{
		 /*�������ٶȻ�PID*/
        shoot->stir_wheel->output_current = pid_calc(&shoot_deal_done_structure,\
														shoot->stir_wheel->base_info->speed,\
														DEAL_DONE_SPEED);	//��ת
	}
	else
	{
		/*�������ٶȻ�PID*/
        shoot->stir_wheel->output_current = pid_calc(&shoot_deal_done_structure,\
														shoot->stir_wheel->base_info->speed,\
														(-1)*DEAL_DONE_SPEED);//��ת
	}
	
	if(shoot->done_time == DEAL_DONE_ROUND_TIME)
	{
		deal_done = 0;//������ת
		shoot->cnt_three_done  = 0;//����ģʽδ������ת
		shoot->cnt_three_done  = 0;//����ģʽδ������ת
		shoot->done_time       = 0;//��תʱ�����������
	}	
	
}




/**
  * @brief  ����ϵͳ���ݸ���
  * @param	hight_speed_shoot 1 ��ʾ����
  * @retval ���� ��ģʽ
  */
void DATA_UPDATE(shoot_t* shoot)
{
	
	   shoot->cooling_heat  = judge_sensor.info->power_heat_data.shooter_id1_17mm_cooling_heat;
	   shoot->cooling_limit = judge_sensor.info->game_robot_status.shooter_id1_17mm_cooling_limit;
		
		if(shoot->cooling_heat >= shoot->cooling_limit - 9)
		{
			 /*�������ٶȻ�PID*/
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
  * @brief  �����������
  * @param
  * @retval
  */
void SHE_JI(shoot_t* shoot)
{
	

}




