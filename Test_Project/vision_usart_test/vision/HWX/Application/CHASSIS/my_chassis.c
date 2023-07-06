#include "my_chassis.h"
#include "my_gimbal.h"
#include "remote.h"
#include "3508_motor.h"
#include "car.h"
#include "dji_pid.h"
#include "bmi.h"
#include "Power_Limit.h"

extern motor_3508_t           motor_3508_LF_structure;
extern motor_3508_base_info_t motor_3508_LF_base_info;
extern motor_3508_info_t      motor_3508_LF_info;

extern motor_3508_t           motor_3508_RF_structure;
extern motor_3508_base_info_t motor_3508_RF_base_info;
extern motor_3508_info_t      motor_3508_RF_info;

extern motor_3508_t           motor_3508_LB_structure;
extern motor_3508_base_info_t motor_3508_LB_base_info;
extern motor_3508_info_t      motor_3508_LB_info;

extern motor_3508_t           motor_3508_RB_structure;
extern motor_3508_base_info_t motor_3508_RB_base_info;
extern motor_3508_info_t      motor_3508_RB_info;

extern chassis_t              chassis_structure;
extern chassis_config_t       chassis_config_structure;
extern chassis_info_t         chassis_info_structure;

extern gimbal_t               gimbal_structure;

extern pid_t                  chassis_pid_speed_structure[4];
extern pid_t                  chassis_pid_position_structure;

extern rc_t                   rc_structure;

extern car_t                  car_structure;

extern bmi_t                  bmi_structure;

	   int16_t                limit_output_current[4];

extern int8_t                 switch_head;
extern float sin_x,cos_x,tese,bbb;
/**
  * @brief  ���̳�ʼ��
  * @param  
  * @retval 
  */
void chassis_init(chassis_t *chassis)
{
	/* ��ʼ���ĸ���� */
	MOTOR_3508_INIT( &motor_3508_LF_structure, &motor_3508_LF_base_info, &motor_3508_LF_info);
	MOTOR_3508_INIT( &motor_3508_RF_structure, &motor_3508_RF_base_info, &motor_3508_RF_info);
	MOTOR_3508_INIT( &motor_3508_LB_structure, &motor_3508_LB_base_info, &motor_3508_LB_info);
	MOTOR_3508_INIT( &motor_3508_RB_structure, &motor_3508_RB_base_info, &motor_3508_RB_info);
	
	chassis->motor_LF = &motor_3508_LF_structure;
	chassis->motor_RF = &motor_3508_RF_structure;
	chassis->motor_LB = &motor_3508_LB_structure;
	chassis->motor_RB = &motor_3508_RB_structure;
	chassis->config   = &chassis_config_structure;
	chassis->info     = &chassis_info_structure;
	chassis->config->chassis_output_max = 12000;
	chassis->config->chassis_speed_max  = 8000;
	
	/* ��ʼ��PID */
	for(uint8_t i = 0;i < 4 ; i++ )
	{
		PID_struct_init( &chassis_pid_speed_structure[i],POSITION_PID,SPEED_LIM_O,SPEED_LIM_I,SPEED_PID_P,SPEED_PID_I,0);
	}
	
	PID_struct_init( &chassis_pid_position_structure,POSITION_PID,POSITION_LIM_O,POSITION_LIM_I,POSITION_PID_P,POSITION_PID_I,POSITION_PID_D);
	
	
	/* ���������ʼ�� */
	chassis_commond_init(chassis);
	
	/* ������Ϣ��ʼ�� */
	chassis_info_init(chassis->info);
	
}
/**
  * @brief  ���������ʼ��
  * @param  
  * @retval 
  */
void chassis_commond_init(chassis_t *chassis)
{

}
/**
  * @brief  ������Ϣ��ʼ��
  * @param  
  * @retval 
  */
void chassis_info_init(chassis_info_t *info)
{
	info->target_front_speed = 0;
	info->target_right_speed = 0;
	info->target_cycle_speed = 0;
	info->move_mode          = C_M_offline;
	info->cycle_mode         = C_C_offline;
	info->back_cnt           = 0;
	info->speed_cnt          = 0;
}


/**
  * @brief  ���̿�������
  * @param  
  * @retval 
  */
void chassis_ctrl_task(chassis_t *chassis)
{
	/* ����ģʽ���� */
  chassis_mode_update(chassis);
	
	/* ���̵�ǰ�ٶȸ��� */
	chassis_measure_speed_update(chassis);
	
	/* ����������Ӧ */
//  chassis_commond_respond(chassis);
	
	/* ���̹��� */
  chassis_work(chassis);
	
	/* ���̲���ϵͳ���� */
//	chassis_judge_limit(chassis);
	
	/* �����ʼ�� */
	chassis_commond_init(chassis);
}


/**
  * @brief  ����ģʽ����
  * @param  
  * @retval 
  */
void chassis_mode_update(chassis_t *chassis)
{
  switch (car_structure.move_mode_status)
  {
	  
    case offline_CAR://����ģʽ 
      chassis->info->move_mode = C_M_offline;
      chassis->info->cycle_mode = C_C_offline;
      break;
	
	case vision_CAR:
    case machine_CAR://����ģʽ 
      chassis->info->back_cnt = 0;
      chassis->info->move_mode = C_M_normal;//�����ܻ���C_C_Lock
      chassis->info->cycle_mode = C_C_lock;
      break;
	

	case follow_CAR://����ģʽ
      chassis->info->back_cnt = 0;
      chassis->info->move_mode = C_M_follow;
      chassis->info->cycle_mode = C_C_follow;
      break;
	
	 case spin_CAR://С����
      chassis->info->back_cnt = 0;
      chassis->info->move_mode = C_M_spin;
      chassis->info->cycle_mode = C_C_spin;
      break;
		
    default:
      break;
  }

}
/**
  * @brief  ���̵�ǰ�ٶȸ���
  * @param  
  * @retval 
  */
void chassis_measure_speed_update(chassis_t *chassis)
{
	int16_t motor_speed_LF = chassis->motor_LF->base_info->speed;
	int16_t motor_speed_RF = chassis->motor_RF->base_info->speed;
	int16_t motor_speed_LB = chassis->motor_LB->base_info->speed;
	int16_t motor_speed_RB = chassis->motor_RB->base_info->speed;
	
	int16_t front_speed = chassis->info->measure_front_speed;
	int16_t right_speed = chassis->info->measure_right_speed;
	
	chassis->info->measure_front_speed = (motor_speed_LF - motor_speed_RF + motor_speed_LB - motor_speed_RB) / 4;
	chassis->info->measure_right_speed = (motor_speed_LF + motor_speed_RF - motor_speed_LB - motor_speed_RB) / 4;
	chassis->info->measure_cycle_speed = (motor_speed_LF + motor_speed_RF + motor_speed_LB + motor_speed_RB) / 4;
	
	chassis->info->measure_front_speed_dif = chassis->info->measure_front_speed - front_speed;
	chassis->info->measure_right_speed_dif = chassis->info->measure_right_speed - right_speed;
	
}


/**
  * @brief  ���̹���
  * @param  
  * @retval 
  */
void chassis_work(chassis_t *chassis)
{
	chassis_config_t *config = chassis->config;
	int8_t change_head = gimbal_structure.front_or_back;//ͷβ����λ
	float tem_front = 0;
	float tem_right = 0;
	float tem_round = 0;
	
  switch(chassis->info->move_mode)
  {	
		
    case C_M_offline:
      chassis->info->target_front_speed = 0;
      chassis->info->target_right_speed = 0;
			//PID����ȫ������
      break;
		case C_M_spin:
			 switch(car_structure.ctrl_mode)
			{
				case RC_CAR:
					chassis->info->target_front_speed = (float)rc_structure.base_info->ch3 * (float)config->chassis_speed_max / 660.f;
					chassis->info->target_right_speed = (float)rc_structure.base_info->ch2 * (float)config->chassis_speed_max / 660.f;
				break;
				
				case KEY_CAR:
					tem_front += (float)rc_structure.base_info->W.cnt / (float)KEY_W_CNT_MAX * (float)config->chassis_speed_max;
					tem_front -= (float)rc_structure.base_info->S.cnt / (float)KEY_S_CNT_MAX * (float)config->chassis_speed_max;
					tem_right += (float)rc_structure.base_info->D.cnt / (float)KEY_D_CNT_MAX * (float)config->chassis_speed_max;
					tem_right -= (float)rc_structure.base_info->A.cnt / (float)KEY_A_CNT_MAX * (float)config->chassis_speed_max;
				
					chassis->info->target_front_speed = tem_front;
					chassis->info->target_right_speed = tem_right;
				break;
			}
      break;
		case C_M_follow:
			//Ŀ����ת�ٶ���gimbal�и���
			switch(car_structure.ctrl_mode)
			{
				case RC_CAR:
					chassis->info->target_front_speed = (float)rc_structure.base_info->ch3 * (float)config->chassis_speed_max / 660.f *change_head;
					chassis->info->target_right_speed = (float)rc_structure.base_info->ch2 * (float)config->chassis_speed_max / 660.f *change_head;
				break;
				case KEY_CAR:
					tem_front += (float)rc_structure.base_info->W.cnt / (float)KEY_W_CNT_MAX * (float)config->chassis_speed_max;
					tem_front -= (float)rc_structure.base_info->S.cnt / (float)KEY_S_CNT_MAX * (float)config->chassis_speed_max;
					tem_right += (float)rc_structure.base_info->D.cnt / (float)KEY_D_CNT_MAX * (float)config->chassis_speed_max;
					tem_right -= (float)rc_structure.base_info->A.cnt / (float)KEY_A_CNT_MAX * (float)config->chassis_speed_max;
				
					chassis->info->target_front_speed = tem_front;
					chassis->info->target_right_speed = tem_right;
				break;
				
			}
		
    case C_M_normal:
      switch(car_structure.ctrl_mode)
      {
				
				
        case RC_CAR:
          chassis->info->target_front_speed = (float)rc_structure.base_info->ch3 * (float)config->chassis_speed_max / 660.f *change_head;
          chassis->info->target_right_speed = (float)rc_structure.base_info->ch2 * (float)config->chassis_speed_max / 660.f *change_head;
			break;

		case KEY_CAR:
			tem_front += (float)rc_structure.base_info->W.cnt / (float)KEY_W_CNT_MAX * (float)config->chassis_speed_max;
			tem_front -= (float)rc_structure.base_info->S.cnt / (float)KEY_S_CNT_MAX * (float)config->chassis_speed_max;
			tem_right += (float)rc_structure.base_info->D.cnt / (float)KEY_D_CNT_MAX * (float)config->chassis_speed_max;
			tem_right -= (float)rc_structure.base_info->A.cnt / (float)KEY_A_CNT_MAX * (float)config->chassis_speed_max;
		
			chassis->info->target_front_speed = tem_front *change_head;
			chassis->info->target_right_speed = tem_right *change_head;
		
		
			break;

        default:
          break;
      }
      break;
		}
	
		
	switch(chassis->info->cycle_mode)
	{
		
		case C_C_offline:
			chassis->info->target_cycle_speed = 0;
			break;
		case C_C_lock:
			switch(car_structure.ctrl_mode)
			{
				case RC_CAR:
					chassis->info->target_cycle_speed = (float)rc_structure.base_info->ch0 / 660.f * config->chassis_speed_max;
				break;
				
				case KEY_CAR://��еģʽ��ת�ٶ�С
					tem_round -= (float)rc_structure.base_info->Q.cnt / (float)KEY_Q_CNT_MAX * (float)config->chassis_speed_max;
					tem_round += (float)rc_structure.base_info->E.cnt / (float)KEY_E_CNT_MAX * (float)config->chassis_speed_max;
				
					chassis->info->target_cycle_speed = tem_round / 5;//��ת�ٶ�С������������ 
				break;

				
			}
		
			break;
		case C_C_spin:
			//chassis->info->target_cycle_speed = 2000;//��ת�ٶȿ��Ը���
			//��ת�ٶ���car.c�и���
			
			break;
		
		case C_C_normal://���������ת������̨����
//			switch(car_structure.ctrl_mode)
//			{
//				case RC_CAR:
//					chassis->info->target_cycle_speed = (float)rc_structure.base_info->ch0 / 660.f * config->chassis_speed_max;
//				break;

//				case KEY_CAR://����ģʽ��ת�ٶȴ�
//					tem_round += (float)rc_structure.base_info->Q.cnt / (float)KEY_Q_CNT_MAX * (float)config->chassis_speed_max;
//					tem_round -= (float)rc_structure.base_info->E.cnt / (float)KEY_E_CNT_MAX * (float)config->chassis_speed_max;
//				
//					chassis->info->measure_cycle_speed = tem_round;
//				break;
//			}
		
			break;
			
		default://���������
			break;
	}
	
	/* ���̵������from���� */
	motor_chassis_update(chassis);
	
	
	
	limit_output_current[0] = motor_3508_LF_structure.output_current;
	limit_output_current[1] = motor_3508_RF_structure.output_current;
	limit_output_current[2] = motor_3508_LB_structure.output_current;
	limit_output_current[3] = motor_3508_RB_structure.output_current;
	
	if(rc_structure.base_info->Shift.value != 1)
	{
		/*�����㷨*/
		Chassis_Motor_Power_Limit(limit_output_current);
	}
	
	/*���͵���*/
	MOTOR_3508_CAN1_SENT_DATA(limit_output_current[0],\
							  limit_output_current[1],\
							  limit_output_current[2],\
							  limit_output_current[3]);
	
	
	
	
}





/**
  * @brief  ���̵�����´ӵ���
  * @param  
  * @retval 
  */
void motor_chassis_update(chassis_t *chassis)
{
	int16_t front, right, round;
	
	/* ���̵���ٶ����� */
	if(car_structure.move_mode_status == spin_CAR || car_structure.move_mode_status == follow_CAR)
	{
		spin_work();///�ᵼ�¸���ģʽ�µĻ�ͷ�ٶȶ�ʧ
	}
	
	#if 0
	
	int16_t front_err;
	int16_t right_err;
	int16_t cycle_err;
	
	front_err = sgn(chassis->info->target_front_speed - chassis->info->measure_front_speed);
	right_err = sgn(chassis->info->target_right_speed - chassis->info->measure_right_speed);
	cycle_err = sgn(chassis->info->target_cycle_speed - chassis->info->measure_cycle_speed);
	
	chassis->motor_LF->base_info->target_prm   = (  front + right + round) *(float)( front_err + right_err + cycle_err) * 0.1;
	chassis->motor_RF->base_info->target_prm   = (- front + right + round) *(float)(-front_err + right_err + cycle_err) * 0.1;
	chassis->motor_LB->base_info->target_prm   = (  front - right + round) *(float)( front_err - right_err + cycle_err) * 0.1;
	chassis->motor_RB->base_info->target_prm   = (- front - right + round) *(float)(-front_err - right_err + cycle_err) * 0.1;
	#endif
	motor_speed_limit(chassis, &front, &right, &round);
	
	chassis->motor_LF->base_info->target_prm   = (  front + right + round);
	chassis->motor_RF->base_info->target_prm   = (- front + right + round);
	chassis->motor_LB->base_info->target_prm   = (  front - right + round);
	chassis->motor_RB->base_info->target_prm   = (- front - right + round);
	//�������Ŀ���ٶȣ�pid��������������ٶȻ�ȥ����
}

/**
  * @brief  ���̵���ٶ�����
  * @param  
  * @retval 
  */
void motor_speed_limit(chassis_t *chassis, int16_t *front, int16_t *right, int16_t *round)
{
	int16_t speed_sum;
	float RATE;
	speed_sum = abs(chassis->info->target_front_speed) + \
	            abs(chassis->info->target_right_speed) + \
              abs(chassis->info->target_cycle_speed);
	
	if(speed_sum > chassis->config->chassis_speed_max)
	{
		RATE = (float)chassis->config->chassis_speed_max / (float)speed_sum;
	}
	else 
	{
		RATE = 1;
	}
	
	*front = chassis->info->target_front_speed * RATE;
	*right = chassis->info->target_right_speed * RATE;
	*round = chassis->info->target_cycle_speed * RATE;
	chassis->info->target_front_speed = (float)chassis->info->target_front_speed * RATE;
	chassis->info->target_right_speed = (float)chassis->info->target_right_speed * RATE;
	chassis->info->target_cycle_speed = (float)chassis->info->target_cycle_speed * RATE;
}


/**
  * @brief  С����ģʽ�����ٶȽ���
  * @param  
  * @retval 
  */

void spin_work()
{		
		
	  int16_t front_speed = chassis_structure.info->target_front_speed,\
	          right_speed = chassis_structure.info->target_right_speed;
		
	  if(car_structure.move_mode_status == spin_CAR)
	  {
		bbb= (float)( 2100 - gimbal_structure.yaw->base_info->angle);
	  }
	  else
	  {
		if(gimbal_structure.front_or_back == 1)
		{
			bbb= (float)( YAW_MIDDLE_FOUNT - gimbal_structure.yaw->base_info->angle);
		}
		else
		{
			bbb= (float)( YAW_MIDDLE_BACK - gimbal_structure.yaw->base_info->angle);
		}
	  }
		
	  tese = (bbb/8191)*2*PI;
		
	  sin_x = (float)sin((double)(tese));
	  cos_x = (float)cos((double)(tese));
	
	 chassis_structure.info->target_front_speed =  (int16_t)(cos_x*(float)front_speed)-\
														    (float)(sin_x*right_speed);
	
	 chassis_structure.info->target_right_speed =  (int16_t)(sin_x*(float)front_speed)+\
													        (float)(cos_x*right_speed);


}




