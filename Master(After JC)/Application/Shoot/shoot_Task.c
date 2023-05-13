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

/*�����ٴ���*/
extern uint8_t                       shoot_1_over_speed_flag;
extern uint8_t                       shoot_2_over_speed_flag;


float L_shoot_speed = 0;
float R_shoot_speed = 0;

#define AERIAL_SHOOT 1
#define AERIAL_STOP_SHOOT 0

uint8_t shoot_aerial_cmd = AERIAL_SHOOT;

void Shoot_Work(shoot_t* shoot)
{
	/*������ʼ�Ӿ�*/
	if(rc_structure.info->status == REMOTE_ONLINE )
	{
		if(judge.base_info->game_progress == 4) //��������ʼ��ǿ�ƽ��򵰿���Ȩ���Ӿ�
		{
			L_shoot_structure.status = Visin_Shoot;
			R_shoot_structure.status = Visin_Shoot;
			
		}
		else if(judge.base_info->game_progress == 5)//��������ǿ�ƹرշ������
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
	
	/*��̨�������*/
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
	/*����ϵͳ���ߴ���*/
	else
	{
		shoot_aerial_cmd = AERIAL_STOP_SHOOT;
	}
	/*�Ӿ����ٴ���*/
	
	L_shoot_speed = -((vision_structure.rx_pack->RxData.L_shoot_speed/10.0)/8.0)*60*36;
	R_shoot_speed = -((vision_structure.rx_pack->RxData.R_shoot_speed/10.0)/8.0)*60*36;
	
	
	/*ǹ����������*/
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
	
	
	/*���߼��*/
	MOTOR_2006_HEART(&L_motor_2006_PLUCK_structure);
	MOTOR_2006_HEART(&R_motor_2006_PLUCK_structure);
	
	/*��ת���*/
	Done_Check(&L_shoot_structure);
	Done_Check(&R_shoot_structure);
	
	
	/*�󲦵���*/
	/*����ģʽ*/
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
			/*��������*/
			Running_Fire(&L_shoot_structure);

			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*����ģʽ*/
	else if (L_shoot_structure.status == Single_Shoot)
	{
		

		if(L_shoot_structure.flag.locked == 0)
		{
			//Car.C�д���
			//L_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			
			/*�Ƕȿ���*/
			Single_Fire(&L_shoot_structure);
			
			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*�Ӿ�����*/
	else if(L_shoot_structure.status == Visin_Shoot )
	{

		
		if(L_shoot_structure.flag.locked == 0)
		{
			
			//Car.C�д���
			//L_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			
			#if 0
			if(L_shoot_structure.base.last_shoot_speed != L_shoot_structure.base.shoot_speed && \
			   L_shoot_structure.base.last_shoot_speed == 0)
			{
				/*Ϊ�˿��ٴ����һ��*/
				L_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
			}
			/*����һ���Ƿ���*/
			if(abs(L_shoot_structure.stir_wheel->base_info->angle_sum - L_shoot_structure.stir_wheel->base_info->target_angle_sum) <= 2000)
			{
				L_shoot_structure.flag.vision_first_shoot = 1;
			}
			
			/*�����ٶ�Ϊ��������־λ*/
			if(L_shoot_structure.base.shoot_speed == 0)
			{
				L_shoot_structure.flag.vision_first_shoot = 0;
			}
			
			/*����һ��������л�Ϊ�ٶȿ���*/
			if(L_shoot_structure.flag.vision_first_shoot == 1)//�����һ��
			{
				Running_Fire(&L_shoot_structure);
			}
			else
			{
				Single_Fire(&L_shoot_structure);
			}
			#endif
			
			#if 1
			/*����Ԥ����*/
			L_shoot_structure.base.last_shoot_speed = L_shoot_structure.base.shoot_speed;
			L_shoot_structure.base.shoot_speed = L_shoot_speed;
			
			Running_Fire(&L_shoot_structure);
			#endif
			
			/*��ת����*/
			L_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&L_shoot_structure);
		}
	}
	/*��������ر�ģʽ*/
	else
	{	
		/*ֹͣת��*/
		L_shoot_structure.base.shoot_speed = 0;
		Running_Fire(&L_shoot_structure);
		
		/*�ǶȺ�����*/
		L_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		L_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		/*��ת��־λ����*/
		L_shoot_structure.cnt.done_time = 0;
		L_shoot_structure.flag.locked = 0;
	}
		
		

	/*�Ҳ�����*/
	/*����ģʽ*/
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

			/*��������*/
			Running_Fire(&R_shoot_structure);

			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*����ģʽ*/
	else if (R_shoot_structure.status == Single_Shoot)
	{
		
		if(R_shoot_structure.flag.locked == 0)
		{
			//Car.C�д���
			//R_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			/*�Ƕȿ���*/
			Single_Fire(&R_shoot_structure);
			
			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*�Ӿ�ģʽ*/
	else if(R_shoot_structure.status == Visin_Shoot)
	{
		
		
		if(R_shoot_structure.flag.locked == 0)
		{
			//Car.C�д���
			//L_shoot_structure.stir_wheel->base_info->target_angle_sum += 36864;
			#if 0
			if(R_shoot_structure.base.last_shoot_speed != R_shoot_structure.base.shoot_speed && \
			   R_shoot_structure.base.last_shoot_speed == 0)
			{
				/*Ϊ�˿��ٴ����һ��*/
				R_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
			}
			/*����һ���Ƿ���*/
			if(abs(R_shoot_structure.stir_wheel->base_info->angle_sum - R_shoot_structure.stir_wheel->base_info->target_angle_sum) <= 2000)
			{
				R_shoot_structure.flag.vision_first_shoot = 1;
			}
			
			/*�����ٶ�Ϊ��������־λ*/
			if(R_shoot_structure.base.shoot_speed == 0)
			{
				R_shoot_structure.flag.vision_first_shoot = 0;
			}
			
			/*����һ��������л�Ϊ�ٶȿ���*/
			if(R_shoot_structure.flag.vision_first_shoot == 1)//�����һ��
			{
				Running_Fire(&R_shoot_structure);
			}
			else
			{
				Single_Fire(&R_shoot_structure);
			}
			#endif
				
			#if 1
			/*����Ԥ����*/
			R_shoot_structure.base.last_shoot_speed = R_shoot_structure.base.shoot_speed;
			R_shoot_structure.base.shoot_speed = R_shoot_speed;
			
			Running_Fire(&R_shoot_structure);
			#endif
			/*��ת����*/
			R_shoot_structure.cnt.deal_time = 0;
		}
		else
		{
			DONE_WORK(&R_shoot_structure);
		}
	}
	/*��������ر�ģʽ*/
	else
	{
		R_shoot_structure.base.shoot_speed = 0;
		Running_Fire(&R_shoot_structure);
		/*�ǶȺ�����*/
		R_shoot_structure.stir_wheel->base_info->target_angle_sum = 0;
		R_shoot_structure.stir_wheel->base_info->angle_sum = 0;
		/*��ת��־λ����*/
		R_shoot_structure.cnt.done_time = 0;
		R_shoot_structure.flag.locked = 0;
	}

	
	/*��ֹ��ת�����µ�����ȣ���ֹ���ζ�ת��������*/
	/*���ӳ�������������������������������������������*/
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
	/*���ӳ�������������������������������������������*/
	
	/*Ħ����״̬����*/
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
	/*������ݷ��ͣ�����������ߵ������㣬����һ��������һ��Ҳ�����䣩*/
	
	/*���߱���*/
	
	/*������g��*/
//	L_shoot_structure.stir_wheel->output_current = 0;
//	Master_Head_structure.Send_L_Head.shoot_mode = 0;
	/*������g��*/
	
	if( rc_structure.info->status == REMOTE_OFFLINE	)
	{
		int16_t L_current = L_shoot_structure.stir_wheel->output_current;
		int16_t R_current = R_shoot_structure.stir_wheel->output_current;
		
		if(L_shoot_structure.stir_wheel->info->status  == _DEV_OFFLINE)
		{
			/*״̬����*/
			L_current = 0;
			L_shoot_structure.base.status = Shoot_Offline;
		}
			
		if(R_shoot_structure.stir_wheel->info->status  == _DEV_OFFLINE)
		{
			/*״̬����*/
			R_current = 0;
			R_shoot_structure.base.status = Shoot_Offline;
		}
		
		/*ж��*/
		MOTOR_3508_CAN2_SENT_DATA(L_current,R_current,0,0);
			
	}
	/*��������*/
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
		/*���ݷ���*/
		MOTOR_3508_CAN2_SENT_DATA(L_shoot_structure.stir_wheel->output_current,\
								  R_shoot_structure.stir_wheel->output_current,\
								  0,\
								  0);
		/*״̬����*/
		L_shoot_structure.base.status = Shoot_Online;
		R_shoot_structure.base.status = Shoot_Online;
	}
	
	
	

}	































