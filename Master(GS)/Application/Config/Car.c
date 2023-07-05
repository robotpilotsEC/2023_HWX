#include "Car.h"
#include "remote.h"
#include "rp_chassis.h"
#include "rp_gimbal.h"
#include "can_protocol.h"
#include "rp_shoot.h"
#include "vision.h"
#include "rp_shoot.h"
#include "judge.h"
#include "FreeRTOS.h"
#include "can_protocol.h"

extern judge_t                judge;
extern car_t                  car_structure;
extern rc_t                   rc_structure;
extern chassis_t              Chassis; 
extern gimbal_t               Gimbal;
extern shoot_t                L_shoot_structure;
extern shoot_t                R_shoot_structure;
extern Master_Head_t          Master_Head_structure;
extern vision_t               vision_structure;



/*ң�������������*/
int8_t            s2_down_flag   = 1;
int8_t            s2_up_flag     = 1;
extern uint8_t shoot_1_over_speed_flag;
extern uint8_t shoot_2_over_speed_flag;
extern int16_t    test_shoot_speed;
void MODE_INIT(void)
{
	car_structure.ctrl = RC_CAR;
	car_structure.mode  = offline_CAR;

}


void MODE_CHECK(void)
{
    /*״̬����*/
    rc_wheel_status_interrupt_update(rc_structure.base_info);
	all_key_board_status_interrupt_update(rc_structure.base_info);
	all_key_board_status_update(rc_structure.base_info);
	
	/*�������(������²���)*/
	/*����Ҳ�������*/
	if(rc_structure.base_info->s2.value_last != rc_structure.base_info->s2.value \
			&& rc_structure.base_info->s2.value == 2)
	{
		s2_down_flag = s2_down_flag*(-1);
	}

	/*����Ҳ�������*/
	if(rc_structure.base_info->s2.value_last != rc_structure.base_info->s2.value \
			&& rc_structure.base_info->s2.value == 1)
	{
		s2_up_flag = s2_up_flag*(-1);
	}
	
	if(rc_structure.info->status == REMOTE_ONLINE)
	{
		/*ģ��״̬����*/

		
		/*����ӳ��*/
		if(S1_DOWN)//�����˶�����
		{

			/*���������״̬����*/
			L_shoot_structure.status = Stop_Shoot;
			R_shoot_structure.status = Stop_Shoot;
			
			
			if(S2_UP)
			{
				/*С����*/
				car_structure.mode = spin_CAR;
			}
		   else if(S2_DOWN)
			{
				/*��������ͷ*/
				car_structure.mode = machine_CAR;
				
				if(rc_structure.base_info->thumbwheel.status == down_R)
				{
						car_structure.mode = offline_CAR;
					
						Motor_9015_Off();
						Motor_3508_Can1SentData(0,0,0,0);
					
						HAL_Delay(20);
					
						__set_FAULTMASK(1);
						NVIC_SystemReset();
				}
				else if(rc_structure.base_info->thumbwheel.status == up_R)
				{
						car_structure.mode = offline_CAR;
					
						Motor_9015_Off();
						Motor_3508_Can1SentData(0,0,0,0);
					
						HAL_Delay(20);
					
						__set_FAULTMASK(1);
						NVIC_SystemReset();			
				}
			}
			else if(S2_MIDDLE)
			{	
				/*����д��Ĭ�Ͻ������ģʽ��*/
				static car_move_mode_e s2_down_mode  = follow_CAR;
				car_structure.mode = s2_down_mode;
				
				if(rc_structure.base_info->thumbwheel.value == -660)
				{
					/*����ģʽ*/
					s2_down_mode = follow_CAR;
				}
				else if(rc_structure.base_info->thumbwheel.value == 660)
				{
					/*����ģʽ*/
					s2_down_mode = navigation_CAR;				
				}
			}

		}
		else if(S1_MIDDLE)//��̨����ģʽ
		{
			
			if(S2_UP)
			{
				/*Ѳ��ģʽ*/
				//car_structure.mode = patrol_CAR;
				car_structure.mode = shake_CAR;
				/*���������״̬����*/
				L_shoot_structure.status = Stop_Shoot;
				R_shoot_structure.status = Stop_Shoot;

			}
			else if(S2_MIDDLE)
			{
				/*����˫ͷģʽ*/
				car_structure.mode = two_CAR;
				
				if(rc_structure.base_info->thumbwheel.value == 660)
				{
					L_shoot_structure.status = Running_Shoot;
					R_shoot_structure.status = Running_Shoot;
				}
				
				if(rc_structure.base_info->thumbwheel.value == -660)
				{
					L_shoot_structure.status = Stop_Shoot;
					R_shoot_structure.status = Stop_Shoot;
				}


			}
			else if(S2_DOWN)
			{
				/*����ģʽ*/
				car_structure.mode = aim_CAR;
				
				/*���������״̬����*/
				L_shoot_structure.status = Stop_Shoot;
				R_shoot_structure.status = Stop_Shoot;
			}

			

		}
		else if(S1_UP)//�ϲ���
		{

				
			if(S2_MIDDLE)
			{	
				/*�Ӿ�ģʽ*/
				if(vision_structure.rx_pack->FrameHeader.cmd_id == 1)
				{
					car_structure.mode = vision_CAR;
				}
				else if(vision_structure.rx_pack->FrameHeader.cmd_id == 2)
				{
					car_structure.mode = shake_CAR;
				}
				else
				{
					car_structure.mode = patrol_CAR;
				}

				L_shoot_structure.status = Stop_Shoot;
				R_shoot_structure.status = Stop_Shoot;
				
				/*��vision.c�и���*/
				/*���Ӿ�ģʽ�£������Ӿ����ı�־λѡ���������ǵ���*/
				/*�����쳣�����޳�*/
//				if(L_shoot_structure.cnt.shoot_cnt != L_shoot_structure.cnt.last_shoot_cnt)
//				{
//					uint8_t L_err =abs(L_shoot_structure.cnt.last_shoot_cnt - L_shoot_structure.cnt.shoot_cnt);
//					if(L_err >= 5)
//					{
//						L_err = 0;
//					}
//					
//					L_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864*(L_err);
//				}
//				if(R_shoot_structure.cnt.shoot_cnt != R_shoot_structure.cnt.last_shoot_cnt)
//				{
//					uint8_t R_err =abs(L_shoot_structure.cnt.last_shoot_cnt - L_shoot_structure.cnt.shoot_cnt);
//					if(R_err >= 5)
//					{
//						R_err = 0;
//					}
//					
//					R_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864*((R_err));
//				}
				

			}
			else if(S2_UP)
			{
				/*�Ӿ�ģʽ*/
				
				car_structure.mode = vision_CAR;
				
				if(judge.base_info->game_progress == 4)
				{
					L_shoot_structure.status = Visin_Shoot;
					R_shoot_structure.status = Visin_Shoot;
					/*��������ʼ��ǿ�ƽ���*/
					if(vision_structure.rx_pack->FrameHeader.cmd_id == 1)
					{
						car_structure.mode = vision_CAR;
					}
					else if(vision_structure.rx_pack->FrameHeader.cmd_id == 2)
					{
						car_structure.mode = shake_CAR;
					}
					else
					{
						car_structure.mode = patrol_CAR;
					}
					
				} 
				else
				{
					car_structure.mode = patrol_CAR;
					L_shoot_structure.status = Stop_Shoot;
					R_shoot_structure.status = Stop_Shoot;
					
				}
				/*�������ϣ�����ģʽ*/
				/*������� �� ����������ģʽ*/
				/*���̣��Ӿ�����ģʽ*/
				/*��̨���Ӿ�����ģʽ */
			}
			else if(S2_DOWN)
			{
		
				/*�Ӿ�ģʽ*/
				car_structure.mode = vision_CAR;
				L_shoot_structure.status = Single_Shoot;
				R_shoot_structure.status = Single_Shoot;
				/*����*/
				if(rc_structure.base_info->thumbwheel.status == down_R)
				{
					L_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
				}
				else if(rc_structure.base_info->thumbwheel.status == up_R)
				{
					R_shoot_structure.stir_wheel->base_info->target_angle_sum -= 36864;
				}
			}
		}	
	}
	else
	{
		/*ж��*/
		/*����ж����rp_chassis.c�����*/
		/*��̨ж����ҪͨѶ*/
		Master_Head_structure.Send_L_Head.shoot_mode = 0;
		Master_Head_structure.Send_R_Head.shoot_mode = 0;
		L_shoot_structure.status = Stop_Shoot;
		R_shoot_structure.status = Stop_Shoot;
		car_structure.mode = offline_CAR;
		
	}
	
	





}

















