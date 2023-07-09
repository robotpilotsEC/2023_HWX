/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v2.0		
  * @Author     : hwx			
  * @Date       : 2023-7-06         
  * @Description:    
  *
  *
  ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "rp_chassis.h"
#include "remote.h"
#include "Car.h"
#include "vision.h"
#include "bmi.h"
#include "rp_gimbal.h"
#include "judge.h"
#include "judge_protocol.h"
#include "math.h"

/* Private macro -------------------------------------------------------------*/
#define change_p              (1.0f)
#define pi                    (3.14159265354f)

#define UNDER_SMALL_HIT       (1)
#define UNDER_BIGGG_HIT       (2)
#define NO_HIT                (0)

#define AERIAL_W              (1)
#define AERIAL_A              (2)
#define AERIAL_S              (3)
#define AERIAL_D              (4)
#define AERIAL_NO             (5)
#define AERIAL_MOVE_TIME      (1500)


/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float                         cita        = 0;  

uint32_t                      sin_cnt     = 0;
uint32_t                      sin_tick    = 0; 
uint32_t                      cos_tick    = 0; 
uint32_t                      hurt_cnt    = 0;

uint16_t                      right_cnt   = 0;
uint16_t                      fornt_cnt   = 0;
uint16_t                      right_time  = 0;
uint16_t                      fornt_time  = 0;
uint16_t                      aerial_time = 0; //��̨��ָ������ʱ��

int32_t                       hurt_change = 0;
int32_t                       vision_spin_time = 0;

int16_t                       spin_speed  = 0;
int16_t                       vision_spin_speed = 0;

uint8_t                       game_star   = 0;
uint8_t                       position_ok = 0;
uint8_t                       hurt_mode   = 0;

uint8_t                       chassis_aerial_cmd = AERIAL_NO;
/* Exported variables --------------------------------------------------------*/
extern chassis_t              Chassis; 
extern rc_t                   rc_structure;
extern car_t                  car_structure;
extern vision_t               vision_structure;
extern bmi_t                  bmi_structure;
extern gimbal_t               Gimbal;
extern judge_t                judge;
extern pid_t                  chassis_pid_follow_structure;

/* Exported macro ------------------------------------------------------------*/

/* Function  body --------------------------------------------------------*/
/**
  * @Name    chassis_work
  * @brief   ��������
  * @param   ���� 
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void chassis_work()
{
	
	if(car_structure.ctrl == RC_CAR)
	{
		/*��еģʽ*/
		if(car_structure.mode == machine_CAR)
		{
			/*���̿��Ʋ���*/
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.cycle_speed = (float)rc_structure.base_info->ch0 / 660.f * Chassis.work_info.config.speed_max;
			
			/*״̬λ���*/
			Chassis.config_top_calc(CHASSIS_TOP_CALC_OFF);
			sin_cnt = 0;
			
			/*�����ٶ�����*/
			if(rc_structure.base_info->thumbwheel.value == -660)
			{
				if(Chassis.work_info.config.speed_max ++ >= 10000)
				{
					Chassis.work_info.config.speed_max = 10000;
				}
			}
			else if(rc_structure.base_info->thumbwheel.value == 660)
			{
				if(Chassis.work_info.config.speed_max -- <= 0)
				{
					Chassis.work_info.config.speed_max = 0;
				}					
			}	
		}
		else if(car_structure.mode == navigation_CAR)
		{	
			
			Chassis.base_info.target.front_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_front*change_p;//chassis_front �Ӿ��� ǰ��
			Chassis.base_info.target.right_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_right*change_p; //�Ӿ��� ����
			
			Chassis.base_info.target.cycle_speed = 1000;
			
						
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
			
			
		}
		else if(car_structure.mode == spin_CAR )
		{
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)CHASSIS_SPEED_MAX / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)CHASSIS_SPEED_MAX / 660.f;
			Chassis.base_info.target.cycle_speed = spin_speed;
			
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
			
			/*�����ٶ�����*/
			if(rc_structure.base_info->thumbwheel.value == -660)
			{
				if(spin_speed ++ >= 7000)
				{
					spin_speed = 7000;
				}
			}
			else if(rc_structure.base_info->thumbwheel.value == 660)
			{
				if(spin_speed -- <= -7000)
				{
					spin_speed = -7000;
				}					 
			}
			
		}
		else if(car_structure.mode == two_CAR)
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 0;

			
		}
		else if(car_structure.mode == follow_CAR)
		{
			/*���̿��Ʋ���*/
			Chassis.base_info.target.front_speed = (-1)*(float)rc_structure.base_info->ch3 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.right_speed = (-1)*(float)rc_structure.base_info->ch2 * (float)Chassis.work_info.config.speed_max / 660.f;
			Chassis.base_info.target.cycle_speed = pid_calc_err_9015(&chassis_pid_follow_structure,
																	  Gimbal.Yaw_9015->base_info->encoder,\
																	  MOROT_9015_MIDDLE); //��pid�������
			
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			if(Gimbal.Yaw_9015->info->status == _9015_OFFLINE)
			{
				Chassis.base_info.target.cycle_speed = 0;	
			}
			Chassis.config_top_calc(CHASSIS_TOP_CALC_OFF);
			

			
		}
		else if(car_structure.mode == patrol_CAR || car_structure.mode ==  shake_CAR || car_structure.mode == vision_CAR)
		{
			/*ת�ٱ仯*/
			if(vision_spin_time++ %30000 <= 7000)
			{
				vision_spin_speed = 3300;
			}
			else if(vision_spin_time++ %30000 >= 140000)
			{
				vision_spin_speed = 3500 +  300*sin( ((sin_tick)%1300)*2*3.14f );
			}
			else
			{
				vision_spin_speed = -3300;
			}
			
			/*����С����*/
			if(judge.base_info->game_progress == 4 && judge.base_info->friendly_outposts_HP == 0)
			{
				Chassis.base_info.target.cycle_speed = vision_spin_speed + 0*sin( ((sin_tick++)%1000)*2*3.14f ) + hurt_change;
			}
			else
			{
				Chassis.base_info.target.cycle_speed = 1000;
			}
			
			/*��ת�⸳ֵ*/
			Chassis.base_info.target.front_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_front*change_p;//chassis_front �Ӿ��� ǰ��
			Chassis.base_info.target.right_speed = (-1)*(float)vision_structure.rx_pack->RxData.chassis_right*change_p; //�Ӿ��� ����
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			
			Chassis_Top_Speed_Calculating(&Chassis);
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
			
			/*״̬�ı�*/
			if(judge.base_info->last_HP - judge.base_info->remain_HP >= 9 && hurt_mode == NO_HIT)
			{
				hurt_mode = UNDER_SMALL_HIT;
			}
			else if(judge.base_info->last_HP - judge.base_info->remain_HP >= 99 && hurt_mode == NO_HIT)
			{
				hurt_mode = UNDER_BIGGG_HIT;
			}
			
			/*״̬����*/
			if(hurt_mode == UNDER_SMALL_HIT)
			{
				if(hurt_cnt++ > 2700)
				{
					hurt_cnt  = 0;
					hurt_mode = NO_HIT;
				}
				
				if(hurt_cnt%1700 >= 900)
				{
					if(vision_spin_speed>= 0 )
					{
						hurt_change = -7000;
					}
					else
					{
						hurt_change = -500;
					}						
				}
				else
				{
					hurt_change = 700;
				}
			}
			else if(hurt_mode == UNDER_BIGGG_HIT)
			{
				if(hurt_cnt++ > 6000)
				{
					hurt_cnt  = 0;
					hurt_mode = NO_HIT;
				}
				
				if(hurt_cnt%3000 >= 2000)
				{
					if(vision_spin_speed>= 0 )
					{
						hurt_change = -6000;
					}
					else
					{
						hurt_change = -700;
					}						
				}
				else if(hurt_cnt%3000 <= 1000)
				{
					if(vision_spin_speed>= 0 )
					{
						hurt_change = 600;
					}
					else
					{
						hurt_change = -500;
					}	
				}
				else
				{
					hurt_change = 1000;
				}
			}
			else
			{
				hurt_change = 0;
			}

		}
		/*���湤��*/
		else if(1)
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 0;
		}
		else 
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 0;
			Chassis.base_info.target.cycle_speed = 0;
		}
		
	}
	else if(car_structure.ctrl == MINIPC_CAR)
	{
		
		/*��ֵ*/
		Chassis.base_info.target.front_speed = (float)vision_structure.rx_pack->RxData.chassis_front;
		Chassis.base_info.target.right_speed = (float)vision_structure.rx_pack->RxData.chassis_right;
		Chassis.base_info.target.cycle_speed = (float)vision_structure.rx_pack->RxData.chassis_cycle;
		
	}
	else if(car_structure.ctrl == TEST_CAR)
	{
		Chassis.base_info.target.front_speed = (float)rc_structure.base_info->ch3 * (float)CHASSIS_SPEED_MAX / 660.f;
		Chassis.base_info.target.right_speed = (float)rc_structure.base_info->ch2 * (float)CHASSIS_SPEED_MAX / 660.f;
		Chassis.base_info.target.cycle_speed = (float)rc_structure.base_info->ch0 / 660.f * CHASSIS_SPEED_MAX;
		
	}
	
	/*��̨��ָ��*/
	if(judge.info->status == JUDGE_ONLINE)
	{
		if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->robot_commond == 'W') //'W'
		{
			chassis_aerial_cmd = AERIAL_W;
		}
		else if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->robot_commond == 'A') //'A'
		{
			chassis_aerial_cmd = AERIAL_A;
		}
		else if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->robot_commond == 'S') //'S'
		{
			chassis_aerial_cmd = AERIAL_S;
		} 
		else if(judge.base_info->robot_commond != judge.base_info->last_commond && judge.base_info->robot_commond == 'D') //'D'
		{
			chassis_aerial_cmd = AERIAL_D;
		}
	}
	else
	{
		chassis_aerial_cmd = AERIAL_NO;
	}
	
	if(chassis_aerial_cmd == AERIAL_W)
	{
		if(aerial_time++ >= AERIAL_MOVE_TIME)
		{
			chassis_aerial_cmd = AERIAL_NO;
			aerial_time = 0;
		}
		else
		{
			Chassis.base_info.target.front_speed = -1000;
			Chassis.base_info.target.right_speed = 0;
				
			/*����С���ݽ���*/
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			/*�����ٶ�����*/
			Chassis.base_info.measure.top_detal_angle = (float)((bmi_structure.first_yaw_angle - bmi_structure.yaw_angle )/8192.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_A)
	{
		if(aerial_time++ >= AERIAL_MOVE_TIME)
		{
			chassis_aerial_cmd = AERIAL_NO;
			aerial_time = 0;
		}
		else
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = -1000;
				
			/*����С���ݽ���*/
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			/*�����ٶ�����*/
			Chassis.base_info.measure.top_detal_angle = (float)((bmi_structure.first_yaw_angle - bmi_structure.yaw_angle )/8192.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_D)
	{
		if(aerial_time++ >= AERIAL_MOVE_TIME)
		{
			chassis_aerial_cmd = AERIAL_NO;
			aerial_time = 0;
		}
		else
		{
			Chassis.base_info.target.front_speed = 0;
			Chassis.base_info.target.right_speed = 1000;
				
			/*����С���ݽ���*/
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			/*�����ٶ�����*/
			Chassis.base_info.measure.top_detal_angle = (float)((bmi_structure.first_yaw_angle - bmi_structure.yaw_angle )/8192.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	else if(chassis_aerial_cmd == AERIAL_S)
	{
		if(aerial_time++ >= AERIAL_MOVE_TIME)
		{
			chassis_aerial_cmd = AERIAL_NO;
			aerial_time = 0;
		}
		else
		{
			Chassis.base_info.target.front_speed = 1000;
			Chassis.base_info.target.right_speed = 0;
				
			/*����С���ݽ���*/
			Chassis.base_info.measure.top_detal_angle = (float)((MOROT_9015_MIDDLE -  Gimbal.Yaw_9015->base_info->encoder )/65536.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			/*�����ٶ�����*/
			Chassis.base_info.measure.top_detal_angle = (float)((bmi_structure.first_yaw_angle - bmi_structure.yaw_angle )/8192.0)*2*PI;
			Chassis_Top_Speed_Calculating(&Chassis);
			
			
			Chassis.config_top_calc(CHASSIS_TOP_CALC_ON);
		}
	}
	
	Chassis_Work(&Chassis);

}

#define DISTANCE_MAX   21.0f
#define RADIUS         1.0f
#define STARING_YAW    4096
void Follow_Engineer(int16_t *speed_x, int16_t *speed_y, float first_yaw_angle)
{
	/*��ȡ����λ��*/
	float Engi_x = judge.data->robot_position.engineer_x;
	float Engi_y = judge.data->robot_position.engineer_y;
	
	/*��ȡ����λ��*/
	float Shao_x = judge.data->game_robot_pos.x;
	float Shao_y = judge.data->game_robot_pos.y;
	
	float dx = Engi_x - Shao_x;
	float dy = Engi_y - Shao_y;
	
	
	/*�������*/
	int16_t distance = sqrt(pow((dx),2) + pow((dy),2));
	
	if(distance <= DISTANCE_MAX && distance >= RADIUS)
	{
		// ����Ƕȣ��Ի���Ϊ��λ��
		float angle_rad = atan2(dy, dx);

		// ���Ƕ�ת��Ϊ�� x ��������Ϊ����˳ʱ��Ƕȣ��Զ�Ϊ��λ��
		float angle_deg = angle_rad * 180.0 / PI ;
		angle_deg = angle_deg * 22.7555 + first_yaw_angle;
		if (angle_deg < 0) 
		{
			angle_deg += 8192.0;
		}
		else if(angle_deg > 8191)
		{
			angle_deg -= 8192.0;
		}
		
		/*�ٶȼ���*/
		*speed_x = dx*100 + signbit(dx) * 500;
		*speed_y = dy*100 + signbit(dy) * 500;
		
		/*��Χ����*/
		if(*speed_y >= 3000)
		{
			*speed_y = 3000;
		}
		else if(*speed_y <= -3000)
		{
			*speed_y = -3000;
		}
		if(*speed_x >= 3000)
		{
			*speed_x = 3000;
		}
		else if(*speed_x <= -3000)
		{
			*speed_x = -3000;
		}
		
		/*�������������ϵ�¹���������ҵĽǶ�*/
		/*С���ݽ���*/
		Chassis.base_info.measure.top_detal_angle = (float)((first_yaw_angle - angle_deg )/8192.0)*2*PI;
		Chassis_Top_Speed_Calculating(&Chassis);
		

	}
	else if(distance <= RADIUS)
	{
		*speed_x = 0;
		*speed_y = 0;
	}
	
	
}

