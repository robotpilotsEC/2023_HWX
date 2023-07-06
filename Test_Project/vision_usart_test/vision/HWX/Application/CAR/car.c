#include "car.h"
#include "REMOTE.h"
#include "my_gimbal.h"
#include "judge_infantrypotocol.h"
#include "judge_sensor.h"
#include "my_shoot.h"
#include "my_chassis.h"
#include "bmi.h"


#define SWITCH_HEAD_TIME_MAX  2000

extern car_t      car_structure;
extern rc_t       rc_structure;
extern gimbal_t   gimbal_structure;
extern shoot_t    shoot_structure; 
extern chassis_t  chassis_structure;
extern bmi_t      bmi_structure;
/*换头标志位*/
int8_t            switch_head = 0;
uint16_t          switch_head_cnt = 0;

/*遥控器跳变检测变量*/
int8_t            s2_down_flag   = 1;
int8_t            s2_up_flag     = 1;


bool              gimbal_init_ok = 0;
bool              rc_conect      = 0;


extern judge_sensor_t         judge_sensor;


int8_t            key_K_jump_flag = 0;
int8_t            key_V_jump_flag = 0;


/*键盘跳变检测变量*/
int8_t            F_last       = 0;
int8_t            C_last       = 0;
int8_t            Q_last       = 0;
int8_t            E_last       = 0;
int8_t            B_last       = 0;
int8_t            R_last       = 0;
int8_t            Mouse_L_LAST = 0;

/*小陀螺变速变量*/
uint32_t           sin_tick = 0; 

/*堵转工作状态标志位*/
extern int8_t deal_done;
       int8_t fric_work = 1;
	   int8_t one_shoot_done = 1;
extern int8_t over_heat;
	   bool   hight_speed_shoot = 0;

  

void MODE_CHECK()
{
	
    /*遥控器模式*/
	if(rc_structure.base_info->s1.value == 1 && rc_structure.base_info->s2.value == 3)
	{
		car_structure.ctrl_mode = KEY_CAR;
	}
	else
	{
		car_structure.ctrl_mode = RC_CAR;
	}
	
    /*状态更新*/
    rc_wheel_status_interrupt_update(rc_structure.base_info);
	all_key_board_status_interrupt_update(rc_structure.base_info);
	all_key_board_status_update(rc_structure.base_info);
	
	
	if(rc_conect)
	{
		if(car_structure.ctrl_mode == RC_CAR)
		{
			if(rc_structure.base_info->s1.value == 2)//下拨码
			{
				/*机械模式*/
				car_structure.move_mode_status = machine_CAR;

			}
			else if(rc_structure.base_info->s1.value == 3)//中拨码
			{
				/*小陀螺模式*/
				if(rc_structure.base_info->thumbwheel.value >= 600)
				{
					car_structure.move_mode_status = spin_CAR;
					chassis_structure.info->target_cycle_speed = 2000;
					
					/*头尾判断(当小陀螺模式开启的时候才会判断头尾)*/
					if( gimbal_structure.yaw->base_info->angle    <= 52 \
						|| gimbal_structure.yaw->base_info->angle >= 4148)
					{
						gimbal_structure.front_or_back = -1;
					}
					else
					{
						gimbal_structure.front_or_back = 1;
					}

				}
				else 
				{
					/*跟随模式*/
					car_structure.move_mode_status =follow_CAR;
				}

			}	
			else if(rc_structure.base_info->s1.value == 1 ) 
			{
				if( rc_structure.base_info->s2.value == 2)
				{
					/*视觉模式（左拨码向上，右拨码向下）*/
					car_structure.move_mode_status = vision_CAR;//底盘工程，云台单独一个模式
				}
			}
				/*检测跳变(检测向下拨动)*/
				/*检测右拨杆向下*/
				if(rc_structure.base_info->s2.value_last != rc_structure.base_info->s2.value \
						&& rc_structure.base_info->s2.value == 2)
				{
					s2_down_flag = s2_down_flag*(-1);
				}

				/*检测右拨杆向上*/
				if(rc_structure.base_info->s2.value_last != rc_structure.base_info->s2.value \
						&& rc_structure.base_info->s2.value == 1)
				{
					s2_up_flag = s2_up_flag*(-1);
				}
				
				rc_structure.base_info->s2.value_last = rc_structure.base_info->s2.value;
				}
		
				
				
		/*键盘状态检测*/
		else if(car_structure.ctrl_mode == KEY_CAR)
		{	
			//为了保证没有更新，不能出现else
			
			/*风车复位*/
			if(rc_structure.base_info->Z.status == long_press_K)
			{
				if(rc_structure.base_info->X.status == long_press_K)
				{
					

						/*风车复位(进机械)*/
						car_structure.move_mode_status =offline_CAR;
						__set_FAULTMASK(1);
						NVIC_SystemReset();
						
						
						
				}
			}
			
			/*模式切换*/
			if(rc_structure.base_info->Ctrl.status == long_press_K)//长按ctrl进入机械模式
			{
				car_structure.move_mode_status = machine_CAR;
				car_structure.f_times = 0;
			}
			else if(rc_structure.base_info->Ctrl.status == short_press_K)
			{
				//关闭弹舱
				shoot_structure.box_open = 0;
				
				//关闭小陀螺
				if(car_structure.move_mode_status == spin_CAR)
				{
					car_structure.move_mode_status = follow_CAR;
					car_structure.f_times = 0;
				}
				//关闭45°朝向
				gimbal_structure.angle_45 = 0;
					
			}
			else if(rc_structure.base_info->V.status == long_press_K)//长按V进入跟随模式
			{
				car_structure.move_mode_status = follow_CAR;
				car_structure.f_times = 0;//小陀螺计数器清零
			}
			else if(rc_structure.base_info->V.status == short_press_K)//短按v侧向45度
			{
				gimbal_structure.angle_45 = 1;
			}
			else if(rc_structure.base_info->G.status == long_press_K)
			{
				car_structure.move_mode_status = vision_CAR;
			}
			/*发射机构*/
			else if(rc_structure.base_info->mouse_btn_l.status == long_press_K && fric_work)
			{
				shoot_structure.stri_mode = SIX_SHOOT; //连发波蛋轮
			}
			else if(rc_structure.base_info->mouse_btn_l.status == relax_K && (one_shoot_done) || shoot_structure.stri_mode == SIX_SHOOT )
			{
				shoot_structure.stri_mode = NOT_SHOOT;
			}
			
			
			
			
		 
			/*键盘跳变检测*/

			if(car_structure.move_mode_status == machine_CAR)
			{
				//B
				if(rc_structure.base_info->B.value != B_last && rc_structure.base_info->B.value == 1)
				{
					shoot_structure.box_open = !shoot_structure.box_open;//开关弹舱
				}
				B_last =rc_structure.base_info->B.value;
			}
			else if(car_structure.move_mode_status == follow_CAR)
			{
				//Q
				if(rc_structure.base_info->Q.value != Q_last && rc_structure.base_info->Q.value == 1)
				{
					gimbal_structure.info->target_yaw_angle += 2048;//快速旋转90
				}
				Q_last =rc_structure.base_info->Q.value;
				
				//E
				if(rc_structure.base_info->E.value != E_last && rc_structure.base_info->E.value == 1)
				{
					gimbal_structure.info->target_yaw_angle -= 2048;//快速旋转90
				}
				E_last =rc_structure.base_info->E.value;	
				
				//C C_last
				if(car_structure.move_mode_status == follow_CAR && rc_structure.base_info->C.value != C_last && rc_structure.base_info->C.value == 1 )
				{
					//gimbal_structure.front_or_back          *= -1;//快速换头而不是旋转180°
					gimbal_structure.info->target_yaw_angle += 4096;//头也要转180°
					switch_head = 1;
					chassis_structure.info->target_cycle_speed =0;
					
				}
				C_last =rc_structure.base_info->C.value;

				//左键
				if(rc_structure.base_info->mouse_btn_l.value != Mouse_L_LAST && rc_structure.base_info->mouse_btn_l.value == 1 && !deal_done &&!over_heat &&fric_work )
				{
					shoot_structure.stri_mode = ONE_SHOOT;
					shoot_structure.stir_wheel->base_info->target_angle_sum -= (int32_t)8191*4.5; //8191*4.5  36860
					one_shoot_done = 0;
				}
				Mouse_L_LAST =rc_structure.base_info->mouse_btn_l.value;
				//B
				if(rc_structure.base_info->B.value != B_last && rc_structure.base_info->B.value == 1 )
				{
					 fric_work = !fric_work ;//开关摩擦轮
					
				}
				B_last =rc_structure.base_info->B.value;
				//R R_last
				if(rc_structure.base_info->R.value != R_last && rc_structure.base_info->R.value == 1)
				{
					hight_speed_shoot = !hight_speed_shoot;
					
				}
				R_last =rc_structure.base_info->R.value;
			}
			else
			{
				
				//F
				if(rc_structure.base_info->F.value != F_last && rc_structure.base_info->F.value == 1)
				{
					car_structure.f_times++;
				}
				F_last =rc_structure.base_info->F.value;
								
			}
			
		
			
			
			/*换头判断*/
			if(switch_head == 1)
			{
				if(abs(gimbal_structure.info->target_yaw_angle -bmi_structure.yaw_angle) <= 50)
				{
					switch_head = 0;//防止单发未完成就松开
				}
			}
			/*单发完成判断*/
			
			if(one_shoot_done == 0)
			{
				if(abs(shoot_structure.stir_wheel->base_info->angle_sum -shoot_structure.stir_wheel->base_info->target_angle_sum) <= 20)//shoot_structure.stir_wheel->base_info->angle_sum -shoot_structure.stir_wheel->base_info->target_angle_sum
				{
					one_shoot_done = 1;
				}
			}
			
			/*头尾判断(当小陀螺模式开启的时候才会判断头尾)*///侧向45°会出现问题
			if( gimbal_structure.yaw->base_info->angle    <= 52   - gimbal_structure.angle_45 * 1024 \
				|| gimbal_structure.yaw->base_info->angle >= 4148 + gimbal_structure.angle_45 * 1024)
			{
				gimbal_structure.front_or_back = -1;
			}
			else
			{
				gimbal_structure.front_or_back = 1;
			}

			
			
			/*小陀螺模式切换处理*/
			if(rc_structure.base_info->F.status == short_press_K || car_structure.move_mode_status == spin_CAR)//短按f切换不同小陀螺速度，要增加跳变检测
			{
				car_structure.move_mode_status = spin_CAR;
				switch(car_structure.f_times % 3)
				{
					case 0:
					{
						chassis_structure.info->target_cycle_speed = 4000;
						
						break;
					}
					case 1:
					{
						chassis_structure.info->target_cycle_speed = 2000;
						
						break;
					}
					case 2:
					{
						chassis_structure.info->target_cycle_speed = 1500*sin( ((sin_tick++)%1300)*2*3.14f ) + 3500;
						
						break;
					}
				}
			}
		}
	else
	{
		/*遥控器掉线模式模式*/
		car_structure.move_mode_status =offline_CAR;
		rc_conect = 0;
	}
		
		
	}
	else
	{
		if(rc_structure.base_info->s1.value == 2 || rc_structure.base_info->s1.value == 1)//后面的是为了键盘模式复位增加的
		{
			rc_conect = 1;
			if(car_structure.ctrl_mode == RC_CAR)
			{
				car_structure.move_mode_status = machine_CAR;
			}
			else 
			{
				car_structure.move_mode_status = follow_CAR;
			}
			
		}
		car_structure.move_mode_status =offline_CAR;
	}
	
	/*键盘状态检测*/
	
	
	
	
	
	
	/*裁判系统心跳*/
	judge_sensor.heart_beat( &judge_sensor);
	
	
	rc_structure.info->mouse_x = rc_structure.base_info->mouse_vx;
	rc_structure.info->mouse_y = rc_structure.base_info->mouse_vy;
	
	/*鼠标速度滤波*/
	rc_structure.info->mouse_x_K = Low_Pass_Fliter(rc_structure.info->mouse_x_K,rc_structure.info->mouse_x,0.1);
	rc_structure.info->mouse_y_K = Low_Pass_Fliter(rc_structure.info->mouse_y_K,rc_structure.info->mouse_y,0.1);

}



