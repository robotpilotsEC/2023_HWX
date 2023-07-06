#ifndef __MY_CHASSIS_H
#define __MY_CHASSIS_H

#include "3508_motor.h"

#define _RC_CAR 1


typedef enum 
{
	C_M_offline, //失联模式
	C_M_stop,    //停止模式
	C_M_normal,  //普通模式（以底盘朝向为正方向）
	C_M_special, //特殊模式（以云台朝向为正方向）
	C_M_follow,
	C_M_spin,
}chassis_move_mode_e;

/* 底盘旋转模式枚举 */
typedef enum 
{
	C_C_offline, //失联模式
	C_C_stop,    //停止模式
	C_C_normal,  //正常模式（可控制）
	C_C_follow,  //跟随模式（不可前后颠倒）
	C_C_back,    //归位模式（不可前后颠倒）
	C_C_cycle,   //小陀螺模式
	C_C_lock,
	C_C_spin,
}chassis_cycle_mode_e;

///* 底盘工作模式枚举 */
//typedef enum 
//{
//	offline_CAR, //失联模式
//	init_CAR,    //停止模式
//	high_shoot_CAR,  //正常模式（可控制）
//	machine_CAR,  //跟随模式（不可前后颠倒）
//	test_CAR
//}chassis_work_mode_e;




typedef struct
{
	uint8_t chassis_cycle_speed_mode; //小陀螺模式下转速模式 //0匀速转动 //1匀加速转动 //2三角函数式
	uint8_t chassis_target_speed_mode; //底盘target速度变化模式 //0根据按键计数梯形变化 //1单值过微分跟踪器
	int16_t chassis_cycle_mode_0_speed; //底盘小陀螺模式0速度
	int16_t chassis_cycle_mode_1_speed; //底盘小陀螺模式1速度
	int16_t chassis_cycle_mode_2_speed; //底盘小陀螺模式1速度
	uint16_t chassis_output_max; //底盘最大输出
	uint16_t chassis_speed_max; //底盘最高速度
}chassis_config_t;

/* 底盘信息 */
typedef struct 
{
	int16_t target_front_speed;  //目标前进速度
	int16_t target_right_speed;  //目标左移速度
	int16_t target_cycle_speed;  //目标旋转速度
	int16_t measure_front_speed; //当前前进速度
	int16_t measure_right_speed; //当前左移速度
	int16_t measure_cycle_speed; //当前旋转速度
	int16_t measure_front_speed_dif; //当前前进速度差分
	int16_t measure_right_speed_dif; //当前左移速度差分
	uint8_t move_mode;           //移动模式
	uint8_t cycle_mode;          //旋转模式
	int16_t back_cnt;            //归位计数
	int16_t speed_cnt;           //速度函数计数
	int16_t spin_front;          //陀螺模式下的前进速度
	int16_t spin_right;          //陀螺模式下的平移速度
}chassis_info_t;

/* 底盘 */
typedef struct 
{
	motor_3508_t        *motor_LF;
	motor_3508_t        *motor_RF;
	motor_3508_t        *motor_LB;
	motor_3508_t        *motor_RB;
	chassis_config_t    *config;
	chassis_info_t      *info;
//	chassis_work_mode_e mode;
	
}chassis_t;

/* 外部变量 */
extern chassis_t chassis_structure;
extern chassis_config_t       chassis_config_structure;//底盘设置
extern chassis_info_t         chassis_info_structure;  //底盘信息


/* 初始化 */
void chassis_init(chassis_t *chassis);
void chassis_info_init(chassis_info_t *info);
void chassis_commond_init(chassis_t *chassis);

/* 控制任务 */
void chassis_ctrl_task(chassis_t *chassis);
void chassis_mode_update(chassis_t *chassis);
void chassis_measure_speed_update(chassis_t *chassis);
void chassis_commond_respond(chassis_t *chassis);
void chassis_work(chassis_t *chassis);
void chassis_judge_limit(chassis_t *chassis);

void motor_chassis_update(chassis_t *chassis);
void motor_speed_limit(chassis_t *chassis, int16_t *front, int16_t *right, int16_t *round);

void chassis_cycle_auto_check(void);
void chassis_speed_max_auto_set(chassis_t *chassis);
void spin_work(void);


#endif
