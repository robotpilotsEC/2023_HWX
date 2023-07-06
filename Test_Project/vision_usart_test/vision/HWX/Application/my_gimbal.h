#ifndef __MY_GIMBAL_H
#define __MY_GIMBAL_H



#include "stm32F4xx_hal.h"
#include "6020_motor.h"
#define sgn(x) (((x)>0)?1:((x)<0?-1:0))
/****************************************工作模式枚举****************************************/

/* 云台模块 */
typedef enum 
{
	G_G_offline,  //失联
	G_G_init,     //初始化
	G_G_follow,   //跟随
	G_G_shoot,    //吊射
	G_G_check,    //检查补弹
}gimbal_work_status_e;

/* 云台pitch轴电机 */
typedef enum 
{
	G_P_offline,  //失联
	G_P_stop,     //停止
	G_P_speed,    //速度环
	G_P_lock,     //速度环堵转
	G_P_angle,    //角度环
	G_P_done,     //角度环堵转or就位
	G_P_normal,   //正常
	G_P_slow,     //慢速（吊射时）
}s_pitch_work_status_e;

/* 云台yaw轴电机 */
typedef enum 
{
	G_Y_offline,  //失联
	G_Y_stop,     //停止
	G_Y_angle,    //角度环
	G_Y_done,     //角度环堵转or就位
	G_Y_normal,   //正常
	G_Y_slow,     //慢速（吊射时）
}s_yaw_work_status_e;

/* 云台望远镜电机 */
typedef enum 
{
	G_W_offline,  //失联
	G_W_stop,     //停止
	G_W_speed,    //速度环
	G_W_lock,     //速度环堵转
	G_W_angle,    //角度环
	G_W_done,     //角度环堵转or就位
	G_W_retract,  //收镜
	G_W_open,     //开镜
}watch_work_status_e;

/****************************************工作信息结构体****************************************/

/* 云台模块 */
typedef struct
{
	uint8_t command;
	uint8_t status;
	uint8_t init_Y_O_N;
	uint8_t work_status;
}gimbal_work_info_t;

/* 云台pitch轴电机 */
typedef struct
{
	uint8_t status;
	int16_t cnt;
}s_pitch_work_info_t;

/* 云台yaw轴电机 */
typedef struct
{
	uint8_t status;
	int16_t cnt;
}s_yaw_work_info_t;

/* 云台望远镜电机 */
typedef struct
{
	uint8_t status;
	int16_t cnt;
}watch_work_info_t;

/****************************************模块结构体****************************************/

/* 云台配置 */
typedef struct 
{
	int16_t watch_init_out_max;     //望远镜初始化时输出最大值（由于速度环堵转，保护）
	int16_t watch_normal_out_max;   //望远镜正常时输出最大值
	int16_t watch_init_speed;       //望远镜初始化速度
	int32_t watch_up_angle;         //望远镜上端角度
	int16_t pitch_init_speed;       //pitch轴初始化速度
	int16_t pitch_up_angle;         //pitch轴上端角度
	int16_t yaw_check_angle;        //yaw轴审查补弹角度
	int16_t pitch_check_angle;      //pitch轴审查补弹角度
	int16_t yaw_middle_angle;       //yaw轴角度中值
	int16_t lock_cnt_max;           //堵转计数上限
	int16_t pitch_angle_permit_max; //pitch轴角度允许上限
	int16_t pitch_angle_permit_min; //pitch轴角度允许下限
}gimbal_config_t;

/* 云台信息 */
typedef struct
{
	int16_t  pitch_speed;        //偏航角速度
	int16_t  
	; //目标偏航角速度
	int32_t  pitch_angle;        //偏航角
	int32_t  target_pitch_angle; //目标偏航角
	int16_t  yaw_speed;          //
	int16_t  target_yaw_speed;   //
	int16_t  yaw_angle;          //
	int16_t  target_yaw_angle;   //
	int16_t  watch_speed;        //
	int16_t  target_watch_speed; 
	int32_t  watch_angle;
	int32_t  target_watch_angle;
	int16_t  return_yaw_angle;
	int32_t  return_pitch_angle;
	uint8_t  shoot_watch_flag;
}gimbal_info_t;

/* 云台 */
typedef struct 
{
	motor_6020_t    *pitch;         //俯仰角
	motor_6020_t    *yaw;           //偏航角
	motor_6020_t    *watch;         //进弹？ 暂时用6020代替
	
	gimbal_work_info_t *work;       //云台模块
	s_pitch_work_info_t *pitch_work;//云台pitch轴电机
	s_yaw_work_info_t *yaw_work;    //云台yaw轴电机
  watch_work_info_t *watch_work;
	
	gimbal_config_t *config;        //云台配置
	gimbal_info_t   *info;          //云台信息
}gimbal_t;

/****************************************外部变量****************************************/
extern gimbal_t gimbal_structure;

/****************************************函数****************************************/

/* 初始化 */
void gimbal_init(gimbal_t *gimbal);
void gimbal_info_init(gimbal_info_t *info);
void gimbal_commond_init(gimbal_t *gimbal);

/* 控制与工作任务 */
void gimbal_ctrl_task(gimbal_t *gimbal);
void gimbal_mode_update(gimbal_t *gimbal);
void gimbal_commond_respond(gimbal_t *gimbal);
void gimbal_work(gimbal_t *gimbal);
void gimbal_pitch_work(gimbal_t *gimbal);
void gimbal_yaw_work(gimbal_t *gimbal);
//void gimbal_watch_work(gimbal_t *gimbal);

/* can更新 */
void gimbal_yaw_can_update(gimbal_t *gimbal);
void gimbal_pitch_can_update(gimbal_t *gimbal);
void gimbal_watch_can_update(gimbal_t *gimbal);

/* 审查 */
void gimbal_pitch_angle_check(gimbal_t *gimbal);
void gimbal_yaw_angle_check(gimbal_t *gimbal);

/* 环控制 */
void gimbal_yaw_speed_ctrl(gimbal_t *gimbal);
void gimbal_yaw_angle_ctrl(gimbal_t *gimbal);
void gimbal_pitch_speed_ctrl(gimbal_t *gimbal);
void gimbal_pitch_angle_ctrl(gimbal_t *gimbal);
void gimbal_watch_speed_ctrl(gimbal_t *gimbal);
void gimbal_watch_angle_ctrl(gimbal_t *gimbal);


#endif

