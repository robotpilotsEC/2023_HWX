/**
  ******************************************************************************
  * @file           : config_launcher.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#ifndef __CONFIG_LAUNCHER_H
#define __CONFIG_LAUNCHER_H

#define friction_motor_work_speed_init 7000
#define position_motor_work_speed_init 5400
#define position_motor_work_cnt_init   200
#define dial_motor_work_delay_cnt_init 100   //拨盘延时拨弹
#define auto_shoot_T_init              3000
#define dial_front_output_max_init     6000  //拨弹最大输出
#define dial_back_output_max_init      5000  //退弹最大输出
#define dial_keep_output_max_init      3000  //保持静止时最大输出

#define Reload_Times_init 10         //补弹次数上限
#define F_lock_Times_init 0          //补弹卡弹次数上限//3
#define Unload_Times_init 4          //退弹次数上限
#define B_lock_Times_init 2          //退弹卡弹次数上限
#define Reload_angle_init 33480      //补弹转动角度  //31462.5
#define Reload_angle_check_init 500  //补弹审查是否成功角度
#define Unload_angle_init 33480      //退弹转动角度
#define Unload_angle_check_init 500  //退弹审查是否成功角度
#define F_lock_angle_init 3000       //正向卡弹退后角度
#define F_lock_angle_check_init 100  //正向卡弹退后审查是否成功角度
#define B_lock_angle_init 1000       //反向卡弹退后角度
#define B_lock_angle_check_init 100  //反向卡弹退后审查是否成功角度
#define Reload_cnt_init 50           //补弹限时
#define F_lock_cnt_init 50           //正向卡弹限时
#define B_lock_cnt_init 50           //退弹限时
#define Unload_cnt_init 50           //反向卡弹限时

//#define LAUNCHER_PART_OFF     //关闭发射机构部分
//#define DIAL_PART_OFF         //关闭拨盘部分

#endif
