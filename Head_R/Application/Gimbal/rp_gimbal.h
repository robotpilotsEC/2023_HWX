#ifndef __RP_GIMBAL_H
#define __RP_GIMBAL_H



#include "stm32F4xx_hal.h"
#include "6020_motor.h"
#include "stdbool.h"
#include "dji_pid.h"
#include "bmi.h"

#define DATA_FROM_MOTOR            (0)
#define DATA_FROM_BMI              (1)
#define DATA_FROM_BMI_AND_MOTOR    (2)



/*下限 5074 上限 3640*/
#define BMI_UPP_LIMIT       (6170)
#define BMI_LOW_LIMIT       (4600)



#define BMI_YAW_MID         (4096)
#define BMI_PIT_MID         (4096)


#define HALF_CIRLE         (4096)
#define HALF_HALF_CIRLE    (1024)
#define YAW_MIDDLE_FOUNT   (2100)
#define YAW_MIDDLE_BACK    ((2100) + HALF_CIRLE)
#define PIT_MIDDLE         (4357)
#define PIT_RANGE          (717)
#define PIT_M_UP_RANGE     (PIT_MIDDLE - PIT_RANGE)
#define PIT_M_DOWN_RANGE   (PIT_MIDDLE + PIT_RANGE)
#define PIT_B_UP_RANGE     (HALF_CIRLE + PIT_RANGE)
#define PIT_B_DOWN_RANGE   (HALF_CIRLE - PIT_RANGE)
#define MOTOR_GAP_BMI      (PIT_MIDDLE - HALF_CIRLE)

/*主要是PITCH的变换*/
#define motor2bmi(m,b)     (b=(m)-MOTOR_GAP_BMI)
#define bmi2motor(b,m)     (m=(b)+MOTOR_GAP_BMI)

#define sgn(x)             (((x)>0)?1:((x)<0?-1:0))

/*BMI_AND_MOTOR*/
#if 0
/*PIT轴速度环*/
#define S_PIT_PID_P 5
#define S_PIT_PID_I 0.1
#define S_PIT_PID_D 0
#define S_PIT_LIM_I 1000    //积分上限
#define S_PIT_LIM_O 25000   //输出上限

/*PIT轴位置环*/
#define P_PIT_PID_P 20
#define P_PIT_PID_I 0
#define P_PIT_PID_D 2
#define P_PIT_LIM_I 500    //积分上限
#define P_PIT_LIM_O 25000   //输出上限

/*YAW轴速度环*/
#define S_YAW_PID_P 10
#define S_YAW_PID_I 0.1
#define S_YAW_PID_D 0
#define S_YAW_LIM_I 1000    //积分上限
#define S_YAW_LIM_O 25000   //输出上限

/*YAW轴位置环*/
#define P_YAW_PID_P 30
#define P_YAW_PID_I 0
#define P_YAW_PID_D 2
#define P_YAW_LIM_I 1000    //积分上限
#define P_YAW_LIM_O 25000   //输出上限
#endif
/*BMI_AND_MOTOR*/

/*BMI*/
#if 1
/*PIT轴速度环*/
#define S_PIT_PID_P 10       //10
#define S_PIT_PID_I 0.1
#define S_PIT_PID_D 0
#define S_PIT_LIM_I 1000    //积分上限
#define S_PIT_LIM_O 28000   //输出上限

/*PIT轴位置环*/
#define P_PIT_PID_P 10      //10
#define P_PIT_PID_I 0
#define P_PIT_PID_D 2
#define P_PIT_LIM_I 2000    //积分上限
#define P_PIT_LIM_O 28000   //输出上限

/*YAW轴速度环*/
#define S_YAW_PID_P 8
#define S_YAW_PID_I 0.1
#define S_YAW_PID_D 0
#define S_YAW_LIM_I 2000    //积分上限
#define S_YAW_LIM_O 25000   //输出上限

/*YAW轴位置环*/
#define P_YAW_PID_P 10      // 10  
#define P_YAW_PID_I 0.2
#define P_YAW_PID_D 2
#define P_YAW_LIM_I 100    //积分上限
#define P_YAW_LIM_O 28000   //输出上限

//yaw 纯电机参数 
#endif
//yaw 纯电机参数 

typedef enum
{
    Gimbal_Offline,  //失联
    Gimbal_Online,  //失联

} gimbal_status_e;

typedef struct 
{
	uint8_t          offline_cnt;
	uint8_t          offline_cnt_max;
	uint8_t          status;
	
}gimbal_work_info_t;

typedef struct 
{
	int16_t target_pit;
	int16_t target_yaw;
	uint8_t mode;
	
}gimbal_base_info_t;



/* 云台 */
typedef struct 
{
	motor_6020_t         *pitch;         //俯仰角
	motor_6020_t         *yaw;           //偏航角
	
	pid_t                *pid_s_pit;
	pid_t                *pid_s_yaw;
	
	pid_t                *pid_p_pit;
	pid_t                *pid_p_yaw;
	
	bmi_t                *bmi;
	
	gimbal_work_info_t    work;
	gimbal_base_info_t    base;
}gimbal_t;

/****************************************外部变量****************************************/
extern gimbal_t gimbal_structure;

/****************************************函数*******************************************/

/* 初始化 */
void gimbal_init(gimbal_t *gimbal);
//void gimbal_info_init(gimbal_info_t *info);
void gimbal_commond_init(gimbal_t *gimbal);

/* 控制与工作任务 */
void gimbal_ctrl_task(gimbal_t *gimbal);
void gimbal_mode_update(gimbal_t *gimbal);
void gimbal_commond_respond(gimbal_t *gimbal);
void gimbal_work(gimbal_t *gimbal);
void gimbal_pitch_work(gimbal_t *gimbal);
void gimbal_yaw_work(gimbal_t *gimbal);
void gimbal_yaw_angle_lock_ctrl(gimbal_t *gimbal);
void gimbal_yaw_lock_speed_ctrl(gimbal_t *gimbal);
//void gimbal_watch_work(gimbal_t *gimbal);

/* can更新 */
void gimbal_yaw_can_update(gimbal_t *gimbal);
void gimbal_pitch_can_update(gimbal_t *gimbal);
void gimbal_watch_can_update(gimbal_t *gimbal);

void Gimbal_Heart(gimbal_t* gimbal);

#endif

