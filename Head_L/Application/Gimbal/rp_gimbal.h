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



/*���� 5074 ���� 3640*/
#define BMI_UPP_LIMIT       (6170)
#define BMI_LOW_LIMIT       (4600)

#define MOT_UPP_LIMIT       (4096)  
#define MOT_LOW_LIMIT       (4096)

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

/*��Ҫ��PITCH�ı任*/
#define motor2bmi(m,b)     (b=(m)-MOTOR_GAP_BMI)
#define bmi2motor(b,m)     (m=(b)+MOTOR_GAP_BMI)

#define sgn(x)             (((x)>0)?1:((x)<0?-1:0))

/*BMI_AND_MOTOR*/

#if 1
/*PIT���ٶȻ�*/
#define S_PIT_PID_P 10       //10
#define S_PIT_PID_I 0.1
#define S_PIT_PID_D 0
#define S_PIT_LIM_I 1000    //��������
#define S_PIT_LIM_O 28000   //�������

/*PIT��λ�û�*/
#define P_PIT_PID_P 10      //10
#define P_PIT_PID_I 0
#define P_PIT_PID_D 2
#define P_PIT_LIM_I 2000    //��������
#define P_PIT_LIM_O 28000   //�������

/*YAW���ٶȻ�*/
#define S_YAW_PID_P 8
#define S_YAW_PID_I 0.1
#define S_YAW_PID_D 0
#define S_YAW_LIM_I 2000    //��������
#define S_YAW_LIM_O 25000   //�������

/*YAW��λ�û�*/
#define P_YAW_PID_P 10      // 10  
#define P_YAW_PID_I 0.2
#define P_YAW_PID_D 2
#define P_YAW_LIM_I 100    //��������
#define P_YAW_LIM_O 28000   //�������


//yaw ��������� 

//yaw ��������� 
#endif
/*BMI_AND_MOTOR*/

#if 0
/*PIT���ٶȻ�*/
#define S_PIT_PID_P 15       //7
#define S_PIT_PID_I 0.1
#define S_PIT_PID_D 0
#define S_PIT_LIM_I 1000    //��������
#define S_PIT_LIM_O 20000   //�������

/*PIT��λ�û�*/
#define P_PIT_PID_P 10
#define P_PIT_PID_I 0
#define P_PIT_PID_D 2
#define P_PIT_LIM_I 500    //��������
#define P_PIT_LIM_O 25000   //�������


/*YAW���ٶȻ�*/
#define S_YAW_PID_P 15
#define S_YAW_PID_I 0.1
#define S_YAW_PID_D 0
#define S_YAW_LIM_I 1000    //��������
#define S_YAW_LIM_O 20000   //�������

/*YAW��λ�û�*/
#define P_YAW_PID_P 10
#define P_YAW_PID_I 0
#define P_YAW_PID_D 2
#define P_YAW_LIM_I 1000    //��������
#define P_YAW_LIM_O 25000   //�������

//yaw ��������� 
#endif

typedef enum
{
    Gimbal_Offline,  //ʧ��
    Gimbal_Online,  //ʧ��

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



/* ��̨ */
typedef struct 
{
	motor_6020_t         *pitch;         //������
	motor_6020_t         *yaw;           //ƫ����
	
	pid_t                *pid_s_pit;
	pid_t                *pid_s_yaw;
	
	pid_t                *pid_p_pit;
	pid_t                *pid_p_yaw;
	
	bmi_t                *bmi;
	
	gimbal_work_info_t    work;
	gimbal_base_info_t    base;
}gimbal_t;

/****************************************�ⲿ����****************************************/
extern gimbal_t gimbal_structure;

/****************************************����****************************************/

/* ��ʼ�� */
void gimbal_init(gimbal_t *gimbal);
//void gimbal_info_init(gimbal_info_t *info);
void gimbal_commond_init(gimbal_t *gimbal);

/* �����빤������ */
void gimbal_ctrl_task(gimbal_t *gimbal);
void gimbal_mode_update(gimbal_t *gimbal);
void gimbal_commond_respond(gimbal_t *gimbal);
void gimbal_work(gimbal_t *gimbal);
void gimbal_pitch_work(gimbal_t *gimbal);
void gimbal_yaw_work(gimbal_t *gimbal);
void gimbal_yaw_angle_lock_ctrl(gimbal_t *gimbal);
void gimbal_yaw_lock_speed_ctrl(gimbal_t *gimbal);
//void gimbal_watch_work(gimbal_t *gimbal);

/* can���� */
void gimbal_yaw_can_update(gimbal_t *gimbal);
void gimbal_pitch_can_update(gimbal_t *gimbal);
void gimbal_watch_can_update(gimbal_t *gimbal);

void Gimbal_Heart(gimbal_t* gimbal);

#endif

