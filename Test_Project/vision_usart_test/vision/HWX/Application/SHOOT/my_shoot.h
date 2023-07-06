#ifndef __MY_SHOOT
#define __MY_SHOOT


#include "stm32F4xx_hal.h"
#include "3508_motor.h"
#include "2006_motor.h"

/*��������*/
#define SHOOT_OFFLINE_CNT_MAX  	  (50)
#define DEAL_DONE_SPEED      	  (900)
#define SHOOT_DONE_SPEED_MAX   	  (100)
#define SIX_SHOOT_DONE_TIME_MAX   (1000)
#define SHOOT_DONE_TIME_MAX    	  (5)
#define SHOOT_MY_SPEED            ((-1)*1000)
#define SHOOT_STRI_SPEED      	  ((-1)*4000)
#define SHOOT_STRI_SPEED_ERR  	  (100)
#define DEAL_DONE_ROUND_TIME   	  (600) 
#define DEAL_DONE_ROUND_HALF_TIME (300) 

/*Ħ�����ٶȻ�*/
#define PID_FIRC_OUTPUT_MAX    8000
#define PID_FIRC_INTEGRA_MAX   5000
#define PID_FIRC_KP            10
#define PID_FIRC_KI            0
#define PID_FIRC_KD            2

/*��ת�����ٶȻ�*/
#define PID_DONE_OUTPUT_MAX    9000
#define PID_DONE_INTEGRA_MAX   5000
#define PID_DONE_KP            8
#define PID_DONE_KI            0.01
#define PID_DONE_KD            0

/*����������*/
/*�ٶ�*/
#define SPID_STRI_OUTPUT_MAX    8000
#define SPID_STRI_INTEGRA_MAX   3000
#define SPID_STRI_KP            5
#define SPID_STRI_KI            0
#define SPID_STRI_KD            0

/*λ��*/
#define PPID_STRI_OUTPUT_MAX    3000
#define PPID_STRI_INTEGRA_MAX   1000
#define PPID_STRI_KP            0.5
#define PPID_STRI_KI            0
#define PPID_STRI_KD            0

/*�����������ٶȻ�*/
#define TPID_STRI_OUTPUT_MAX    9000
#define TPID_STRI_INTEGRA_MAX   5000
#define TPID_STRI_KP            8
#define TPID_STRI_KI            0.01
#define TPID_STRI_KD            0


#define ONE_SHOOT               0
#define NOT_SHOOT               1
#define SIX_SHOOT               2



typedef enum
{
    S_S_offline,  //ʧ��
    S_S_oneshot,  //����
    S_S_threeshot,//����
    S_S_done,     //��ת

} shoot_work_status_e;

typedef struct shoot_structure
{
    motor_2006_t*        stir_wheel;
    motor_3508_t*        fric_left;
    motor_3508_t*        fric_right;
    shoot_work_status_e  work_sta;
    uint8_t              cnt_three_done;
    uint8_t              cnt_right;
    uint8_t              cnt_stri;
    uint8_t              shoot_time;
    int16_t              shoot_speed;
	int8_t               stri_mode;
	uint16_t             done_time;
	uint16_t             six_done_time;
	uint16_t             cooling_heat;
	uint16_t             cooling_limit;
	
	bool                 box_open;
	
    void                 (*init)  (struct shoot_structure* shoot);
    void                 (*check) (struct shoot_structure* shoot);
    void                 (*update)(struct shoot_structure* shoot);
    void                 (*data)(struct shoot_structure* shoot);
    void                 (*work)(struct shoot_structure* shoot);
    void                 (*she_ji)(struct shoot_structure* shoot);
} shoot_t;


void SHOOT_INIT(shoot_t* shoot);
void DONE_CHECK(shoot_t* shoot);
void MODE_UPDATE(shoot_t* shoot);
void DATA_UPDATE(shoot_t* shoot);
void SHOOR_WORK(shoot_t* shoot);
void SHE_JI(shoot_t* shoot);
void DONE_WORK(shoot_t* shoot);

void shoot_task(void);


#define abs(x) 					((x)>0? (x):(-(x)))



#endif

