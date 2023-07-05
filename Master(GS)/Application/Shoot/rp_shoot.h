
#ifndef __RP_SHOOT_H_
#define __RP_SHOOT_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "math.h"
#include "dji_pid.h"
#include "drv_can.h"
#include "3508_motor.h"
#include "2006_motor.h"

/* Private macro -------------------------------------------------------------*/
#define RELOAD_CNT                (10)

#define FRIC_TESE_SPEED           ((-1)*4000)

#define SHOOT_OFFLINE_CNT_MAX  	  (50)
#define DEAL_DONE_SPEED      	  (5000)
#define SHOOT_DONE_SPEED_MAX   	  (100)

#define SIX_SHOOT_DONE_TIME_MAX   (140)

#define SHOOT_DONE_TIME_MAX    	  (5)

#define SHOOT_MY_SPEED            ((-1)*1000)

#define SHOOT_STRI_SPEED      	  ((-1)*2000)
#define SHOOT_STRI_SPEED_ERR  	  (100)

/*反转角度太大导致容易一直进堵转*/
/*堵转的时候不检测堵转？*/
#define DEAL_DONE_ROUND_TIME   	  (50) //堵转处理时间
#define DEAL_DONE_ROUND_HALF_TIME (25) 

#define SHOOT_HEAT_MAX            (240) //枪管热量上限

#define DEAL_TIME_MAX             (20)

/*摩擦轮速度环*/
#define PID_FIRC_OUTPUT_MAX    8000
#define PID_FIRC_INTEGRA_MAX   5000
#define PID_FIRC_KP            10
#define PID_FIRC_KI            0.01
#define PID_FIRC_KD            0

/*堵转处理速度环*/
#define PID_DONE_OUTPUT_MAX    9800
#define PID_DONE_INTEGRA_MAX   5000
#define PID_DONE_KP            8
#define PID_DONE_KI            0.01
#define PID_DONE_KD            0

/*单发波胆轮*/
/*速度*/
#define SPID_STRI_OUTPUT_MAX    9800
#define SPID_STRI_INTEGRA_MAX   3000
#define SPID_STRI_KP            7
#define SPID_STRI_KI            0
#define SPID_STRI_KD            0

/*位置*/
#define PPID_STRI_OUTPUT_MAX    8000
#define PPID_STRI_INTEGRA_MAX   1000
#define PPID_STRI_KP            0.5
#define PPID_STRI_KI            0
#define PPID_STRI_KD            0

/*连发波胆轮速度环*/
#define TPID_STRI_OUTPUT_MAX    9800
#define TPID_STRI_INTEGRA_MAX   3000
#define TPID_STRI_KP            15
#define TPID_STRI_KI            0
#define TPID_STRI_KD            0



/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Exported variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


/* Exported macro ------------------------------------------------------------*/






/* Exported types ------------------------------------------------------------*/




typedef enum
{
	Single_Shoot,
	Running_Shoot,
	Stop_Shoot,
	Visin_Shoot,

} shoot_work_status_e;


typedef enum
{
    Shoot_Offline,  //失联
    Shoot_Online,  //失联

} shoot_status_e;


/** 
  * @brief  标志位定义
  */ 
typedef struct shoot_flag_t
{	

	/*标志位*/
	uint8_t               locked;        //堵转
	uint8_t               one_shoot_done;//单发上弹完成
	uint8_t               vision_first_shoot; //视觉单发是否打出
}shoot_flag_t;

/** 
  * @brief  计数器定义
  */ 
typedef struct shoot_cnt_t
{	

	/*标志位*/
	uint16_t               done_time;        //堵转
	uint16_t               deal_time;
	uint16_t               deal_done_cnt;
	uint16_t               shoot_cnt;
	uint16_t               last_shoot_cnt; 	
	
}shoot_cnt_t;


/** 
  * @brief  计数器定义
  */ 
typedef struct shoot_base_info_t
{	

	/*标志位*/
	int16_t            shoot_speed;        //摩擦轮转速
	int16_t            last_shoot_speed;
	int16_t            firct_speed;
	
	/*状态*/
	shoot_status_e     status;
	
}shoot_base_info_t;

/** 
  * @brief  底盘类定义
  */ 
typedef struct shoot_class_t
{	
	/*外部电机*/
    motor_2006_t         *stir_wheel;
	
	pid_t                *pid_s_stir_sin;
	pid_t                *pid_p_stir_sin;
	
	pid_t                *pid_p_stir_done;
	
	pid_t                *pid_s_stir_run;
	
	/*标志位*/
	shoot_flag_t          flag;
	
	/*计数器*/
	shoot_cnt_t           cnt;
	
	shoot_base_info_t     base;
	
	shoot_work_status_e   status;

}shoot_t;





/* Exported functions --------------------------------------------------------*/

/*发射机构初始化*/
void SHOOT_INIT(void);

/*重装载*/
void Shoot_Reload(shoot_t * shoot);

/*连发*/
void Running_Fire(shoot_t * shoot);

/*单发*/
void Single_Fire(shoot_t * shoot);
	
/*检测单发上弹是否完成*/
void Check_One_Shoot(shoot_t * shoot);

/*堵转检测*/
void Done_Check(shoot_t * shoot);

/*摩擦轮工作*/
void Fric_Work(shoot_t * shoot);

/*处理堵转*/
void DONE_WORK(shoot_t * shoot);
#endif


 

