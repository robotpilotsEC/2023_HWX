/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.h   
  * @Version    : v1.2		
  * @Author     : hwx			
  * @Date       : 2022-11-08         
  * @Description: 基本的底盘包，后续有待增加 1.直线修正 2.旋转量占比限制
  *
  *               目前无用变量：底盘状态            -- chassis_work_sate
  *                             旋转量占比         -- chassis_cycle_div_front
  *								底盘最大输出       -- chassis_output_max
  *
  *               目前无用函数：修改旋转量占比函数  -- Chassis_Config_cycle_div_front
  *					
  *				  有待更改的部分：最终发送电流的方式还要去到can的驱动层的函数，希望可以直接用电机包的函数
  *                               
  *
  ******************************************************************************
 */
  /*版本更新说明
  * V1.1 解决了函数名称错误导致的编译报错,有待解决由于驱动不同导致的can发送问题
  * V1.2 解决了一系列问题
  * */
/*******************************************************************************
*                               底盘包使用教程
* @Tips   : 在使用底盘包之前请使用电机包注册好四个底盘电机
* @Tips   : 事先定义好了底盘包的结构体 “Chassis”，但需要用户完成初始化
* @Tips   : 定义一个Chassis_InitTypeDef结构体，并完成结构体成员赋值
* @Tips   : 然后调用函数 Chassic_init 或 使用结构体成员.init 完成底盘注册
* @Tips   : 接下来就可以直接调用 Chassis_Work 或 使用结构体成员.work 让底盘工作
* @Tips   : Config函数可以在中途调用，修改底盘工作配置
* @Tips   : 
*
*******************************************************************************/

/*******************************************************************************
*                               底盘包使用注意事项
* @Tips   : 需要传入注册好的电机（主要使用到PID与）
* @Tips   : 注意修改宏定义 “CHASSIS_DRV_CAN_USE” 选择与电机通讯所用的外设
* @Tips   : 注意修改宏定义 “CHASSIS_CAN_STD_ID”  修改CAN发送的报文ID
* @Tips   : 记得将裁判系统内容中的底盘功率地址传入初始化结构体
* @Tips   : 若初始化将功率buffer传入，底盘功率限制是默认开启的，
* @Tips   : 若想关闭请调用Chassis_Config_power_limit(POWER_LIMIT_OFF)
*
*******************************************************************************/

#ifndef __RP_CHASSIS_H_
#define __RP_CHASSIS_H_


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

#define SIX_SHOOT_DONE_TIME_MAX   (400)

#define SHOOT_DONE_TIME_MAX    	  (5)

#define SHOOT_MY_SPEED            ((-1)*3000)

#define SHOOT_STRI_SPEED      	  ((-1)*2000)
#define SHOOT_STRI_SPEED_ERR  	  (100)

#define DEAL_DONE_ROUND_TIME   	  (180) //堵转处理时间
#define DEAL_DONE_ROUND_HALF_TIME (90) 


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
#define SPID_STRI_OUTPUT_MAX    8000
#define SPID_STRI_INTEGRA_MAX   3000
#define SPID_STRI_KP            10
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
    S_S_offline,  //失联
    S_S_oneshot,  //单发
    S_S_threeshot,//连发
    S_S_done,     //堵转

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
	
}shoot_flag_t;

/** 
  * @brief  计数器定义
  */ 
typedef struct shoot_cnt_t
{	

	/*计数器位*/
	uint16_t               done_time;        //堵转
	uint16_t               deal_time;
	
}shoot_cnt_t;


/** 
  * @brief  计数器定义
  */ 
typedef struct shoot_base_info_t
{	

	/*标志位*/
	int16_t            shoot_speed;        //摩擦轮转速
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
    motor_3508_t         *fric_left;
    motor_3508_t         *fric_right;
	
	pid_t                *pid_s_left;
	pid_t                *pid_s_righ;
	
	pid_t                *pid_s_stir_sin;
	pid_t                *pid_p_stir_sin;
	
	pid_t                *pid_p_stir_done;
	
	pid_t                *pid_s_stir_run;
	
	/*标志位*/
	shoot_flag_t          flag;
	
	/*计数器*/
	shoot_cnt_t           cnt;
	
	shoot_base_info_t     base;
	

}shoot_t;





/* Exported functions --------------------------------------------------------*/

/*发射机构初始化*/
void SHOOT_INIT(shoot_t * shoot);

/*掉线检测*/
void Shoot_Heart(shoot_t * shoot);

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

#endif


 

