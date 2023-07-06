/**
 * @file        chassis.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        23-October-2020
 * @brief       Chassis Module.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "chassis.h"

#include "can_protocol.h"
#include "rp_math.h"
#include "pid.h"
#include "kalman.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void CHASSIS_Init(void);
void CHASSIS_Ctrl(void);
void CHASSIS_Test(void);
void CHASSIS_SelfProtect(void);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// 底盘电机本地驱动
drv_can_t				*chas_drv[CHAS_MOTOR_CNT];
chassis_motor_t			*chas_motor[CHAS_MOTOR_CNT];
chassis_motor_info_t	*chas_motor_info[CHAS_MOTOR_CNT];

/* Exported variables --------------------------------------------------------*/
// 底盘电机PID控制器
chassis_motor_pid_t 	chas_motor_pid[CHAS_MOTOR_CNT] = {
	[CHAS_LF] = {
		.speed.kp = 11.5,
		.speed.ki = 0.472,
		.speed.kd = 0,
		.speed.integral_max = 18000,
		.speed.out_max = 10000,
		.angle.kp = 0.55,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
	[CHAS_RF] = {
		.speed.kp = 11.5,
		.speed.ki = 0.472,
		.speed.kd = 0,
		.speed.integral_max = 18000,
		.speed.out_max = 10000,
		.angle.kp = 0.55,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
	[CHAS_LB] = {
		.speed.kp = 11.5,
		.speed.ki = 0.472,
		.speed.kd = 0,
		.speed.integral_max = 18000,
		.speed.out_max = 10000,
		.angle.kp = 0.55,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
	[CHAS_RB] = {
		.speed.kp = 11.5,
		.speed.ki = 0.472,
		.speed.kd = 0,
		.speed.integral_max = 18000,
		.speed.out_max = 10000,
		.angle.kp = 0.55,
		.angle.ki = 0,
		.angle.kd = 0.008,
		.angle.integral_max = 0,
		.angle.out_max = 9000,
	},
};

// 底盘模块控制器
chassis_ctrl_t		chas_ctrl = {
	.motor = &chas_motor_pid,
};

// 底盘模块传感器
chassis_dev_t		chas_dev = {
	.chas_motor[CHAS_LF] = &chassis_motor[CHAS_LF],
	.chas_motor[CHAS_RF] = &chassis_motor[CHAS_RF],
	.chas_motor[CHAS_LB] = &chassis_motor[CHAS_LB],
	.chas_motor[CHAS_RB] = &chassis_motor[CHAS_RB],
	.imu_sensor = &imu_sensor,
	.rc_sensor = &rc_sensor,
};

// 底盘模块信息
chassis_info_t 	chas_info = {
	.remote_mode = RC,
	.local_mode = CHASSIS_MODE_NORMAL,
};

chassis_t chassis = {
	.controller = &chas_ctrl,
	.dev = &chas_dev,
	.info = &chas_info,
	.init = CHASSIS_Init,
	.test = CHASSIS_Test,
	.ctrl = CHASSIS_Ctrl,
	.self_protect = CHASSIS_SelfProtect,
};

/* Private functions ---------------------------------------------------------*/
/* 驱动层 --------------------------------------------------------------------*/
/**
 *	@brief	底盘电机PID参数初始化
 */
void CHASSIS_PidParamsInit(chassis_motor_pid_t *pid, uint8_t motor_cnt)
{
	for(uint8_t i = 0; i < motor_cnt; i++) {
		pid_val_init(&pid[i].speed);
		pid[i].out = 0;
	}
}

/**
 *	@brief	底盘电机卸力
 */
static void CHASSIS_Stop(chassis_motor_pid_t *pid)
{
	for(uint8_t i=0; i<CHAS_MOTOR_CNT; i++)
	{
		pid[i].out = 0;
		chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].out);
	}
}

/**
 *	@brief	底盘电机PID输出
 */
static void CHASSIS_PidOut(chassis_motor_pid_t *pid)
{
	for(uint8_t i=0; i<1; i++) {
		if(chas_motor[i]->work_state == DEV_ONLINE) {
            chas_drv[i]->add_halfword(chas_drv[i], (int16_t)pid[i].out);
		} 
		else {
            chas_drv[i]->add_halfword(chas_drv[i], 0);
		}
	}    
}

/**
 *	@brief	底盘电机速度环
 */
static void CHASSIS_Speed_PidCalc(chassis_motor_pid_t *pid, chassis_motor_cnt_t MOTORx)
{
	pid[MOTORx].speed.err = pid[MOTORx].speed.target - pid[MOTORx].speed.measure;
	single_pid_ctrl(&pid[MOTORx].speed);
	pid[MOTORx].out = pid[MOTORx].speed.out;
}

/* Exported functions --------------------------------------------------------*/
/* 信息层 --------------------------------------------------------------------*/
/**
 *	@brief	底盘获取系统信息
 */
void CHASSIS_GetSysInfo(void)
{
	/*----控制方式修改----*/
	if(sys.remote_mode == RC) {
		chas_info.remote_mode = RC;
	}
	else if(sys.remote_mode == KEY) {
		chas_info.remote_mode = KEY;
	}
	
	/*----本地模式修改----*/
}

void CHASSIS_GetRcInfo(void)
{
}

void CHASSIS_UpdateController(void)
{
	chas_motor_pid[CHAS_LF].speed.measure = chas_motor_info[CHAS_LF]->speed;
	chas_motor_pid[CHAS_RF].speed.measure = chas_motor_info[CHAS_RF]->speed;
	chas_motor_pid[CHAS_LB].speed.measure = chas_motor_info[CHAS_LB]->speed;
	chas_motor_pid[CHAS_RB].speed.measure = chas_motor_info[CHAS_RB]->speed;	
}

/* 应用层 --------------------------------------------------------------------*/
/* 任务层 --------------------------------------------------------------------*/
void CHASSIS_Init(void)
{
	chas_drv[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->driver;
	chas_drv[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->driver;
	chas_drv[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->driver;
	chas_drv[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->driver;

	chas_motor[CHAS_LF] = chas_dev.chas_motor[CHAS_LF];
	chas_motor[CHAS_RF] = chas_dev.chas_motor[CHAS_RF];
	chas_motor[CHAS_LB] = chas_dev.chas_motor[CHAS_LB];
	chas_motor[CHAS_RB] = chas_dev.chas_motor[CHAS_RB];	
	
	chas_motor_info[CHAS_LF] = chas_dev.chas_motor[CHAS_LF]->info;
	chas_motor_info[CHAS_RF] = chas_dev.chas_motor[CHAS_RF]->info;
	chas_motor_info[CHAS_LB] = chas_dev.chas_motor[CHAS_LB]->info;
	chas_motor_info[CHAS_RB] = chas_dev.chas_motor[CHAS_RB]->info;
	
}

void CHASSIS_GetInfo(void)
{
	CHASSIS_GetSysInfo();
	CHASSIS_GetRcInfo();
	CHASSIS_UpdateController();
}

void CHASSIS_SelfProtect(void)
{
	CHASSIS_Stop(chas_motor_pid);
	CHASSIS_PidParamsInit(chas_motor_pid, CHAS_MOTOR_CNT);
	CHASSIS_GetInfo();
}

void CHASSIS_PidCtrl(void)
{
	// 底盘电机速度环
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_LF);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_LB);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_RF);
	CHASSIS_Speed_PidCalc(chas_motor_pid, CHAS_RB);
	
	// 底盘电机输出响应
	CHASSIS_PidOut(chas_motor_pid);
}

void CHASSIS_RcCtrl(void)
{
	
}

void CHASSIS_KeyCtrl(void)
{

}

void CHASSIS_Ctrl(void)
{
	/*----信息读入----*/
	CHASSIS_GetInfo();
	/*----期望修改----*/ 
	if(chas_info.remote_mode == RC) {
		CHASSIS_RcCtrl();
	}
	else if(chas_info.remote_mode == KEY) {
		CHASSIS_KeyCtrl();
	}
	/*----最终输出----*/
	CHASSIS_PidCtrl();	
}

void CHASSIS_Test(void)
{
    
}
