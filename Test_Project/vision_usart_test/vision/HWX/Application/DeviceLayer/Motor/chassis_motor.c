/**
 * @file        chassis_motor.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       Chassis Motor(RM3508).
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(24-October-2021)
 *                  1.将init与update函数定义在本文件中
 *              v1.1.1(8-November-2021)
 *                  1.更新驱动函数
 */
 
/* Includes ------------------------------------------------------------------*/
#include "chassis_motor.h"

#include "can_protocol.h"
#include "rm_protocol.h"
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void chassis_motor_init(chassis_motor_t *motor);
static void chassis_motor_update(chassis_motor_t *motor, uint8_t *rxBuf);
static void chassis_motor_check(chassis_motor_t *motor);
static void chassis_motor_heart_beat(chassis_motor_t *motor);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 底盘电机驱动
drv_can_t		chassis_motor_driver[] = {
	[CHAS_LF] = {
		.id = DRV_CAN1,
		.rx_id = CHASSIS_CAN_ID_LF,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 2,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
        .manual_tx = CAN_ManualTx
	},
	[CHAS_RF] = {
		.id = DRV_CAN1,
		.rx_id = CHASSIS_CAN_ID_RF,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 2,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
        .manual_tx = CAN_ManualTx
	},
	[CHAS_LB] = {
		.id = DRV_CAN1,
		.rx_id = CHASSIS_CAN_ID_LB,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 2,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
        .manual_tx = CAN_ManualTx
	},
	[CHAS_RB] = {
		.id = DRV_CAN1,
		.rx_id = CHASSIS_CAN_ID_RB,
//		.tx_id = RM3508_GetTxId,
//		.data_idx = RM3508_GetDataId,
        .tx_period = 2,
		.add_msg = CAN_AddMsg,
        .add_byte = CAN_AddByte,
        .add_halfword = CAN_AddHalfWord,
        .add_word = CAN_AddWord,
        .manual_tx = CAN_ManualTx
	},
};

// 底盘电机信息
chassis_motor_info_t 	chassis_motor_info[] = {
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
	{
        .init_flag = false,
		.offline_max_cnt = 50,
	},
};

// 底盘电机传感器
chassis_motor_t		chassis_motor[] = {
	[CHAS_LF] = {
		.info = &chassis_motor_info[CHAS_LF],
		.driver = &chassis_motor_driver[CHAS_LF],
		.init = chassis_motor_init,
		.update = chassis_motor_update,
		.check = chassis_motor_check,
		.heart_beat = chassis_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS_LF,
	},
	[CHAS_RF] = {
		.info = &chassis_motor_info[CHAS_RF],
		.driver = &chassis_motor_driver[CHAS_RF],
		.init = chassis_motor_init,
		.update = chassis_motor_update,
		.check = chassis_motor_check,
		.heart_beat = chassis_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS_RF,
	},
	[CHAS_LB] = {
		.info = &chassis_motor_info[CHAS_LB],
		.driver = &chassis_motor_driver[CHAS_LB],
		.init = chassis_motor_init,
		.update = chassis_motor_update,
		.check = chassis_motor_check,
		.heart_beat = chassis_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS_LB,
	},
	[CHAS_RB] = {
		.info = &chassis_motor_info[CHAS_RB],
		.driver = &chassis_motor_driver[CHAS_RB],
		.init = chassis_motor_init,
		.update = chassis_motor_update,
		.check = chassis_motor_check,
		.heart_beat = chassis_motor_heart_beat,
		.work_state = DEV_OFFLINE,
		.id = DEV_ID_CHASSIS_RB,
	},
};

/* Private functions ---------------------------------------------------------*/
static void chassis_motor_init(chassis_motor_t *motor)
{
	drv_can_t *drv_can = motor->driver;
	
	motor->info->offline_cnt = motor->info->offline_max_cnt+1;
	motor->work_state = DEV_OFFLINE;
	
	motor->errno = NONE_ERR;
	if(motor->id == DEV_ID_CHASSIS_LF) {
		drv_can->data_idx = RM3508_GetDataId(drv_can);
		drv_can->tx_id = RM3508_GetTxId(drv_can);
	}
	else if(motor->id == DEV_ID_CHASSIS_RF) {
		drv_can->data_idx = RM3508_GetDataId(drv_can);
		drv_can->tx_id = RM3508_GetTxId(drv_can);
	}
	else if(motor->id == DEV_ID_CHASSIS_LB) {
		drv_can->data_idx = RM3508_GetDataId(drv_can);
		drv_can->tx_id = RM3508_GetTxId(drv_can);	
	}
	else if(motor->id == DEV_ID_CHASSIS_RB) {
		drv_can->data_idx = RM3508_GetDataId(drv_can);
		drv_can->tx_id = RM3508_GetTxId(drv_can);		
	}
	else {
		motor->errno = DEV_ID_ERR;
	}
}

static void chassis_motor_update(chassis_motor_t *chas_motor, uint8_t *rxBuf)
{
	chassis_motor_info_t *motor_info = chas_motor->info;
	
	motor_info->angle = RM3508_GetMotorAngle(rxBuf);
	motor_info->speed = RM3508_GetMotorSpeed(rxBuf);
	motor_info->current = RM3508_GetMotorCurrent(rxBuf);
	motor_info->temperature = RM3508_GetMotorTemperature(rxBuf);
    
	motor_info->offline_cnt = 0;
}

static void chassis_motor_check(chassis_motor_t *motor)
{
	int16_t err;
	chassis_motor_info_t *motor_info = motor->info;
	
	/* 未初始化 */
	if( !motor_info->init_flag )
	{
		motor_info->init_flag = true;
		motor_info->angle_prev = motor_info->angle;
		motor_info->angle_sum = 0;
	}
	
	err = motor_info->angle - motor_info->angle_prev;
	
	/* 过零点 */
	if(abs(err) > 4095)
	{
		/* 0↓ -> 8191 */
		if(err >= 0)
			motor_info->angle_sum += -8191 + err;
		/* 8191↑ -> 0 */
		else
			motor_info->angle_sum += 8191 + err;
	}
	/* 未过零点 */
	else
	{
		motor_info->angle_sum += err;
	}
	
	motor_info->angle_prev = motor_info->angle;		
}

static void chassis_motor_heart_beat(chassis_motor_t *motor)
{
	chassis_motor_info_t *motor_info = motor->info;
	
	motor_info->offline_cnt++;
	if(motor_info->offline_cnt > motor_info->offline_max_cnt) {
		motor_info->offline_cnt = motor_info->offline_max_cnt;
		motor->work_state = DEV_OFFLINE;
	}
	else {
		if(motor->work_state == DEV_OFFLINE)
			motor->work_state = DEV_ONLINE;
	}
}

/* Exported functions --------------------------------------------------------*/

