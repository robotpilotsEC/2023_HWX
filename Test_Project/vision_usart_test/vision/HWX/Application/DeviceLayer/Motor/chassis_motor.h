/**
 * @file        chassis_motor.h
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

#ifndef __CHASSIS_MOTOR_H
#define __CHASSIS_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct chassis_motor_info_struct {
	volatile uint16_t	 angle;
	volatile int16_t	 speed;
	volatile int16_t	 current;
  volatile uint8_t   temperature;
	volatile uint16_t	 angle_prev;
	volatile int32_t	 angle_sum;
	volatile bool	     init_flag;
	volatile uint8_t   offline_cnt;
	const    uint8_t	 offline_max_cnt;	
} chassis_motor_info_t;

typedef struct chassis_motor_struct {
	chassis_motor_info_t        *info;
	drv_can_t			        *driver;
	void				        (*init)(struct chassis_motor_struct *self);
	void				        (*update)(struct chassis_motor_struct *self, uint8_t *rxBuf);
	void				        (*check)(struct chassis_motor_struct *self);	
	void					    (*heart_beat)(struct chassis_motor_struct *self);
	volatile dev_work_state_t   work_state;
	volatile dev_errno_t		errno;
	const    dev_id_t			id;
} chassis_motor_t;

extern chassis_motor_t	chassis_motor[CHAS_MOTOR_CNT];

/* Exported functions --------------------------------------------------------*/


#endif
