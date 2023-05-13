#ifndef __6020_MOTOR_H
#define __6020_MOTOR_H

//#include "stm32f4xx_hal.h"
//#include "config.h"
//#include "pid.h"
//#include "drv_can.h"

//typedef struct 
//{
//	int16_t		angle;						//0~8191
//	int16_t 	speed;						//RPM
//	int16_t 	current;					//
//	int16_t		temperature;			//°„C
//	int16_t		angle_add;				//-4096~4096
//	int32_t		angle_sum;				//-2147683647~2147683647
//}motor_6020_base_info_t;

//typedef struct
//{
//	uint8_t offline_cnt_max;
//	uint8_t offline_cnt;
//	uint8_t status;
//	int32_t target_angle_sum;
//}motor_6020_info_t;

//typedef struct motor_6020_t
//{
//	motor_6020_base_info_t *base_info;
//	motor_6020_info_t      *info;
//	pid_t                  *motor_pid_speed;
//	pid_t                  *imu_pid_speed;
//	pid_t                  *motor_pid_angle;
//	pid_t                  *imu_pid_angle;
//	drv_can_t              *can;
//	void                  (*init)(struct motor_6020_t *motor);
//	void                  (*update)(struct motor_6020_t *motor, uint8_t *rxBuf);
//}motor_6020_t;

//void motor_6020_init(motor_6020_t *motor);
//void motor_6020_info_init(motor_6020_info_t *info);
//void motor_6020_update(motor_6020_t *motor, uint8_t *rxBuf);

#endif
