#ifndef __JUDGE_SENSOR_H
#define __JUDGE_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

//__ALGO_H
#include "pid.h"
#include "rp_math.h"
#include "kalman.h"
#include "crc.h"

//#include "DRIVE.h"

#include "drv_io.h"
#include "drv_tim.h"
#include "drv_can.h"
#include "soft_iic.h"
#include "drv_uart.h"
#include "drv_adda.h"
#include "drv_haltick.h"
#include "judge_infantrypotocol.h"


#define JUDGE_ONLINE   judge_sensor.work_state == DEV_ONLINE
#define JUDGE_OFFLINE  judge_sensor.work_state == DEV_OFFLINE

#define SHOOT_SPEED    judge_info.shoot_data.bullet_speed

#define SHOOT_SPEED_LIMIT       judge_info.game_robot_status.shooter_id1_17mm_speed_limit
#define SHOOT_SPEED_LIMIT_ADD   judge_info.game_robot_status.shooter_id2_17mm_speed_limit

#define BARREL_SPEED_1   judge_info.game_robot_status.shooter_id1_17mm_speed_limit
#define BARREL_SPEED_2   judge_info.game_robot_status.shooter_id2_17mm_speed_limit

#define COOLING_HEAT        judge_sensor.info->power_heat_data.shooter_id1_17mm_cooling_heat
#define COOLING_HEAT_ADD    judge_sensor.info->power_heat_data.shooter_id2_17mm_cooling_heat

#define COOLING_HEAT_LIMIT      judge_sensor.info->game_robot_status.shooter_id1_17mm_cooling_limit
#define COOLING_HEAT_LIMIT_ADD  judge_sensor.info->game_robot_status.shooter_id2_17mm_cooling_limit


//#if CARR_MODE == 0 || CARR_MODE == 1 || CARR_MODE == 2 || CARR_MODE == 4

  #define COOLING_HEAT_LOW ((COOLING_HEAT_LIMIT - COOLING_HEAT) <  50)

	#define COOLING_HEAT_ALLOW      ((COOLING_HEAT_LIMIT - COOLING_HEAT) > 20)

//#endif



#define BARREL_2_ONLINE    (SHOOT_SPEED_LIMIT_ADD && COOLING_HEAT_LIMIT_ADD)

#define COOLING_HEAT_OK_1 ((((COOLING_HEAT_LIMIT - COOLING_HEAT)     >=  40) && State.Barrel_Wise == CLOCK     && BARREL_2_ONLINE))
#define COOLING_HEAT_OK_2 ((((COOLING_HEAT_LIMIT - COOLING_HEAT_ADD) >=  40) && State.Barrel_Wise == ANTICLOCK && BARREL_2_ONLINE))

#define BARREL_1_ALLOW      ((COOLING_HEAT_LIMIT - COOLING_HEAT) > 20 && State.Barrel_Wise == ANTICLOCK && BARREL_2_ONLINE)
#define BARREL_2_ALLOW      ((COOLING_HEAT_LIMIT - COOLING_HEAT_ADD) > 20 && State.Barrel_Wise == CLOCK && BARREL_2_ONLINE)

//#if CARR_MODE == 3

	#define COOLING_HEAT_LOW_1 (((COOLING_HEAT_LIMIT - COOLING_HEAT) <=  50 && State.Barrel_Wise == ANTICLOCK && BARREL_2_ONLINE))
	#define COOLING_HEAT_LOW_2 (((COOLING_HEAT_LIMIT - COOLING_HEAT_ADD) <=  50 && State.Barrel_Wise == CLOCK && BARREL_2_ONLINE))

  #define HEAT_LOW_OFFLINE   (((COOLING_HEAT_LIMIT - COOLING_HEAT) < 50) && !BARREL_2_ONLINE)
  #define HEAT_ALLOW_OFFLINE (((COOLING_HEAT_LIMIT - COOLING_HEAT) > 20) && !BARREL_2_ONLINE)
	
//	#define COOLING_HEAT_LOW (COOLING_HEAT_LOW_1 || COOLING_HEAT_LOW_2 || HEAT_LOW_OFFLINE)

//	#define COOLING_HEAT_ALLOW  (BARREL_1_ALLOW || BARREL_2_ALLOW || HEAT_ALLOW_OFFLINE)

//#endif


extern judge_info_t judge_info;
extern judge_sensor_t judge_sensor;

/* Exported functions --------------------------------------------------------*/

#endif
