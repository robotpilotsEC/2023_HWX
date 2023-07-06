#ifndef SLAVE_H
#define SLAVE_H

#include "rp_config.h"
#include "rp_math.h"
//#include "ALGO.h"

#include "pid.h"
#include "rp_math.h"
#include "kalman.h"
#include "crc.h"


#include "drv_can.h"
#include "judge_sensor.h"
//#include "imu.h"


typedef __packed struct
{	
	float    chassis_power; 
	uint16_t chassis_power_buffer;
	uint16_t shooter_id1_17mm_cooling_heat;	
	
} slave_pack1_t; 

typedef __packed struct
{
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t chassis_power_limit; 	

	char id;	
  dev_work_state_t	 imu_work_state; 
	
} slave_pack2_t; 

typedef __packed struct
{
	int16_t slave_pit;	
	int16_t slave_roll;

	float bullet_speed;	

} slave_pack3_t;

typedef __packed struct
{
	int16_t slave_pit;	
	int16_t slave_roll;
	dev_work_state_t	 imu_work_state; 
	
} slave_pack4_t;

//global_fiction,global_clip,global_spin,global_auto_aim
typedef __packed struct
{
	char fiction;
	char clip;
	char spin;
	char auto_aim;
	
	char auto_arrive;
	char move_mode;	
	char barrel_type;
} slave_pack5_t;

typedef __packed struct
{
	uint16_t shooter_id2_17mm_cooling_heat;		
	uint16_t shooter_id2_17mm_cooling_limit;	
	uint16_t shooter_id2_17mm_speed_limit;	
	
} slave_pack6_t;




typedef struct{

	slave_pack1_t pack1;
	slave_pack2_t pack2;
	slave_pack3_t pack3;
	
	slave_pack4_t pack4;
	slave_pack5_t pack5;
	slave_pack6_t pack6;
	
  uint16_t		       offline_cnt;
  uint16_t		       offline_max_cnt;	
  dev_work_state_t	 work_state; 
	
}slave_t;



void slave_heart_beat(void);
void Up_RX(uint32_t ID, uint8_t *date);
void Down_Send(judge_sensor_t *judge);
void Get_Rudder(void);
void Up_Send(void);
void Down_RX(uint32_t ID, uint8_t *date);

extern slave_t SLAVE;


















#endif






