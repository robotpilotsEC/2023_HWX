#ifndef __CAR_H
#define __CAR_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define S1_DOWN      (rc_structure.base_info->s1.value == 2)
#define S1_MIDDLE    (rc_structure.base_info->s1.value == 3)
#define S1_UP        (rc_structure.base_info->s1.value == 1)

#define S2_DOWN      (rc_structure.base_info->s2.value == 2)
#define S2_MIDDLE    (rc_structure.base_info->s2.value == 3)
#define S2_UP        (rc_structure.base_info->s2.value == 1)


/* 车行动模式枚举 */
typedef enum
{
    offline_CAR,        //离线模式
    init_CAR,           //初始化模式
    machine_CAR,        //机械模式
    follow_CAR,         //跟随模式
    spin_CAR,           //小陀螺模式
	vision_CAR,			//视觉、比赛模式
	two_CAR,		    //调整双头
	patrol_CAR,         //巡逻模式
	aim_CAR,            //测试自瞄
	navigation_CAR,     //导航模式
	shake_CAR,
} car_move_mode_e;

/* 车控制模式枚举 */
typedef enum
{
    RC_CAR,             //遥控器控制
    MINIPC_CAR,         //小电脑控制
	TEST_CAR,           //测试模式
} car_ctrl_mode_e;

/* 车配置枚举 */
typedef struct
{
    uint8_t reserve;
} car_config_t;


typedef struct
{
	car_ctrl_mode_e ctrl;
	car_move_mode_e mode;
	
} car_t;




void MODE_INIT(void);
void MODE_CHECK(void);







#endif
