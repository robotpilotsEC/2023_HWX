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

#define PI (3.1415926)


/* ���ж�ģʽö�� */
typedef enum
{
    offline_CAR,        //����ģʽ
    init_CAR,           //��ʼ��ģʽ
    machine_CAR,        //��еģʽ
    follow_CAR,         //����ģʽ
    spin_CAR,           //С����ģʽ
	vision_CAR,			//�Ӿ�������ģʽ
	two_CAR,		    //����˫ͷ
	patrol_CAR,         //Ѳ��ģʽ
	aim_CAR,            //��������
	navigation_CAR,     //����ģʽ
	shake_CAR,
} car_move_mode_e;

/* ������ģʽö�� */
typedef enum
{
    RC_CAR,             //ң��������
    MINIPC_CAR,         //С���Կ���
	TEST_CAR,           //����ģʽ
} car_ctrl_mode_e;

/* ������ö�� */
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
