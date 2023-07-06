#ifndef __CAR_H
#define __CAR_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* ���ж�ģʽö�� */
typedef enum
{
    offline_CAR,        //����ģʽ
    init_CAR,           //��ʼ��ģʽ
    machine_CAR,        //��еģʽ
    follow_CAR,
    spin_CAR,           //С����ģʽ
	vision_CAR,
} car_move_mode_e;

/* ������ģʽö�� */
typedef enum
{
    RC_CAR,             //ң��������
    KEY_CAR,            //���̿���
} car_ctrl_mode_e;

/* ������ö�� */
typedef struct
{
    uint8_t reserve;
} car_config_t;


typedef struct
{
    car_config_t *config;  //��������

    uint8_t move_mode_commond;  //�ƶ�ģʽ����
    uint8_t move_mode_status;   //�ƶ�ģʽ״̬
    uint8_t ctrl_mode;          //����ģʽ
    uint8_t last_mode;
	
	uint16_t f_times;
} car_t;





void MODE_CHECK(void);



#define PI (3.1415926)



#endif
