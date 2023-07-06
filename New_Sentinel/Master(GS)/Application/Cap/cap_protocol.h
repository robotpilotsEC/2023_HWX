#ifndef __CAP_PROTOCOL_H
#define __CAP_PROTOCOL_H

#include "stm32f4xx_hal.h"


typedef struct
{
    float cap_Ucr;    //电容两端电压Ucr，0~30V
    float cap_I;    //电容电流I，-20~20A
    union
    {
        uint16_t state;     //电容状态
        struct
        {
            uint16_t warning : 1;   //报警
            uint16_t cap_U_over : 1;    //电容过压
            uint16_t cap_I_over : 1;    //电容过流
            uint16_t cap_U_low : 1;     //电容欠压
            uint16_t bat_I_low : 1;     //裁判系统欠压
            uint16_t can_receive_miss : 1;    //电容未接收到CAN通信数据
        }bit;
    }cap_state;
	
}cap_receive_data_t;

extern cap_receive_data_t cap_receive_data;

void cap_send_2E(void);
void cap_send_2F(void);
void cap_update(uint8_t *rxbuf);
int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);

#endif

