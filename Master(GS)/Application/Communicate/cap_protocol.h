#ifndef __CAP_PROTOCOL_H
#define __CAP_PROTOCOL_H

#include "main.h"

typedef struct
{
    uint16_t chassis_power_buffer;  //底盘功率缓冲，0~60J
    uint16_t chassis_power_limit;   //机器人底盘功率限制上限，0~120W
    int16_t output_power_limit;     //电容放电功率限制，-120~300W
    uint16_t input_power_limit;     //电容充电功率限制，0~150W
    uint16_t chassis_volt;			//底盘输出电压 单位 毫伏 **
	uint16_t chassis_current;		//底盘输出电流 单位 毫安 **
    union{
        uint16_t all;
        struct
        {
            uint16_t cap_switch : 1;    //电容开关
            uint16_t cap_record : 1;    //oled记录功能开关
            uint16_t gamegoing : 1;     //比赛进行中为1，否则为0 ***
        }bit;
    }cap_control;
}cap_send_data_t;

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

extern int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
extern float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);
void can_send_0x2E(void);
void can_send_0x2F(void);
void set_message(void);

#endif

