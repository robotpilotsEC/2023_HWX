#include "cap_protocol.h"
//#include "judge_sensor.h"
#include "drv_can.h"
cap_send_data_t    cap_send_data;
cap_receive_data_t cap_receive_data;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

int16_t can_data_0x2E[4] = {0};
int16_t can_data_0x2F[4] = {0};

//extern judge_sensor_t	judge_sensor;
//void set_message()
//{
//    cap_send_data.chassis_power_buffer = judge_sensor.info->power_heat_data.chassis_power_buffer;
//    cap_send_data.chassis_power_limit = judge_sensor.info->game_robot_status.chassis_power_limit;
//    cap_send_data.chassis_volt = judge_sensor.info->power_heat_data.chassis_volt;	//**
//    cap_send_data.chassis_current = judge_sensor.info->power_heat_data.chassis_current;	//**
//    cap_send_data.output_power_limit = 300;
//    cap_send_data.input_power_limit = 150;
//    cap_send_data.cap_control.bit.cap_switch = 1;
//    cap_send_data.cap_control.bit.cap_record = 0;
//	
//    if(judge_sensor.info->game_status.game_progress == 4)	//***
//    {
//    	cap_send_data.cap_control.bit.gamegoing = 1;
//	}
//	else
//	{
//		cap_send_data.cap_control.bit.gamegoing = 0;
//	}
//}

//裁判系统发送功率缓冲的频率为50Hz，该函数需要每20ms调用一次。
void can_send_0x2E()
{
    can_data_0x2E[0] = cap_send_data.chassis_power_buffer;    //底盘功率缓冲，0~60J
    can_data_0x2E[1] = cap_send_data.chassis_volt;    	//底盘输出电压 单位 毫伏 **
    can_data_0x2E[2] = cap_send_data.chassis_current;    //底盘输出电流 单位 毫安 **
    CAN_SendData(&hcan1, (uint32_t)0x2E, can_data_0x2E);
}

void can_send_0x2F()
{
    can_data_0x2F[0] = cap_send_data.chassis_power_limit;   //底盘功率限制上限，0~120W
    can_data_0x2F[1] = cap_send_data.output_power_limit;    //电容放电功率限制，-120~300W
    can_data_0x2F[2] = cap_send_data.input_power_limit;    //电容充电功率限制，0~150W
    can_data_0x2F[3] = cap_send_data.cap_control.all;            //电容开关，0（关闭）、1（开启）
    CAN_SendData(&hcan1, (uint32_t)0x2F, can_data_0x2F);
}

void CAN_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    if(canId == 0x30)
    {
        cap_receive_data.cap_Ucr = int16_to_float(((uint16_t)rxBuf[0] << 8| rxBuf[1]), 32000, -32000, 30, 0);
        cap_receive_data.cap_I = int16_to_float(((uint16_t)rxBuf[2] << 8| rxBuf[3]), 32000, -32000, 20, -20);
        cap_receive_data.cap_state.state = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);        
    }
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
    int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;   //加0.5使向下取整变成四舍五入
    return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
    float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
    return b;
}





