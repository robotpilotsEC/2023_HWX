#include "Power_Limit.h"
#include "judge_infantrypotocol.h"
#include "judge_sensor.h"


/*-祖传功率-*/
void Chassis_Motor_Power_Limit(int16_t *data)
{
	float buffer = judge_info.power_heat_data.chassis_power_buffer;
	float heat_rate,Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;
	
	uint16_t OUT_MAX = 0;
	
	OUT_MAX = POWER_PID_O_MAX * 4;
	
	if(buffer > 60)buffer = 60;//防止飞坡之后缓冲250J变为正增益系数
	
	Limit_k = buffer / 60;
	
	if(buffer < 25)
		Limit_k = Limit_k * Limit_k ;// * Limit_k; //3方
	else
		Limit_k = Limit_k;// * str->Limit_k; //平方
	
	if(buffer < 60)
		CHAS_LimitOutput = Limit_k * OUT_MAX;
	else 
		CHAS_LimitOutput = OUT_MAX;    
	
	CHAS_TotalOutput = abs(data[0]) + abs(data[1]) + abs(data[2]) + abs(data[3]) ;
	
	heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;
	
  if(CHAS_TotalOutput >= CHAS_LimitOutput)
  {
		for(char i = 0 ; i < 4 ; i++)
		{	
			data[i] = (int16_t)(data[i] * heat_rate);	
		}
	}
}


/*-祖传功率-*/
void my_Chassis_Motor_Power_Limit(int16_t *data , float buffer)
{
	float heat_rate,Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;
	
	uint16_t OUT_MAX = 0;
	
	OUT_MAX = POWER_PID_O_MAX * 4;
	
	if(buffer > 60)buffer = 60;//防止飞坡之后缓冲250J变为正增益系数
	
	Limit_k = buffer / 60;
	
	if(buffer < 25)
		Limit_k = Limit_k * Limit_k ;// * Limit_k; //3方
	else
		Limit_k = Limit_k;// * str->Limit_k; //平方
	
	if(buffer < 60)
		CHAS_LimitOutput = Limit_k * OUT_MAX;
	else 
		CHAS_LimitOutput = OUT_MAX;    
	
	CHAS_TotalOutput = abs(data[0]) + abs(data[1]) + abs(data[2]) + abs(data[3]) ;
	
	heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;
	
  if(CHAS_TotalOutput >= CHAS_LimitOutput)
  {
		for(char i = 0 ; i < 4 ; i++)
		{	
			data[i] = (int16_t)(data[i] * heat_rate);	
		}
	}
}
