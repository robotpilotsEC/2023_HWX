#if 0
#ifndef __S_FUNCTION_H
#define __S_FUNCTION_H

//#include "POTOCAL.h"


float Limit_Target(float tar);
void  Judge_Dir(motor_t *motor);
float Half_Turn(float angle,float max);
void  MID_RESET(motor_t *motor, int16_t MID, uint16_t data);

float PID_CAL(pid_ctrl_t *out, 
							pid_ctrl_t *inn, 
              float meas1, float meas2,
	            char err_cal_mode);


float UP_FX(float x);
float DO_FX(float x);			
				
float FRI_FX(float x,int max);
	
float SF(float t,float *slopeFilter,float res);
float SF_2(float t,float *slopeFilter,float res,int length);//滑动滤波；		

int Get_Symbol(float num);//获取±性		
float JUDGE_NULL(float last, float now);			

bool Motor_Stucking(motor_t *motor);							
#endif


#endif //  0 