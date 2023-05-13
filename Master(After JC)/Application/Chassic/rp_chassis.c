/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022, hwx, China, SZU.
  *                            N0   Rights  Reserved
  *                              
  *                   
  * @FileName   : rp_chassis.c   
  * @Version    : v1.1		
  * @Author     : hwx			
  * @Date       : 2022-11-06         
  * @Description:    
  *
  *
  ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "rp_chassis.h"
#include "remote.h"
#include "judge.h"

/* Exported variables --------------------------------------------------------*/

chassis_t Chassis = {
	
	.init      = Chassis_Init,
	.work      = Chassis_Work,
	
	.config_rotation_ratio  = Chassis_Config_rotation_ratio,
	.config_power_limit     = Chassis_Config_power_limit,
	.config_top_calc        = Chassis_config_top_calc,

};
extern motor_3508_t           motor_3508_LF_structure;
extern motor_3508_base_info_t motor_3508_LF_base_info;
extern motor_3508_info_t      motor_3508_LF_info;

extern motor_3508_t           motor_3508_RF_structure;
extern motor_3508_base_info_t motor_3508_RF_base_info;
extern motor_3508_info_t      motor_3508_RF_info;

extern motor_3508_t           motor_3508_LB_structure;
extern motor_3508_base_info_t motor_3508_LB_base_info;
extern motor_3508_info_t      motor_3508_LB_info;

extern motor_3508_t           motor_3508_RB_structure;
extern motor_3508_base_info_t motor_3508_RB_base_info;
extern motor_3508_info_t      motor_3508_RB_info;

extern pid_t                  chassis_pid_speed_structure[4];
extern pid_t                  chassis_pid_follow_structure;
extern rc_t                   rc_structure;

/*裁判系统*/
extern judge_t                judge;
/* Function  body --------------------------------------------------------*/
 

void Chassis_Init(chassis_t *Chassis )
{
	
    /* 初始化四个电机 */
	MOTOR_3508_INIT( &motor_3508_LF_structure, &motor_3508_LF_base_info, &motor_3508_LF_info);
	MOTOR_3508_INIT( &motor_3508_RF_structure, &motor_3508_RF_base_info, &motor_3508_RF_info);
	MOTOR_3508_INIT( &motor_3508_LB_structure, &motor_3508_LB_base_info, &motor_3508_LB_info);
	MOTOR_3508_INIT( &motor_3508_RB_structure, &motor_3508_RB_base_info, &motor_3508_RB_info);
	
	/* 初始化PID */
	for(uint8_t i = 0;i < 4 ; i++ )
	{
		PID_struct_init( &chassis_pid_speed_structure[i],POSITION_PID,SPEED_LIM_O,SPEED_LIM_I,SPEED_PID_P,SPEED_PID_I,0);
	}
	
	/*跟随PID*/
	
	PID_struct_init( &chassis_pid_follow_structure,POSITION_PID,3000,0,0.3,0,0);
	
	
	Chassis->motor_LF = &motor_3508_LF_structure;
	Chassis->motor_RF = &motor_3508_RF_structure;
	Chassis->motor_LB = &motor_3508_LB_structure;
	Chassis->motor_RB = &motor_3508_RB_structure;
	
	
	/*裁判系统功率缓冲区*/
	Chassis->work_info.power_limit_buffer = &judge.base_info->chassis_power_buffer;
	
	
	Chassis->work_info.output_max              = CHASSIS_SPEED_MAX;
	Chassis->work_info.speed_max               = CHASSIS_OUTPUT_MAX;
	Chassis->work_info.config.power_limit      = CHASSIS_POWER_LIMIT_OFF;
	Chassis->work_info.config.rotation_ratio   = CHASSIS_CYCLE_DIV_FRONT;
	Chassis->work_info.config.speed_max        = CHASSIS_SPEED_MAX;

}

/**
  * @Name    Chassis_Power_Limit
  * @brief   底盘功率限制(经典祖传算法)
  * @param   底盘 
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void Chassis_Power_Limit(chassis_t * chassis)
{
	if(CHASSIS_POWER_LIMIT_ON)
	{
		int16_t limit_output_current[4];
	
		float buffer = *Chassis.work_info.power_limit_buffer;
		float heat_rate,Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;
		
		limit_output_current[0] = chassis->base_info.output.motor_LF_current;
		limit_output_current[1] = chassis->base_info.output.motor_RF_current;
		limit_output_current[2] = chassis->base_info.output.motor_LB_current;
		limit_output_current[3] = chassis->base_info.output.motor_RB_current;
		
		uint16_t OUT_MAX = 0;
	
		OUT_MAX = CHASSIS_SPEED_MAX * 4;
		
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
		
		CHAS_TotalOutput = c_abs(limit_output_current[0]) + c_abs(limit_output_current[1]) + c_abs(limit_output_current[2]) + c_abs(limit_output_current[3]) ;
		
		heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;
		
	  if(CHAS_TotalOutput >= CHAS_LimitOutput)
	  {
			for(char i = 0 ; i < 4 ; i++)
			{	
				limit_output_current[i] = (int16_t)(limit_output_current[i] * heat_rate);	
			}
		}
		/*重新赋值*/
		chassis->base_info.output.motor_LF_current = limit_output_current[0];
		chassis->base_info.output.motor_RF_current = limit_output_current[1];
		chassis->base_info.output.motor_LB_current = limit_output_current[2];
		chassis->base_info.output.motor_RB_current = limit_output_current[3];
			
	}

}


/**
  * @Name    Chassis_Config_power_limit
  * @brief   开关底盘功率限制（进入自杀模式）
  * @param   off_or_on
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void Chassis_Config_power_limit(uint8_t off_or_on)
{
	if(off_or_on == CHASSIS_POWER_LIMIT_OFF || off_or_on == CHASSIS_POWER_LIMIT_ON)
	{
		Chassis.work_info.config.power_limit = off_or_on;
	}

}

/**
  * @Name    Chassis_Config_cycle_div_front
  * @brief   修改旋转量比重
  * @param   cycle_div_front ( 0 ~ 100)
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void Chassis_Config_rotation_ratio(uint8_t rotation_ratio)
{
	if(rotation_ratio >= 0 || rotation_ratio <= 100)
	{
		Chassis.work_info.config.rotation_ratio = rotation_ratio;
	}
}

/**
  * @Name    Chassis_Config_cycle_div_front
  * @brief   修改旋转量比重
  * @param   cycle_div_front ( 0 ~ 100)
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void Chassis_config_top_calc(uint8_t off_or_on)
{
	if(off_or_on == CHASSIS_TOP_CALC_OFF || off_or_on == CHASSIS_TOP_CALC_ON)
	{
		Chassis.work_info.config.top_cacl = off_or_on;
	}

}

/**
  * @Name    Chassis_Speed_Limit
  * @brief   速度限制（等比例缩小）
  * @param   底盘 三个方向速度
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void Chassis_Speed_Limit(chassis_t *chassis)
{
	int16_t speed_sum;
	float RATE;
	speed_sum = c_abs(chassis->base_info.target.front_speed) + \
	            c_abs(chassis->base_info.target.right_speed) + \
                c_abs(chassis->base_info.target.cycle_speed);
	
	if(speed_sum > chassis->work_info.speed_max)
	{
		RATE = (float)chassis->work_info.speed_max / (float)speed_sum;
	}
	else 
	{
		RATE = 1;
	}
	
	chassis->base_info.target.front_speed = (float)chassis->base_info.target.front_speed * RATE;
	chassis->base_info.target.right_speed = (float)chassis->base_info.target.right_speed * RATE;
	chassis->base_info.target.cycle_speed = (float)chassis->base_info.target.cycle_speed * RATE;
}


/**
  * @Name    Chassis_Speed_Calculating
  * @brief   底盘速度解算（速度分解）
  * @param   三个方向速度
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void Chassis_Speed_Calculating(chassis_t *chassis, int16_t front, int16_t right, int16_t cycle)
{
	
	chassis->base_info.output.motor_LF_speed   = (  front + right + cycle);
	chassis->base_info.output.motor_RF_speed   = (- front + right + cycle);
	chassis->base_info.output.motor_LB_speed   = (  front - right + cycle);
	chassis->base_info.output.motor_RB_speed   = (- front - right + cycle);
}





/**
  * @Name    Chassis_Top_Speed_Calculating
  * @brief   底盘小陀螺速度解算(将这个函数放在Work前)
  * @param   底盘 三个方向速度 差角（请换算成弧度制，并且注意数据类型是float）
  * @retval 
  * @author  HWX
  * @Date    2022-11-06
**/
void Chassis_Top_Speed_Calculating(chassis_t *chassis)
{
	//if(chassis->work_info.config.top_cacl == CHASSIS_TOP_CALC_ON)
	//{
	  float detal_angle = chassis->base_info.measure.top_detal_angle;
	  
	  float sin_x = (float)sin((double)(detal_angle));
	  float cos_x = (float)cos((double)(detal_angle));
	  int16_t tem_front =  (chassis->base_info.target.front_speed);
	  int16_t tem_right =  (chassis->base_info.target.right_speed);
	
	 /*警钟长鸣！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！*/
	 chassis->base_info.target.front_speed =  (int16_t)((cos_x * tem_front)-(sin_x * tem_right));
	 chassis->base_info.target.right_speed =  (int16_t)((sin_x * tem_front)+(cos_x * tem_right));
	//}
}


/**
  * @Name    Chassis_Pid_Calculating
  * @brief   使用到底盘包
  * @param   底盘 
  * @retval 
  * @author  HWX
  * @Date    2022-11-07
**/
void Chassis_Pid_Calculating(chassis_t *chassis)
{
	
	chassis->base_info.output.motor_LF_current = pid_calc(&chassis_pid_speed_structure[0],motor_3508_LF_structure.base_info->speed,chassis->base_info.output.motor_LF_speed);
	chassis->base_info.output.motor_RF_current = pid_calc(&chassis_pid_speed_structure[1],motor_3508_RF_structure.base_info->speed,chassis->base_info.output.motor_RF_speed);
	chassis->base_info.output.motor_LB_current = pid_calc(&chassis_pid_speed_structure[2],motor_3508_LB_structure.base_info->speed,chassis->base_info.output.motor_LB_speed);
	chassis->base_info.output.motor_RB_current = pid_calc(&chassis_pid_speed_structure[3],motor_3508_RB_structure.base_info->speed,chassis->base_info.output.motor_RB_speed);
}
/**
  * @Name    Chassis_Work
  * @brief   底盘核心函数
  * @param   底盘 三个方向速度
  * @retval
  * @author  HWX
  * @Date    2022-11-06
**/
void Chassis_Work(chassis_t *chassis)
{
	int16_t output_current[4];
	
	/*电机掉线检测*/
	MOTOR_3508_HEART(chassis->motor_LB);
	MOTOR_3508_HEART(chassis->motor_RB);
	MOTOR_3508_HEART(chassis->motor_LF);
	MOTOR_3508_HEART(chassis->motor_RF);
	
	if(chassis->motor_LB->info->status == _DEV_OFFLINE|| \
	   chassis->motor_RB->info->status == _DEV_OFFLINE|| \
	   chassis->motor_LF->info->status == _DEV_OFFLINE|| \
	   chassis->motor_RF->info->status == _DEV_OFFLINE )
	{
		chassis->work_info.work_sate = CHASSIC_OFFLINE;
	
	}
	else
	{
		chassis->work_info.work_sate = CHASSIC_ONLINE;
	}
		
	
	if(chassis->work_info.work_sate == CHASSIC_ONLINE && \
	   rc_structure.info->status == REMOTE_ONLINE)
	{
		
		
		int16_t front = chassis->base_info.target.front_speed;
		int16_t right = chassis->base_info.target.right_speed;
		int16_t cycle = chassis->base_info.target.cycle_speed;
		
		
		/*底盘速度解算*/
		Chassis_Speed_Calculating(chassis, front, right, cycle);
		
		/*旋转量算法*/
		
		
		/*小陀螺底盘解算*/
		//Chassis_Top_Speed_Calculating(chassis);
		
		/*底盘速度限制*/
		Chassis_Speed_Limit(chassis);
		
		/*PID计算*/
		Chassis_Pid_Calculating(chassis);

		
		/*裁判系统速度限制*/
		
		if(judge.info->status == JUDGE_ONLINE)
		{
			chassis->work_info.config.speed_max = 8000;
			
			Chassis_Power_Limit(chassis);
		}
		else
		{
			chassis->work_info.config.speed_max = 2000;
			chassis->work_info.output_max = 4000;
		}

		
		
		output_current[0] = chassis->base_info.output.motor_LF_current;
		output_current[1] = chassis->base_info.output.motor_RF_current;
		output_current[2] = chassis->base_info.output.motor_LB_current;
		output_current[3] = chassis->base_info.output.motor_RB_current;	
		

		/*发送电流*/
		CAN_SendData(&CHASSIS_DRV_CAN_USE,CHASSIS_CAN_STD_ID,output_current);
	}
	else
	{
		output_current[0] = 0;
		output_current[1] = 0;
		output_current[2] = 0;
		output_current[3] = 0;	
		
		/*发送电流*/
		CAN_SendData(&CHASSIS_DRV_CAN_USE,CHASSIS_CAN_STD_ID,output_current);
	}

}



