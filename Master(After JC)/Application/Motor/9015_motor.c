#include "9015_motor.h"
#include "drv_can.h"
#include "bmi.h"
/*单圈角度值有问题,角度数据类型问题*/

/* Exported variables --------------------------------------------------------*/
extern bmi_t bmi_structure;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern motor_9015_t           motor_9015_structure;
extern motor_9015_info_t      motor_9015_info_structure;
extern motor_9015_base_info_t motor_9015_base_info_structure;
extern motor_9015_pid_t       motor_9015_pid_structure;

/*9015电机pid计算结构体*/
pid_t                  motor_9015_pid_spd;
pid_t                  motor_9015_pid_pos;
pid_t                  motor_9015_pid_bmi;



/*电机初始化*/
void MOTOR_9015_INIT()
{
	/*PID初始化*/
	PID_struct_init( &motor_9015_pid_spd,POSITION_PID,S_PIT_LIM_O,S_PIT_LIM_I,S_PIT_PID_P,S_PIT_PID_I,S_PIT_PID_D);
	PID_struct_init( &motor_9015_pid_pos,POSITION_PID,P_PIT_LIM_O,P_PIT_LIM_I,P_PIT_PID_P,P_PIT_PID_I,P_PIT_PID_D);
	PID_struct_init( &motor_9015_pid_bmi,POSITION_PID,P_PIT_LIM_O,P_PIT_LIM_I,-5,P_PIT_PID_I,P_PIT_PID_D);
	
	motor_9015_structure.base_info = &motor_9015_base_info_structure;
	motor_9015_structure.info      = &motor_9015_info_structure;
	motor_9015_structure.pid       = &motor_9015_pid_structure;
	
	motor_9015_base_info_structure.target_angle = motor_9015_base_info_structure.angle;
	motor_9015_base_info_structure.target_speed = motor_9015_base_info_structure.speed;
	
	motor_9015_info_structure.offline_cnt     = OFFLINE_TIME_MAX;
	motor_9015_info_structure.offline_cnt_max = OFFLINE_TIME_MAX;
	motor_9015_info_structure.status          = _9015_OFFLINE;
	
	memset(&motor_9015_structure.tx_buff,0,8);
	

	
	
	
	
}


void MOTOR_9015_SET_PID()
{
	
	motor_9015_structure.tx_buff[0]    = PID_TX_RAM_ID ;
	motor_9015_structure.tx_buff[1]    = 0x00;
	motor_9015_structure.tx_buff[2]    = motor_9015_structure.pid->angleKp;
    motor_9015_structure.tx_buff[3]    = motor_9015_structure.pid->angleKi;
    motor_9015_structure.tx_buff[4]    = motor_9015_structure.pid->speedKp;
    motor_9015_structure.tx_buff[5]    = motor_9015_structure.pid->speedKi;
	motor_9015_structure.tx_buff[6]    = motor_9015_structure.pid->iqKp;
    motor_9015_structure.tx_buff[7]    = motor_9015_structure.pid->iqKi;
	
	CAN_u8_SendData(&MOTOR_9015_CAN,MOTOR_9015_SENT_ID,motor_9015_structure.tx_buff);
	
	memset(&motor_9015_structure.tx_buff,0,8);
}

/** 
 *	@brief 电机扭矩控制
 */

void MOTOR_9015_Iq(int16_t iq)
{
	
	/*角度等于0的时候会有问题*/
	motor_9015_structure.tx_buff[0]    = TORQUE_CLOSE_LOOP_ID; //扭矩闭环控制
	motor_9015_structure.tx_buff[1]    = 0x00;
	motor_9015_structure.tx_buff[2]    = 0x00;
    motor_9015_structure.tx_buff[3]    = 0x00;
    motor_9015_structure.tx_buff[4]    = (uint8_t) (iq);
    motor_9015_structure.tx_buff[5]    = (uint8_t) (iq >> 8);
	motor_9015_structure.tx_buff[6]    = 0x00;
    motor_9015_structure.tx_buff[7]    = 0x00;
	
	CAN_u8_SendData(&MOTOR_9015_CAN,MOTOR_9015_SENT_ID,motor_9015_structure.tx_buff);
	
	memset(&motor_9015_structure.tx_buff,0,8);
}


void MOTOR_9015_OFF()
{
	
	MOTOR_9015_Iq(0);
		
}

void MOTOR_9015_SPEED()
{
	/*陀螺仪数据*/
	motor_9015_structure.base_info->speed = (-1)*bmi_structure.yaw_gro;
	//motor_9015_structure.base_info->speed *= -1;
	motor_9015_structure.base_info->target_iq    = pid_calc(&motor_9015_pid_spd,\
														        motor_9015_structure.base_info->speed,\
														        motor_9015_structure.base_info->target_speed);
	
	MOTOR_9015_Iq(motor_9015_structure.base_info->target_iq);
		
}


void MOTOR_9015_POSIT()
{
	/*陀螺仪数据*/
	motor_9015_structure.base_info->speed = bmi_structure.yaw_gro;
	
	motor_9015_structure.base_info->target_speed    = pid_calc_err_9015(&motor_9015_pid_pos,\
														        motor_9015_structure.base_info->encoder,\
														        motor_9015_structure.base_info->target_angle);
	
	motor_9015_structure.base_info->target_iq    = pid_calc(&motor_9015_pid_spd,\
														        motor_9015_structure.base_info->speed,\
														        motor_9015_structure.base_info->target_speed);
	
	MOTOR_9015_Iq(motor_9015_structure.base_info->target_iq);
}

extern float yaw_9015_use;
void MOTOR_9015_POSIT_BMI()
{
	/*陀螺仪数据*/
	int16_t yaw_speed = bmi_structure.yaw_gro;
	float yaw_angle = yaw_9015_use;
	motor_9015_structure.base_info->target_speed    = pid_calc_err(&motor_9015_pid_bmi,\
														        yaw_angle,\
														        (motor_9015_structure.base_info->target_angle/8.0));
	
	motor_9015_structure.base_info->target_iq    = pid_calc(&motor_9015_pid_spd,\
														        yaw_speed,\
														        motor_9015_structure.base_info->target_speed);
	
	MOTOR_9015_Iq(motor_9015_structure.base_info->target_iq);
}
void MOTOR_9015_POSIT_BMI_VISION()
{
	/*陀螺仪数据*/
	int16_t yaw_speed = bmi_structure.yaw_gro;
	int16_t yaw_angle = bmi_structure.yaw_angle;
	motor_9015_structure.base_info->target_speed    = pid_calc_err(&motor_9015_pid_bmi,\
														        yaw_angle,\
														        (motor_9015_structure.base_info->target_angle));
	
	motor_9015_structure.base_info->target_iq     = pid_calc(&motor_9015_pid_spd,\
														        yaw_speed,\
														        motor_9015_structure.base_info->target_speed);
	
	MOTOR_9015_Iq(motor_9015_structure.base_info->target_iq);
}


void MOTOR_9015_HEART()
{
	static uint16_t motor_9015_err_cnt = 0;
	if(motor_9015_structure.info->offline_cnt ++ >= motor_9015_structure.info->offline_cnt_max)
	{
		motor_9015_structure.info->offline_cnt = motor_9015_structure.info->offline_cnt_max;
		motor_9015_structure.info->status = _9015_OFFLINE;
	}
	else
	{
		motor_9015_structure.info->status = _9015_ONLINE;
		/*清除低压保护*/
//		if(abs(motor_9015_structure.base_info->target_iq - motor_9015_structure.base_info->iq) <= 200)
//		{
//			if(motor_9015_err_cnt++ >= 100)
//			{
//				motor_9015_err_cnt = 100;
//				motor_9015_structure.info->status = _9015_ERRO;
//			}
//			else
//			{
//				motor_9015_err_cnt = 0;
//				motor_9015_structure.info->status = _9015_ONLINE;
//			}
//		}
	}
	

}




/** 
 *	@brief 给电机发送主动读取某些参数的命令，内部已经有can的发送函数
 */
void MOTOR_9015_DATA(uint8_t command)
{

	memset(&motor_9015_structure.tx_buff,0,8);
	
	switch(command)
	{
		case PID_RX_ID:   	//读取发送PID结构体参数
			motor_9015_structure.tx_buff[0] = PID_RX_ID;
		break;
		
		case ACCEL_RX_ID:   //读取发送的结构体中的加速度参数
			motor_9015_structure.tx_buff[0] = ACCEL_RX_ID;
		break;
		
		case ENCODER_RX_ID:   //读取发送结构体中的编码器数据
			motor_9015_structure.tx_buff[0] = ENCODER_RX_ID;
		break;
		
		case MOTOR_ANGLE_ID:  //读取电机多圈绝对角度
		    motor_9015_structure.tx_buff[0] = MOTOR_ANGLE_ID;
		break;
		
		case CIRCLE_ANGLE_ID:  //读取电机单圈角度
			 motor_9015_structure.tx_buff[0] = CIRCLE_ANGLE_ID;
		break;
		
		case STATE1_ID:        //读取电机状态1和错误标志位
			 motor_9015_structure.tx_buff[0] = STATE1_ID;
		break;
		
		case STATE2_ID:        //读取电机状态2
			 motor_9015_structure.tx_buff[0] = STATE2_ID;
		break;
		
		case STATE3_ID:        //读取电机状态3
			 motor_9015_structure.tx_buff[0] = STATE3_ID;
		break;
		
		default:
			break;
	}
	
	CAN_u8_SendData(&MOTOR_9015_CAN,MOTOR_9015_SENT_ID,motor_9015_structure.tx_buff);
	

		return;
}


/**
 *	@brief	接收电机发来的信息并自动更新，需要注意，大部分发送给电机的指令，电机也会返回一些数据
						如果传入空指针，认为电机数据出错，并返回
						增加了对电机角度和的更新
 *  @return
 */
/*放在can里面,现在就只有单圈角度和pid参数，速度应该也要*/
void MOTOR_9015_UPDATE(uint8_t *rxBuf)
{

	
	uint8_t ID = rxBuf[0];
	
	int16_t angle_err = 0;
	
	motor_9015_structure.info->offline_cnt = 0;

	switch (ID)
	{
		case PID_RX_ID:                      //读取PID
		case PID_TX_RAM_ID: 
		case PID_TX_ROM_ID:	
			motor_9015_structure.pid->angleKp = rxBuf[2];
			motor_9015_structure.pid->angleKi = rxBuf[3];
			motor_9015_structure.pid->speedKp = rxBuf[4];
			motor_9015_structure.pid->speedKi = rxBuf[5];
			motor_9015_structure.pid->iqKp    = rxBuf[6];
			motor_9015_structure.pid->iqKi    = rxBuf[7];
		break;
		
		
		case TORQUE_CLOSE_LOOP_ID:
		case SPEED_CLOSE_LOOP_ID:
			motor_9015_structure.base_info->temperature =  (int8_t)rxBuf[1];
		
			motor_9015_structure.base_info->iq = (int16_t)rxBuf[3];
			motor_9015_structure.base_info->iq <<= 8;
			motor_9015_structure.base_info->iq |= (int16_t)rxBuf[2];
		
			motor_9015_structure.base_info->speed = (int16_t)rxBuf[5];
			motor_9015_structure.base_info->speed <<= 8;
			motor_9015_structure.base_info->speed |= (int16_t)rxBuf[4];
		
			motor_9015_structure.base_info->encoder = (uint16_t)rxBuf[7];
			motor_9015_structure.base_info->encoder <<= 8;
			motor_9015_structure.base_info->encoder |= (uint16_t)rxBuf[6];
		break;
			
		

#if 0	
		case ACCEL_RX_ID:                    //主动读取加速度
			rx_info->accel  = (int32_t)rxBuf[7];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[6];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[5];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[4];
			rx_info->accel <<= 8;
		break;
		
		case ACCEL_TX_ID:                    //发送加速度参数到RAM会返回
			rx_info->accel  = (int32_t)rxBuf[7];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[6];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[5];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[4];
			rx_info->accel <<= 8;
		break;	

		case ENCODER_RX_ID:                   //主动读取编码器
			rx_info->encoder = (uint16_t)rxBuf[3];
			rx_info->encoder <<= 8;
			rx_info->encoder |= (uint16_t)rxBuf[2];
			rx_info->encoderRaw = (uint16_t)rxBuf[5];
			rx_info->encoderRaw <<= 8;
			rx_info->encoderRaw |= (uint16_t)rxBuf[4];
			rx_info->encoderOffset = (uint16_t)rxBuf[7];
			rx_info->encoderOffset <<= 8;
			rx_info->encoderOffset |= (uint16_t)rxBuf[6];
		break;
		
		case ZERO_ENCODER_TX_ID:              //写入编码器值到ROM作为电机零点会返回
			rx_info->encoderOffset = (uint16_t)rxBuf[7];
			rx_info->encoderOffset <<= 8;
			rx_info->encoderOffset |= (uint16_t)rxBuf[6];
		break;
		
		case ZERO_POSNOW_TX_ID:              //写入编码器值到RAM作为电机零点会返回
			rx_info->encoderOffset = (uint16_t)rxBuf[7];
			rx_info->encoderOffset <<= 8;
			rx_info->encoderOffset |= (uint16_t)rxBuf[6];
		break;
		
		
		case MOTOR_ANGLE_ID:                  //主动读取电机多圈绝对角度，正值顺时针累计角度
			rx_info->motorAngle  = (int64_t)rxBuf[7];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[6];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[5];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[4];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[3];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[2];
			rx_info->motorAngle <<= 8;
			rx_info->motorAngle |= (int64_t)rxBuf[1];
		break;
#endif
		case CIRCLE_ANGLE_ID:                 //主动读取电机单圈角度
			motor_9015_structure.base_info->angle  = (uint32_t)rxBuf[7];
			motor_9015_structure.base_info->angle <<= 8;
			motor_9015_structure.base_info->angle |= (uint32_t)rxBuf[6];
			motor_9015_structure.base_info->angle <<= 8;
			motor_9015_structure.base_info->angle |= (uint32_t)rxBuf[5];
			motor_9015_structure.base_info->angle <<= 8;
			motor_9015_structure.base_info->angle |= (uint32_t)rxBuf[4];
		break;
		
#if 0
		case STATE1_ID:                       //主动读取电机状态1和错误标志位
			rx_info->temperature = (int8_t)rxBuf[1]; 
			rx_info->voltage = (uint16_t)rxBuf[4];
			rx_info->voltage <<= 8;
			rx_info->voltage |= (uint16_t)rxBuf[3];
			rx_info->errorState = rxBuf[7];
		break;
		
		case STATE2_ID:                       //主动读取电机状态2
			rx_info->temperature = (int8_t)rxBuf[1];
			rx_info->current = (int16_t)rxBuf[3];
			rx_info->current <<= 8;
			rx_info->current |= (int16_t)rxBuf[2];
			rx_info->speed = (int16_t)rxBuf[5];
			rx_info->speed <<= 8;
			rx_info->speed |= (int16_t)rxBuf[4];
			rx_info->encoder = (uint16_t)rxBuf[7];
			rx_info->encoder <<= 8;
			rx_info->encoder |= (uint16_t)rxBuf[6];
		break;
		
		case STATE3_ID:                        //主动读取电机状态3
			rx_info->temperature = (int8_t)rxBuf[1];
			rx_info->current_A = (int16_t)rxBuf[3];
			rx_info->current_A <<= 8;
			rx_info->current_A |= (int16_t)rxBuf[2];
			rx_info->current_B = (int16_t)rxBuf[5];
			rx_info->current_B <<= 8;
			rx_info->current_B |= (int16_t)rxBuf[4];
			rx_info->current_C = (int16_t)rxBuf[7];
			rx_info->current_C <<= 8;
			rx_info->current_C |= (int16_t)rxBuf[6];
		break;
		
		case TORQUE_OPEN_LOOP_ID:              //扭矩开环控制会自动返回
			rx_info->temperature = (int8_t)rxBuf[1]; 
			rx_info->powerControl = (int16_t)rxBuf[3];
			rx_info->powerControl <<= 8;	
			rx_info->powerControl |= (int16_t)rxBuf[2];
			rx_info->speed = (int16_t)rxBuf[5];
			rx_info->speed <<= 8;
			rx_info->speed |= (int16_t)rxBuf[4];
			rx_info->encoder = (uint16_t)rxBuf[7];
			rx_info->encoder <<= 8;
			rx_info->encoder |= (uint16_t)rxBuf[6];
		break;
		
		case  TORQUE_CLOSE_LOOP_ID:	//扭矩闭环控制、速度闭环、所有位置闭环都会返回
		case  SPEED_CLOSE_LOOP_ID :
		case  POSI_CLOSE_LOOP_ID1 :
		case  POSI_CLOSE_LOOP_ID2 :
		case  POSI_CLOSE_LOOP_ID3 :
		case  POSI_CLOSE_LOOP_ID4 :	
		case  POSI_CLOSE_LOOP_ID5 :
		case  POSI_CLOSE_LOOP_ID6 :
			rx_info->temperature = (int8_t)rxBuf[1]; 
			rx_info->current = (int16_t)rxBuf[3];
			rx_info->current <<= 8;	
			rx_info->current |= (int16_t)rxBuf[2];
			rx_info->speed = (int16_t)rxBuf[5];
			rx_info->speed <<= 8;
			rx_info->speed |= (int16_t)rxBuf[4];
			rx_info->encoder = (uint16_t)rxBuf[7];
			rx_info->encoder <<= 8;
			rx_info->encoder |= (uint16_t)rxBuf[6];
		break;
		
#endif
		default:
			break;
	}

	
#if 0
	angle_err = rx_info->encoder - rx_info->encoder_prev;
	
	/* 过零点 */
	if( abs(angle_err) > KT_18_BIT_TOTAL_ANGLE / 2 )
	{
		if( angle_err >= 0 )
			rx_info->encoder_sum += -KT_18_BIT_TOTAL_ANGLE + angle_err;

		else
			rx_info->encoder_sum +=  KT_18_BIT_TOTAL_ANGLE  + angle_err;
	}
	/* 未过零点 */
	else
	{
		rx_info->encoder_sum += angle_err;
	}
	
	//转变为角度，再转变为弧度
	rx_info->radian_sum = rx_info->encoder_sum * KT_18_BIT_ANGLE_CONVERSION * ANGLE_CONVERSION_RADIAN;
	
	rx_info->encoder_prev = rx_info->encoder;
#endif
	
}



void MOTOR_9015_STOP()
{

	motor_9015_structure.tx_buff[0]    = MOTOR_CLOSE_ID;
	motor_9015_structure.tx_buff[1]    = 0x00;
	motor_9015_structure.tx_buff[2]    = 0x00;
    motor_9015_structure.tx_buff[3]    = 0x00;
    motor_9015_structure.tx_buff[4]    = 0x00;
    motor_9015_structure.tx_buff[5]    = 0x00;
	motor_9015_structure.tx_buff[6]    = 0x00;
    motor_9015_structure.tx_buff[7]    = 0x00;
	
	CAN_u8_SendData(&MOTOR_9015_CAN,MOTOR_9015_SENT_ID,motor_9015_structure.tx_buff);
	
	memset(&motor_9015_structure.tx_buff,0,8);
}



void MOTOR_9015_STAR()
{
	motor_9015_structure.tx_buff[0]    = MOTOR_RUN_ID;//单圈角度，有方向
	motor_9015_structure.tx_buff[1]    = 0x00;
	motor_9015_structure.tx_buff[2]    = 0x00;
    motor_9015_structure.tx_buff[3]    = 0x00;
    motor_9015_structure.tx_buff[4]    = 0x00;
    motor_9015_structure.tx_buff[5]    = 0x00;
	motor_9015_structure.tx_buff[6]    = 0x00;
    motor_9015_structure.tx_buff[7]    = 0x00;
	
	CAN_u8_SendData(&MOTOR_9015_CAN,MOTOR_9015_SENT_ID,motor_9015_structure.tx_buff);
	
	memset(&motor_9015_structure.tx_buff,0,8);
	
}
















