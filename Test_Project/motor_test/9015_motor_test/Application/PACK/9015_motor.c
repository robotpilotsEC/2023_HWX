#include "9015_motor.h"


#if 0





void Motor_9015_Send_Data(motor_9015_t * motor , motor_9015_cmd_e ID)
{
	int8_t data[8];
	motor_9025_pid_tx_info_t *pid_tx = &motor->pid_info.tx;
	motor_9015_rx_info_t     *data_tx= &motor->base_info.tx_data;
	
	switch (ID)
	{
		case PID_RX_ID:
		
		data[0] = PID_RX_ID;
		
		break;
		
		
		case PID_TX_RAM_ID:
		
		data[0] = PID_TX_RAM_ID;
		data[1] = 0;
		data[2] = pid_tx->angleKp;
		data[3] = pid_tx->angleKi;
		data[4] = pid_tx->speedKp;
		data[5] = pid_tx->speedKi;
		data[6] = pid_tx->iqKp;
		data[7] = pid_tx->iqKi;
		
		break;
		
		case PID_TX_ROM_ID:
			
		data[0] = PID_TX_ROM_ID;
		data[1] = 0;
		data[2] = pid_tx->angleKp;
		data[3] = pid_tx->angleKi;
		data[4] = pid_tx->speedKp;
		data[5] = pid_tx->speedKi;
		data[6] = pid_tx->iqKp;
		data[7] = pid_tx->iqKi;
		
		break;
		
		
		
		case ACCEL_RX_ID:
			
		data[0] = ACCEL_RX_ID;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0;
		data[5] = 0;
		data[6] = 0;
		data[7] = 0;
		
		break;
		
		case ACCEL_TX_ID:
			
		data[0] = ACCEL_TX_ID;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		data[4] = 0;
		data[5] = 0;
		data[6] = 0;
		data[7] = 0;
		
		break;
		

	}
	
	CAN_u8_SendData();
}



/**
  * @Name    Motor_9015_Get_Data
  * @brief   数据接收处理函数
  * @param   motor: [输入/出] 
  * @retval
  * @author  HWX
  * @Date    2022-11-27
**/
void Motor_9015_Get_Data(motor_9015_t *motor , uint8_t *rxBuf)
{
	motor_9015_cmd_e ID = rxBuf[0];

	motor_9025_pid_rx_info_t *pid_rx_info = &motor->pid_info.rx;
	motor_9015_base_info_t   *base_info   = &motor->base_info;
	switch (ID)
	{
		
		/*接收pid数据*/
		case PID_RX_ID:
		case PID_TX_RAM_ID:
		case PID_TX_ROM_ID:
		pid_rx_info->angleKp = rxBuf[2];
		pid_rx_info->angleKi = rxBuf[3];
		pid_rx_info->speedKp = rxBuf[4];
		pid_rx_info->speedKi = rxBuf[5];
		pid_rx_info->iqKp	 = rxBuf[6];
		pid_rx_info->iqKi	 = rxBuf[7];
		break;
		
		
		
		case ACCEL_RX_ID:
		base_info->accel  = rxBuf[7];
		base_info->accel <<= 8;
		base_info->accel |= rxBuf[6];
		base_info->accel <<= 8;
		base_info->accel |= rxBuf[5];
		base_info->accel <<= 8;
		base_info->accel |= rxBuf[4];
		base_info->accel <<= 8;
		break;
		
		
		case ENCODER_RX_ID:
		state_info->encoder = rxBuf[3];
		state_info->encoder <<= 8;
		state_info->encoder |= rxBuf[2];
		state_info->encoderRaw = rxBuf[5];
		state_info->encoderRaw <<= 8;
		state_info->encoderRaw |= rxBuf[4];
		state_info->encoderOffset = rxBuf[7];
		state_info->encoderOffset <<= 8;
		state_info->encoderOffset |= rxBuf[6];
		break;
		
		case MOTOR_ANGLE_ID:
		base_info->motorAngle  = rxBuf[7];
		base_info->motorAngle <<= 8;
		base_info->motorAngle |= rxBuf[6];
		base_info->motorAngle <<= 8;
		base_info->motorAngle |= rxBuf[5];
		base_info->motorAngle <<= 8;
		base_info->motorAngle |= rxBuf[4];
		base_info->motorAngle <<= 8;
		base_info->motorAngle |= rxBuf[3];
		base_info->motorAngle <<= 8;
		base_info->motorAngle |= rxBuf[2];
		base_info->motorAngle <<= 8;
		base_info->motorAngle |= rxBuf[1];
		break;
		
		case CIRCLE_ANGLE_ID:
		base_info->circleAngle  = rxBuf[7];
		base_info->circleAngle <<= 8;
		base_info->circleAngle |= rxBuf[6];
		base_info->circleAngle <<= 8;
		base_info->circleAngle |= rxBuf[5];
		base_info->circleAngle <<= 8;
		base_info->circleAngle |= rxBuf[4];
		break;
		
		case STATE1_ID:
		state_info->temperature = rxBuf[1]; 
		state_info->voltage = rxBuf[4];
		state_info->voltage <<= 8;
		state_info->voltage |= rxBuf[3];
		state_info->errorState = rxBuf[7];
		break;
		
		case STATE2_ID:
		state_info->temperature = rxBuf[1];
		state_info->current = rxBuf[3];
		state_info->current <<= 8;
		state_info->current = rxBuf[2];
		base_info->speed = rxBuf[5];
		base_info->speed <<= 8;
		base_info->speed = rxBuf[4];
		state_info->encoder = rxBuf[7];
		state_info->encoder <<= 8;
		state_info->encoder = rxBuf[6];
		break;
		
		case STATE3_ID:
		state_info->temperature = rxBuf[1];
		state_info->current_A = rxBuf[3];
		state_info->current_A <<= 8;
		state_info->current_A = rxBuf[2];
		state_info->current_B = rxBuf[5];
		state_info->current_B <<= 8;
		state_info->current_B = rxBuf[4];
		state_info->current_C = rxBuf[7];
		state_info->current_C <<= 8;
		state_info->current_C = rxBuf[6];
		break;
	}
	
}
#endif