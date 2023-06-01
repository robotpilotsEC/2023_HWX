#include "9015_motor.h"
#include "drv_can.h"
#include "bmi.h"
/*��Ȧ�Ƕ�ֵ������,�Ƕ�������������*/

/* Exported variables --------------------------------------------------------*/
extern bmi_t bmi_structure;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern motor_9015_t           motor_9015_structure;
extern motor_9015_info_t      motor_9015_info_structure;
extern motor_9015_base_info_t motor_9015_base_info_structure;
extern motor_9015_pid_t       motor_9015_pid_structure;

/*9015���pid����ṹ��*/
pid_t                  motor_9015_pid_spd;
pid_t                  motor_9015_pid_pos;
pid_t                  motor_9015_pid_bmi;



/*�����ʼ��*/
void MOTOR_9015_INIT()
{
	/*PID��ʼ��*/
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
 *	@brief ���Ť�ؿ���
 */

void MOTOR_9015_Iq(int16_t iq)
{
	
	/*�Ƕȵ���0��ʱ���������*/
	motor_9015_structure.tx_buff[0]    = TORQUE_CLOSE_LOOP_ID; //Ť�رջ�����
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
	/*����������*/
	motor_9015_structure.base_info->speed = (-1)*bmi_structure.yaw_gro;
	//motor_9015_structure.base_info->speed *= -1;
	motor_9015_structure.base_info->target_iq    = pid_calc(&motor_9015_pid_spd,\
														        motor_9015_structure.base_info->speed,\
														        motor_9015_structure.base_info->target_speed);
	
	MOTOR_9015_Iq(motor_9015_structure.base_info->target_iq);
		
}


void MOTOR_9015_POSIT()
{
	/*����������*/
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
	/*����������*/
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
	/*����������*/
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
		/*�����ѹ����*/
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
 *	@brief ���������������ȡĳЩ����������ڲ��Ѿ���can�ķ��ͺ���
 */
void MOTOR_9015_DATA(uint8_t command)
{

	memset(&motor_9015_structure.tx_buff,0,8);
	
	switch(command)
	{
		case PID_RX_ID:   	//��ȡ����PID�ṹ�����
			motor_9015_structure.tx_buff[0] = PID_RX_ID;
		break;
		
		case ACCEL_RX_ID:   //��ȡ���͵Ľṹ���еļ��ٶȲ���
			motor_9015_structure.tx_buff[0] = ACCEL_RX_ID;
		break;
		
		case ENCODER_RX_ID:   //��ȡ���ͽṹ���еı���������
			motor_9015_structure.tx_buff[0] = ENCODER_RX_ID;
		break;
		
		case MOTOR_ANGLE_ID:  //��ȡ�����Ȧ���ԽǶ�
		    motor_9015_structure.tx_buff[0] = MOTOR_ANGLE_ID;
		break;
		
		case CIRCLE_ANGLE_ID:  //��ȡ�����Ȧ�Ƕ�
			 motor_9015_structure.tx_buff[0] = CIRCLE_ANGLE_ID;
		break;
		
		case STATE1_ID:        //��ȡ���״̬1�ʹ����־λ
			 motor_9015_structure.tx_buff[0] = STATE1_ID;
		break;
		
		case STATE2_ID:        //��ȡ���״̬2
			 motor_9015_structure.tx_buff[0] = STATE2_ID;
		break;
		
		case STATE3_ID:        //��ȡ���״̬3
			 motor_9015_structure.tx_buff[0] = STATE3_ID;
		break;
		
		default:
			break;
	}
	
	CAN_u8_SendData(&MOTOR_9015_CAN,MOTOR_9015_SENT_ID,motor_9015_structure.tx_buff);
	

		return;
}


/**
 *	@brief	���յ����������Ϣ���Զ����£���Ҫע�⣬�󲿷ַ��͸������ָ����Ҳ�᷵��һЩ����
						��������ָ�룬��Ϊ������ݳ���������
						�����˶Ե���ǶȺ͵ĸ���
 *  @return
 */
/*����can����,���ھ�ֻ�е�Ȧ�ǶȺ�pid�������ٶ�Ӧ��ҲҪ*/
void MOTOR_9015_UPDATE(uint8_t *rxBuf)
{

	
	uint8_t ID = rxBuf[0];
	
	int16_t angle_err = 0;
	
	motor_9015_structure.info->offline_cnt = 0;

	switch (ID)
	{
		case PID_RX_ID:                      //��ȡPID
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
		case ACCEL_RX_ID:                    //������ȡ���ٶ�
			rx_info->accel  = (int32_t)rxBuf[7];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[6];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[5];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[4];
			rx_info->accel <<= 8;
		break;
		
		case ACCEL_TX_ID:                    //���ͼ��ٶȲ�����RAM�᷵��
			rx_info->accel  = (int32_t)rxBuf[7];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[6];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[5];
			rx_info->accel <<= 8;
			rx_info->accel |= (int32_t)rxBuf[4];
			rx_info->accel <<= 8;
		break;	

		case ENCODER_RX_ID:                   //������ȡ������
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
		
		case ZERO_ENCODER_TX_ID:              //д�������ֵ��ROM��Ϊ������᷵��
			rx_info->encoderOffset = (uint16_t)rxBuf[7];
			rx_info->encoderOffset <<= 8;
			rx_info->encoderOffset |= (uint16_t)rxBuf[6];
		break;
		
		case ZERO_POSNOW_TX_ID:              //д�������ֵ��RAM��Ϊ������᷵��
			rx_info->encoderOffset = (uint16_t)rxBuf[7];
			rx_info->encoderOffset <<= 8;
			rx_info->encoderOffset |= (uint16_t)rxBuf[6];
		break;
		
		
		case MOTOR_ANGLE_ID:                  //������ȡ�����Ȧ���ԽǶȣ���ֵ˳ʱ���ۼƽǶ�
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
		case CIRCLE_ANGLE_ID:                 //������ȡ�����Ȧ�Ƕ�
			motor_9015_structure.base_info->angle  = (uint32_t)rxBuf[7];
			motor_9015_structure.base_info->angle <<= 8;
			motor_9015_structure.base_info->angle |= (uint32_t)rxBuf[6];
			motor_9015_structure.base_info->angle <<= 8;
			motor_9015_structure.base_info->angle |= (uint32_t)rxBuf[5];
			motor_9015_structure.base_info->angle <<= 8;
			motor_9015_structure.base_info->angle |= (uint32_t)rxBuf[4];
		break;
		
#if 0
		case STATE1_ID:                       //������ȡ���״̬1�ʹ����־λ
			rx_info->temperature = (int8_t)rxBuf[1]; 
			rx_info->voltage = (uint16_t)rxBuf[4];
			rx_info->voltage <<= 8;
			rx_info->voltage |= (uint16_t)rxBuf[3];
			rx_info->errorState = rxBuf[7];
		break;
		
		case STATE2_ID:                       //������ȡ���״̬2
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
		
		case STATE3_ID:                        //������ȡ���״̬3
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
		
		case TORQUE_OPEN_LOOP_ID:              //Ť�ؿ������ƻ��Զ�����
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
		
		case  TORQUE_CLOSE_LOOP_ID:	//Ť�رջ����ơ��ٶȱջ�������λ�ñջ����᷵��
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
	
	/* ����� */
	if( abs(angle_err) > KT_18_BIT_TOTAL_ANGLE / 2 )
	{
		if( angle_err >= 0 )
			rx_info->encoder_sum += -KT_18_BIT_TOTAL_ANGLE + angle_err;

		else
			rx_info->encoder_sum +=  KT_18_BIT_TOTAL_ANGLE  + angle_err;
	}
	/* δ����� */
	else
	{
		rx_info->encoder_sum += angle_err;
	}
	
	//ת��Ϊ�Ƕȣ���ת��Ϊ����
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
	motor_9015_structure.tx_buff[0]    = MOTOR_RUN_ID;//��Ȧ�Ƕȣ��з���
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
















