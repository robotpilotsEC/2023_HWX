
#include "2006_motor.h"
#include "rp_math.h"
#include "string.h"
//#include "my_shoot.h"
#include "can_protocol.h"

#define one(x)					((x)>0? (1):(-1))

extern Master_Head_t Master_Head_structure;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_RxFrameTypeDef hcan1RxFrame;
extern CAN_RxFrameTypeDef hcan2RxFrame;

//extern shoot_t shoot_structure;
//extern int8_t  deal_done;



/**
  * @brief  初始化6020结构体
  * @param  motor_6020_t*  motor_6020_base_info_t* motor_6020_info_t*
	* @note   提醒自己别忘了给结构体指针赋值
  */

void MOTOR_2006_INIT(motor_2006_t *motor,\
										 motor_2006_base_info_t* base,\
										 motor_2006_info_t* info)
{
	motor->base_info=base;
	memset(motor->base_info,0,sizeof(motor_2006_base_info_t));
	
	motor->info=info;
	motor->info->status=_DEV_OFFLINE;
	
	motor->info->offline_cnt=OFFLINE_TIME_MAX;
	motor->info->offline_cnt_max=OFFLINE_TIME_MAX;
	
	motor->output_current=0;
	motor->base_info->circle_cnt = 0;
	
	motor->base_info->angle = 0 ;
	motor->base_info->target_angle_sum= 0 ; 
	motor->base_info->done_angle = 0;
}


/**
	* @brief  发送数据给2006
  * @param  
* @retval   被征用为M2H发送模式，云台模式与发射机构模式
	* @note   使用can2
  */
HAL_StatusTypeDef MOTOR_2006_CAN_SENT_DATA( int16_t data_1,int16_t data_2,int16_t data_3,int16_t data_4)//晚点再改，先转起来
{
	
	
	uint32_t txMailBox;//发送邮箱
	CAN_TxHeaderTypeDef txFrameHeader;
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint8_t data[8];
	
	/*增加数据*/

	/*数据处理*/
	data[0]      = data_1 >> 8;
	data[1]      = data_1;      //波蛋轮左
	data[2]      = data_2 >> 8;
	data[3]      = data_2;      //波蛋轮右
	
	data[4]      = Master_Head_structure.Send_L_Head.gimbal_mode; //左 云台
	data[5]      = Master_Head_structure.Send_R_Head.gimbal_mode; //右 云台
	data[6]      = Master_Head_structure.Send_L_Head.shoot_mode; //左 发射
	data[7]      = Master_Head_structure.Send_R_Head.shoot_mode; //右 发射
	/*数据帧开头*/
	txFrameHeader.StdId = MOTOR_2006_SENT_ID;
	txFrameHeader.IDE   = CAN_ID_STD;
	txFrameHeader.RTR   = CAN_RTR_DATA;
	txFrameHeader.DLC   = 0x08;
	
	
	  /*默认使用CAN2*/
  ret = HAL_CAN_AddTxMessage(&hcan2, &txFrameHeader, data, &txMailBox);
    
	memset(data,0,sizeof(data));
	
	
	return ret;
		
		
}


/**
  * @brief  将缓冲区中的数据写入结构体
  * @param  motor_2006_t *
  * @retval NONE
  */
void MOTOR_2006_GET_DATA(motor_2006_t *motor)
{
	int16_t  angle_last = motor->base_info->angle;
	
	/*更新信息 */
	motor->base_info->angle = hcan2RxFrame.data[0];
	motor->base_info->angle <<= 8;
	motor->base_info->angle |= hcan2RxFrame.data[1];
	motor->base_info->speed = hcan2RxFrame.data[2];
	motor->base_info->speed <<= 8;
	motor->base_info->speed |= hcan2RxFrame.data[3];
	motor->base_info->current = hcan2RxFrame.data[4];
	motor->base_info->current <<= 8;
	motor->base_info->current |= hcan2RxFrame.data[5];
	motor->base_info->temperature = hcan2RxFrame.data[6];	
	
	
	/* angle_add */
	motor->base_info->angle_add = motor->base_info->angle - angle_last;
	
	if(abs(motor->base_info->angle_add) > 4096)
	{
		motor->base_info->angle_add -= 8192 * one(motor->base_info->angle_add);
	}
	
//	/* angle_sum and target_angle_sum */
//	motor->base_info->angle_sum += motor->base_info->angle_add;
}


/**
  * @brief  检测电机是否掉线
  * @param  motor_2006_t *
  * @retval NONE
  */
void MOTOR_2006_HEART(motor_2006_t *motor)
{
	if(motor->info->offline_cnt++ >= motor->info->offline_cnt_max)
	{
		motor->info->status = _DEV_OFFLINE;
		motor->info->offline_cnt = motor->info->offline_cnt_max;
	}
	else
	{
		motor->info->status = _DEV_ONLINE;
	}
}

