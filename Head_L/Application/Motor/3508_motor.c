
#include "3508_motor.h"
#include "rp_math.h"
#include "string.h"


#define MOTOR_3508_SENT_ID 0x200

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_RxFrameTypeDef hcan1RxFrame;
extern CAN_RxFrameTypeDef hcan2RxFrame;



/**
  * @brief  初始化3508结构体
  * @param
  * @retval
  */
void MOTOR_3508_INIT(motor_3508_t *motor,\
                     motor_3508_base_info_t *base,\
                     motor_3508_info_t *info)
{
    motor->base_info=base;
    memset(motor->base_info,0,sizeof(motor_3508_base_info_t));

    motor->info=info;
    motor->info->status=_DEV_OFFLINE;

    motor->info->offline_cnt=OFFLINE_TIME_MAX;
    motor->info->offline_cnt_max=OFFLINE_TIME_MAX;

    motor->output_current=0;

}

/**
  * @brief
  * @param
  * @retval
	* @note 使用can1
  */
HAL_StatusTypeDef MOTOR_3508_CAN1_SENT_DATA( uint16_t data_1,uint16_t data_2,uint16_t data_3,uint16_t data_4)
{


    uint32_t txMailBox;//发送邮箱
    CAN_TxHeaderTypeDef txFrameHeader;
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t data[8];
    /*数据处理*/
    data[0]      = data_1 >> 8;//hello
    data[1]      = data_1;
    data[2]      = data_2 >> 8;
    data[3]      = data_2;
    data[4]      = data_3 >> 8;
    data[5]      = data_3;
    data[6]      = data_4 >> 8;
    data[7]      = data_4;
    /*数据帧开头*/
    txFrameHeader.StdId = MOTOR_3508_SENT_ID;
    txFrameHeader.IDE   = CAN_ID_STD;
    txFrameHeader.RTR   = CAN_RTR_DATA;
    txFrameHeader.DLC   = 0x08;


    /*默认使用CAN1*/
    ret = HAL_CAN_AddTxMessage(&hcan1, &txFrameHeader, data, &txMailBox);

    memset(data,0,sizeof(data));

    return ret;


}
HAL_StatusTypeDef MOTOR_3508_CAN2_SENT_DATA( uint16_t data_1,uint16_t data_2,uint16_t data_3,uint16_t data_4)
{


    uint32_t txMailBox;//发送邮箱
    CAN_TxHeaderTypeDef txFrameHeader;
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t data[8];
    /*数据处理*/
    data[0]      = data_1 >> 8;//hello
    data[1]      = data_1;
    data[2]      = data_2 >> 8;
    data[3]      = data_2;
    data[4]      = data_3 >> 8;
    data[5]      = data_3;
    data[6]      = data_4 >> 8;
    data[7]      = data_4;
    /*数据帧开头*/
    txFrameHeader.StdId = MOTOR_3508_SENT_ID;
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
  * @param  motor_3508_t *
  * @retval NONE
  */
void MOTOR_3508_GET_DATA1(motor_3508_t *motor)
{

    /*更新信息 */
    motor->base_info->angle = hcan1RxFrame.data[0];
    motor->base_info->angle <<= 8;
    motor->base_info->angle |= hcan1RxFrame.data[1];
    motor->base_info->speed = hcan1RxFrame.data[2];
    motor->base_info->speed <<= 8;
    motor->base_info->speed |= hcan1RxFrame.data[3];
    motor->base_info->current = hcan1RxFrame.data[4];
    motor->base_info->current <<= 8;
    motor->base_info->current |= hcan1RxFrame.data[5];
    motor->base_info->temperature = hcan1RxFrame.data[6];

    if(motor->base_info->angle - motor->base_info->target_angle >= 4096)
    {
        motor->base_info->gap_angle = abs(motor->base_info->angle - motor->base_info->target_angle - 8096);
    } 
	else
    {
        motor->base_info->gap_angle = (motor->base_info->angle - motor->base_info->target_angle)*(-1);
    }


}
void MOTOR_3508_GET_DATA2(motor_3508_t *motor)
{

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

    if(motor->base_info->angle - motor->base_info->target_angle >= 4096)
    {
        motor->base_info->gap_angle = abs(motor->base_info->angle - motor->base_info->target_angle - 8096);
    } 
	else
    {
        motor->base_info->gap_angle = (motor->base_info->angle - motor->base_info->target_angle)*(-1);
    }


}

/**
  * @brief  检测电机是否掉线
  * @param  motor_3508_t *
  * @retval NONE
  */
void MOTOR_3508_HEART(motor_3508_t *motor)
{
	if(motor->info->offline_cnt++ >= motor->info->offline_cnt_max)
	{
		motor->info->status = _DEV_OFFLINE;
		motor->info->offline_cnt--;
	}
	else
	{
		motor->info->status = _DEV_ONLINE;
	}
}









