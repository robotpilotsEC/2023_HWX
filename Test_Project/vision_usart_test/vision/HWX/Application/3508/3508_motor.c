
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














#if 0
/**
  * @brief
  * @param
  * @retval
  */
void motor_3508_info_init(motor_3508_info_t *info)
{
    info->offline_cnt_max = 5;
    info->offline_cnt = 5;
    info->status = DEV_OFFLINE;
    info->target_angle_sum =                                                                                                                                                                                                  0;
}





//	/* angle_add */
//	base_info->angle_add = base_info->angle - last_angle;
//	if(abs(base_info->angle_add) > 4096)
//	{
//		base_info->angle_add -= 8192 * one(base_info->angle_add);
//	}

//	/* angle_sum and target_angle_sum */
//	base_info->angle_sum += base_info->angle_add;
//	if(abs(base_info->angle_sum) > 0x0FFF)
//	{
//		base_info->angle_sum -= 0x0FFF * one(base_info->angle_sum);
//		motor->info->target_angle_sum -= 0x0FFF * one(base_info->angle_sum);
//	}

//	/* flag */
//	motor->info->offline_cnt = 0;
//	motor->info->status = DEV_ONLINE;


/**
  * @brief
  * @param
  * @retval
  */
void motor_3508_speed_ctrl(motor_3508_t *motor)
{
    int16_t output;
    motor->pid_speed->info->measure = motor->base_info->speed;
    single_pid_cal(motor->pid_speed->info);
    output = motor->pid_speed->info->out;
    if(motor->can->hcan == &hcan1)
    {
        can1_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
        can1_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
    }
    else if(motor->can->hcan == &hcan2) {
        can2_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
        can2_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
    }
}

/**
  * @brief
  * @param
  * @retval
  */
void motor_3508_angle_ctrl(motor_3508_t *motor)
{
    motor->pid_angle->info->target = motor->info->target_angle_sum;
    motor->pid_angle->info->measure = motor->base_info->angle_sum;
    single_pid_cal(motor->pid_angle->info);
    motor->pid_speed->info->target = motor->pid_angle->info->out;
    motor_3508_speed_ctrl(motor);
}
#endif

///**
//  * @brief  CAN发送函数
//  * @param
//  * @retval
//  */
//uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat)
//{
//	CAN_TxHeaderTypeDef pHeader;
//	uint32_t txMailBox;
//
//	//判断can有效性
//	if((hcan->Instance != CAN1)&&(hcan->Instance != CAN2))
//	{
//		return HAL_ERROR;
//	}
//
//	//配置帧头
//	pHeader.StdId = stdId;
//	pHeader.IDE = CAN_ID_STD;
//	pHeader.RTR = CAN_RTR_DATA;
//	pHeader.DLC = 8;
//
//	//判断发送成功与否
//	if(HAL_CAN_AddTxMessage(hcan, &pHeader, dat, &txMailBox) != HAL_OK)
//	{
//		return HAL_ERROR;
//	}
//
//	return HAL_OK;
//}
//HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
//{
//  HAL_CAN_StateTypeDef state = hcan->State;

//  assert_param(IS_CAN_RX_FIFO(RxFifo));

//  if ((state == HAL_CAN_STATE_READY) ||
//      (state == HAL_CAN_STATE_LISTENING))
//  {
//    /* Check the Rx FIFO */
//    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
//    {
//      /* Check that the Rx FIFO 0 is not empty */
//      if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) == 0U)
//      {
//        /* Update error code */
//        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

//        return HAL_ERROR;
//      }
//    }
//    else /* Rx element is assigned to Rx FIFO 1 */
//    {
//      /* Check that the Rx FIFO 1 is not empty */
//      if ((hcan->Instance->RF1R & CAN_RF1R_FMP1) == 0U)
//      {
//        /* Update error code */
//        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

//        return HAL_ERROR;
//      }
//    }

//    /* Get the header */
//    pHeader->IDE = CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[RxFifo].RIR;
//    if (pHeader->IDE == CAN_ID_STD)
//    {
//      pHeader->StdId = (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> CAN_TI0R_STID_Pos;
//    }
//    else
//    {
//      pHeader->ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> CAN_RI0R_EXID_Pos;
//    }
//    pHeader->RTR = (CAN_RI0R_RTR & hcan->Instance->sFIFOMailBox[RxFifo].RIR);
//    pHeader->DLC = (CAN_RDT0R_DLC & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
//    pHeader->FilterMatchIndex = (CAN_RDT0R_FMI & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_FMI_Pos;
//    pHeader->Timestamp = (CAN_RDT0R_TIME & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_TIME_Pos;

//    /* Get the data */
//    aData[0] = (uint8_t)((CAN_RDL0R_DATA0 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA0_Pos);
//    aData[1] = (uint8_t)((CAN_RDL0R_DATA1 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA1_Pos);
//    aData[2] = (uint8_t)((CAN_RDL0R_DATA2 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA2_Pos);
//    aData[3] = (uint8_t)((CAN_RDL0R_DATA3 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA3_Pos);
//    aData[4] = (uint8_t)((CAN_RDH0R_DATA4 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA4_Pos);
//    aData[5] = (uint8_t)((CAN_RDH0R_DATA5 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA5_Pos);
//    aData[6] = (uint8_t)((CAN_RDH0R_DATA6 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA6_Pos);
//    aData[7] = (uint8_t)((CAN_RDH0R_DATA7 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA7_Pos);

//    /* Release the FIFO */
//    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
//    {
//      /* Release RX FIFO 0 */
//      SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
//    }
//    else /* Rx element is assigned to Rx FIFO 1 */
//    {
//      /* Release RX FIFO 1 */
//      SET_BIT(hcan->Instance->RF1R, CAN_RF1R_RFOM1);
//    }

//    /* Return function status */
//    return HAL_OK;
//  }
//  else
//  {
//    /* Update error code */
//    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

//    return HAL_ERROR;
//  }
//}
