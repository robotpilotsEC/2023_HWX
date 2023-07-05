/**
 * @file        drv_can.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        20-August-2020
 * @brief       CAN Driver Package(Based on HAL).
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

#include "rp_math.h"
#include "9015_motor.h"
#include "3508_motor.h"
#include "2006_motor.h"
#include "can_protocol.h"
#include "rp_shoot.h"
#include "judge.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Master_Head_t          Master_Head_structure;

extern motor_3508_t           motor_3508_LF_structure;
extern motor_3508_t           motor_3508_RF_structure;
extern motor_3508_t           motor_3508_LB_structure;
extern motor_3508_t           motor_3508_RB_structure;
extern motor_2006_t           L_motor_2006_PLUCK_structure;
extern motor_2006_t           R_motor_2006_PLUCK_structure;
extern shoot_t                L_shoot_structure;
extern shoot_t                R_shoot_structure;

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig);
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);
__WEAK void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
__WEAK void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);

/* Private typedef -----------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
CAN_RxFrameTypeDef hcan1RxFrame;
CAN_RxFrameTypeDef hcan2RxFrame;
CAN_TxFrameTypeDef hcan1TxFrame;
CAN_TxFrameTypeDef hcan2TxFrame;

/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/**
 *	@brief	CAN 标识符过滤器复位成默认配置
 */
static void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterIdHigh = 0;						
	sFilterConfig->FilterIdLow = 0;							
	sFilterConfig->FilterMaskIdHigh = 0;					// 不过滤
	sFilterConfig->FilterMaskIdLow = 0;						// 不过滤
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;	// 过滤器关联到FIFO0
	sFilterConfig->FilterBank = 0;							// 设置过滤器0
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// 标识符屏蔽模式
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32位宽
	sFilterConfig->FilterActivation = ENABLE;				// 激活滤波器
	sFilterConfig->SlaveStartFilterBank = 0;
}

/**
 *	@brief	CAN 接收中断回调函数
 */
static void CAN_Rx_Callback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan1RxFrame.header, hcan1RxFrame.data);
		
		CAN1_rxDataHandler(hcan1RxFrame.header.StdId, hcan1RxFrame.data);
		
		/*测试临时放这*/
		//Motor_9015_Updata(hcan1RxFrame.data);
	}
	else if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan2RxFrame.header, hcan2RxFrame.data);
		
		CAN2_rxDataHandler(hcan2RxFrame.header.StdId, hcan2RxFrame.data);
	}
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	CAN1 初始化
 */
void CAN1_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// 配置CAN标识符滤波器
	CAN_Filter_ParamsInit(&sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	
	// 使能接收中断
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// 开启CAN1
	HAL_CAN_Start(&hcan1);
}

/**
 *	@brief	CAN2 初始化
 */
void CAN2_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// 配置CAN标识符滤波器
	CAN_Filter_ParamsInit(&sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	
	// 使能接收中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// 开启CAN2
	HAL_CAN_Start(&hcan2);
}

/**
 *	@brief	通过CAN发送数据
 */
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef *txFrame;
	
	if(hcan->Instance == CAN1)
		txFrame = &hcan1TxFrame;
	else if(hcan->Instance == CAN2)
		txFrame = &hcan2TxFrame;
	else
		return HAL_ERROR;
	
	txFrame->header.StdId = stdId;
	txFrame->header.IDE = CAN_ID_STD;
	txFrame->header.RTR = CAN_RTR_DATA;
	txFrame->header.DLC = 8;
	
	// 先发高8位数据，再发低8位数据
	txFrame->data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txFrame->data[1] = (uint8_t)((int16_t)dat[0]);
	txFrame->data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txFrame->data[3] = (uint8_t)((int16_t)dat[1]);
	txFrame->data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txFrame->data[5] = (uint8_t)((int16_t)dat[2]);
	txFrame->data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txFrame->data[7] = (uint8_t)((int16_t)dat[3]);		

	if(HAL_CAN_AddTxMessage(hcan, &txFrame->header, &txFrame->data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}
/**
 *	@brief	通过CAN发送数据
 */
uint8_t CAN_u8_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef *txFrame;
	
	if(hcan->Instance == CAN1)
		txFrame = &hcan1TxFrame;
	else if(hcan->Instance == CAN2)
		txFrame = &hcan2TxFrame;
	else
		return HAL_ERROR;
	
	txFrame->header.StdId = stdId;
	txFrame->header.IDE = CAN_ID_STD;
	txFrame->header.RTR = CAN_RTR_DATA;
	txFrame->header.DLC = 8;
	
	// 先发高8位数据，再发低8位数据
	txFrame->data[0] = dat[0];
	txFrame->data[1] = dat[1];
	txFrame->data[2] = dat[2];
	txFrame->data[3] = dat[3];
	txFrame->data[4] = dat[4];
	txFrame->data[5] = dat[5];
	txFrame->data[6] = dat[6];
	txFrame->data[7] = dat[7];	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame->header, &txFrame->data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

uint8_t CAN1_SendData(uint32_t stdId, int16_t *dat)
{
	return CAN_SendData(&hcan1, stdId, dat);
}

uint8_t CAN2_SendData(uint32_t stdId, int16_t *dat)
{
	return CAN_SendData(&hcan2, stdId, dat);
}

/* Callback functions --------------------------------------------------------*/
/**
 *	@brief	重写 CAN RxFifo 中断接收函数
 *	@note	在stm32f4xx_hal_can.c中弱定义
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* CAN1 接收中断 */
	if(hcan->Instance == CAN1)
	{
		CAN_Rx_Callback(hcan);
		// HAL_CAN_DeactivateNotification
		// __HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	// 暂停开启FIF00消息挂号中断，在消息处理任务中处理完成后再使能
	}else
	if(hcan->Instance == CAN2)
	{
		CAN_Rx_Callback(hcan);
	}
}

/* rxData Handler [Weak] functions -------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 CAN1 处理协议
 */
uint8_t motor_9015_init_ok = 0;
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	/*BIG YAW*/
	if (canId == 0x141) //9015
	{
		Motor_9015_Updata(hcan1RxFrame.data);

	}
	else if(canId == 0x202)//3508
	{
		motor_3508_RF_structure.info->offline_cnt=0;
		Motor_3508_GetData1(&motor_3508_RF_structure);
	}
	/*LF 2 /1*/
	else if(canId == 0x201)//3508
	{
		motor_3508_LF_structure.info->offline_cnt=0;
		Motor_3508_GetData1(&motor_3508_LF_structure);
	}
	/*LB 3 */
	else if(canId == 0x203)//3508
	{
		motor_3508_LB_structure.info->offline_cnt=0;
		Motor_3508_GetData1(&motor_3508_LB_structure);
	}
	/*RB 4 */
 	else if(canId == 0x204)//3508
	{
		motor_3508_RB_structure.info->offline_cnt=0;
		Motor_3508_GetData1(&motor_3508_RB_structure);
	}
	/*Judge*/
//	else if(canId == power_heat_data || canId == game_robot_status ||\
//		    canId == shoot_data      || canId == game_robot_pos    ||\
//			canId == robot_hurt      || canId == game_robot_HP)
//	{
//		Judge_Get_Data(canId,rxBuf);
//	}
}
/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 CAN2 处理协议
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	if(canId == FROM_L_HEAD_ID|| canId == FROM_R_HEAD_ID)
	{
		H2M_GETT_DATA(canId, &Master_Head_structure, rxBuf);
	}
	else if(canId == 0x201)//2006
	{
		L_motor_2006_PLUCK_structure.info->offline_cnt=0;
		Motor_2006_GetData(&L_motor_2006_PLUCK_structure);
		
		if(L_shoot_structure.status == Single_Shoot)
		L_motor_2006_PLUCK_structure.base_info->angle_sum += L_motor_2006_PLUCK_structure.base_info->angle_add;
	}
		else if(canId == 0x202)//2006
	{
		R_motor_2006_PLUCK_structure.info->offline_cnt=0;
		Motor_2006_GetData(&R_motor_2006_PLUCK_structure);
		
		if(R_shoot_structure.status == Single_Shoot)
		R_motor_2006_PLUCK_structure.base_info->angle_sum += R_motor_2006_PLUCK_structure.base_info->angle_add;
	
		if(motor_9015_init_ok == 0)
		{
			Motor_9015_Star();
			motor_9015_init_ok = 1;
		}
	}
	
}
