#ifndef __DRV_CAN_H
#define __DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t 			data[8];
} CAN_RxFrameTypeDef;

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_TxFrameTypeDef;

/* Exported functions --------------------------------------------------------*/
void CAN1_Init(void);
void CAN2_Init(void);
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat);
uint8_t CAN1_SendData(uint32_t stdId, int16_t *dat);
uint8_t CAN2_SendData(uint32_t stdId, int16_t *dat);
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, int16_t *dat);
uint8_t CAN_u8_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat);
#endif
