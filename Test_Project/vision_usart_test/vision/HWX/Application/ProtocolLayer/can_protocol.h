/**
 * @file        can_protocol.h
 * @author      RobotPilots
 * @Version     v1.1.2
 * @brief       CAN Protocol.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(24-October-2021)
 *                  1.修改can_potocol.c/.h->can_protocol.c/.h
 *              v1.1.1(8-November-2021)
 *                  1.新增can报文消息发送协议 
 *              v1.1.2(13-November-2021)
 *                  1.去掉add_halfword与add_word函数中冗余的强制转换，另外原先
 *                    add_word中存在转换错误的问题，现已修复
 */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "rm_protocol.h"
/* Exported macro ------------------------------------------------------------*/
// CAN1
#define CHASSIS_CAN_ID_LF	RM3508_CAN_ID_201
#define CHASSIS_CAN_ID_RF	RM3508_CAN_ID_202
#define CHASSIS_CAN_ID_LB	RM3508_CAN_ID_203
#define CHASSIS_CAN_ID_RB	RM3508_CAN_ID_204
// CAN2

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void CAN_AddMsg(drv_can_t *drv, uint8_t *data, uint8_t data_cnt);
void CAN_AddByte(drv_can_t *drv, uint8_t byte);
void CAN_AddHalfWord(drv_can_t *drv, uint16_t hword);
void CAN_AddWord(drv_can_t *drv, uint32_t word);
void CAN_ManualTx(drv_can_t *drv, uint8_t *data);

#endif
