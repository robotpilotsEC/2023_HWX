/**
 * @file        rm_protocol.h
 * @author      RobotPilots
 * @Version     v1.0.3
 * @brief       RM Motor Protocol.
 * @update      
 *              v1.0(24-October-2021)
 *              v1.0.1(8-November-2021)
 *                  1.新增CAN电调发送报文协议
 *              v1.0.2(13-November-2021)
 *                  1.修复电机发送数据下标不正确的bug
 *              v1.0.3(13-November-2021)
 *                  1.宏定义中ID_202的值赋值错误成201，已修复
 */
 
#ifndef __RM_PROTOCOL_H
#define __RM_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

/* Exported macro ------------------------------------------------------------*/
// RM3508电机 CAN ID(RX)
#define RM3508_CAN_ID_201 (0x201U)
#define RM3508_CAN_ID_202 (0x202U)
#define RM3508_CAN_ID_203 (0x203U)
#define RM3508_CAN_ID_204 (0x204U)
#define RM3508_CAN_ID_205 (0x205U)
#define RM3508_CAN_ID_206 (0x206U)
#define RM3508_CAN_ID_207 (0x207U)
#define RM3508_CAN_ID_208 (0x208U)
// GM6020电机 CAN ID(RX)
#define GM6020_CAN_ID_205 (0x205U)
#define GM6020_CAN_ID_206 (0x206U)
#define GM6020_CAN_ID_207 (0x207U)
#define GM6020_CAN_ID_208 (0x208U)
#define GM6020_CAN_ID_209 (0x209U)
#define GM6020_CAN_ID_20A (0x20AU)
#define GM6020_CAN_ID_20B (0x20BU)
// RM2006电机 CAN ID(RX)
#define RM2006_CAN_ID_201 (0x201U)
#define RM2006_CAN_ID_202 (0x202U)
#define RM2006_CAN_ID_203 (0x203U)
#define RM2006_CAN_ID_204 (0x204U)
#define RM2006_CAN_ID_205 (0x205U)
#define RM2006_CAN_ID_206 (0x206U)
#define RM2006_CAN_ID_207 (0x207U)
#define RM2006_CAN_ID_208 (0x208U)
// RM3510电机 CAN ID(RX)
#define RM3510_CAN_ID_201 (0x201U)
#define RM3510_CAN_ID_202 (0x202U)
#define RM3510_CAN_ID_203 (0x203U)
#define RM3510_CAN_ID_204 (0x204U)
// GM3510电机 CAN ID(RX)
#define GM3510_CAN_ID_205 (0x205U)
#define GM3510_CAN_ID_206 (0x206U)
#define GM3510_CAN_ID_207 (0x207U)

/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
// RM电机 CAN电调反馈报文协议
uint16_t CAN_01_GetMotorAngle(uint8_t *rxData);
int16_t CAN_23_GetMotorSpeed(uint8_t *rxData);
int16_t CAN_45_GetMotorCurrent(uint8_t *rxData);
int16_t CAN_23_GetMotorTorque(uint8_t *rxData);
int16_t CAN_45_GetMotorTorque(uint8_t *rxData);
uint8_t CAN_6_GetMotorTemperature(uint8_t *rxData);
// RM电机 CAN电调发送报文协议
void CAN_AddMotorOutput(drv_can_t *drv, uint16_t txData);
// 兼容can_protocol v1.0(默认的CAN电调反馈报文协议)
#define CAN_GetMotorAngle           CAN_01_GetMotorAngle
#define CAN_GetMotorSpeed           CAN_23_GetMotorSpeed
#define CAN_GetMotorCurrent         CAN_45_GetMotorCurrent
#define CAN_GetMotorTemperature     CAN_6_GetMotorTemperature
// RM3508
#define RM3508_GetMotorAngle        CAN_01_GetMotorAngle
#define RM3508_GetMotorSpeed        CAN_23_GetMotorSpeed
#define RM3508_GetMotorCurrent      CAN_45_GetMotorCurrent
#define RM3508_GetMotorTemperature  CAN_6_GetMotorTemperature
uint32_t RM3508_GetTxId(drv_can_t *drv);
uint8_t RM3508_GetDataId(drv_can_t *drv);
// GM6020
#define GM6020_GetMotorAngle        CAN_01_GetMotorAngle
#define GM6020_GetMotorSpeed        CAN_23_GetMotorSpeed
#define GM6020_GetMotorCurrent      CAN_45_GetMotorCurrent
#define GM6020_GetMotorTemperature  CAN_6_GetMotorTemperature
uint32_t GM6020_GetTxId(drv_can_t *drv);
uint8_t GM6020_GetDataId(drv_can_t *drv);
// RM2006
#define RM2006_GetMotorAngle        CAN_01_GetMotorAngle
#define RM2006_GetMotorSpeed        CAN_23_GetMotorSpeed
#define RM2006_GetMotorTorque       CAN_45_GetMotorTorque
uint32_t RM2006_GetTxId(drv_can_t *drv);
uint8_t RM2006_GetDataId(drv_can_t *drv);
// RM3510
#define RM3510_GetMotorAngle        CAN_01_GetMotorAngle
#define RM3510_GetMotorSpeed        CAN_23_GetMotorSpeed
uint32_t RM3510_GetTxId(drv_can_t *drv);
uint8_t RM3510_GetDataId(drv_can_t *drv);
// GM3510
#define RM3510_GetMotorAngle        CAN_01_GetMotorAngle
#define RM3510_GetMotorTorque       CAN_23_GetMotorTorque
uint32_t GM3510_GetTxId(drv_can_t *drv);
uint8_t GM3510_GetDataId(drv_can_t *drv);

#endif
