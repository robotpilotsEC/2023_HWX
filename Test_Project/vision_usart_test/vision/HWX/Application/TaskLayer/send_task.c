/**
 * @file        send_task.c
 * @author      RobotPilots
 * @Version     V1.0
 * @brief       Periodic Send Task.
 * @update      
 *              v1.0(8-November-2021)
 */

/* Includes ------------------------------------------------------------------*/
#include "send_task.h"

#include "drv_can.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	周期发送任务
 */
uint32_t SendFlag = 0;
void StartSendTask(void const * argument)
{
    BaseType_t ret;
    
	for(;;)
	{
        ret = xTaskNotifyWait((uint32_t)0x00,
                              (uint32_t)0xFFFFFFFF,
                              (uint32_t*)&SendFlag,
                              (TickType_t)portMAX_DELAY);
        if(ret == pdPASS)
        {
            if(SendFlag & EVENT_SEND_CAN1_MAILBOX) {
                CAN_AutoTx(&hcan1Mailbox);
            }
            
            if(SendFlag & EVENT_SEND_CAN2_MAILBOX) {
                CAN_AutoTx(&hcan2Mailbox);
            }
        }
        
        //osDelay(1);
	}
}
