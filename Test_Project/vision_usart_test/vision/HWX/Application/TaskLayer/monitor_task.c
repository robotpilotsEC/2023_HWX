/**
 * @file        monitor_task.c
 * @author      RobotPilots
 * @Version     V1.0.1
 * @brief       Monitor&Test Center
 * @update      
 *              v1.0(9-November-2020)
 *              v1.0.1(13-November-2021)
 *                  1.添加任务通知机制，event_notify()
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor_task.h"

#include "drv_io.h"
#include "drv_can.h"
#include "drv_haltick.h"
#include "device.h"
#include "rp_math.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t send_control_time = 0;
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void device_heart_beat(void)
{
	rc_sensor.heart_beat(&rc_sensor);
	imu_sensor.heart_beat(&imu_sensor);
	chassis_motor[CHAS_LF].heart_beat(&chassis_motor[CHAS_LF]);
	chassis_motor[CHAS_RF].heart_beat(&chassis_motor[CHAS_RF]);
	chassis_motor[CHAS_LB].heart_beat(&chassis_motor[CHAS_LB]);
	chassis_motor[CHAS_RB].heart_beat(&chassis_motor[CHAS_RB]);
}

static void system_led_flash(void)
{
	static uint16_t led_blue_flash = 0;
	
	led_blue_flash++;
	if(led_blue_flash > 500) 
	{
		led_blue_flash = 0;
		LED_BLUE_TOGGLE();
	}
}

extern osThreadId SendTaskHandle;
uint32_t event = 0;
static void event_notify(void)
{
    uint32_t can_send_event = 0;
    
    if(CAN_MailboxReadyForTx(&hcan1Mailbox)) {
        SET_EVENT(event, EVENT_SEND_CAN1_MAILBOX);
    }
    
    if(CAN_MailboxReadyForTx(&hcan2Mailbox)) {
        SET_EVENT(event, EVENT_SEND_CAN2_MAILBOX);
    }
    
    can_send_event = GET_EVENT(event, EVENT_SEND_CAN1_MAILBOX|EVENT_SEND_CAN2_MAILBOX);
    if(can_send_event != 0) {
        send_control_time = micros();
        xTaskNotify((TaskHandle_t)SendTaskHandle,
                    (uint32_t)can_send_event,
                    (eNotifyAction)eSetBits);
        
        CLEAR_EVENT(event, can_send_event);
    }
}

/* Exported functions --------------------------------------------------------*/
/**
 *	@brief	系统监控任务
 */
void StartMonitorTask(void const * argument)
{
	//LED_RED_ON();
	for(;;)
	{
        taskENTER_CRITICAL();
		imu_sensor.update(&imu_sensor);
        taskEXIT_CRITICAL();
        
		system_led_flash();
		device_heart_beat();
        event_notify();
        
		osDelay(1);
	}
}
