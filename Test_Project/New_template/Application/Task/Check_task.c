#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "driver.h"
//#include "9015_motor.h"



//extern motor_9015_t           motor_9015_structure;
//extern motor_9015_info_t      motor_9015_info_structure;
//extern motor_9015_base_info_t motor_9015_base_info_structure;
//extern motor_9015_pid_t       motor_9015_pid_structure;
int32_t test_speed = 5000;
int32_t test_angle = 1000;
uint8_t test_flag  = 1;

/*红 -- 底盘
  绿 -- 云台
  蓝 -- 发射机构
  橙 -- 其他
             */


void CheckTask(void const * argument)
{
	while(1)
	{
//		LED_ORANGE_TOGGLE();
//		osDelay(100);
//		
//		LED_BLUE_TOGGLE();
//		osDelay(100);
//		
//		LED_RED_TOGGLE();
//		osDelay(100);
//		
//		LED_GREEN_TOGGLE();
//		osDelay(100);
		
//		motor_9015_structure.base_info->target_speed = test_speed;
//		motor_9015_structure.base_info->target_angle = test_angle;
//		
//		MOTOR_9015_HEART();
//		MOTOR_9015_SPEED();
		
		//MOTOR_9015_DATA(CIRCLE_ANGLE_ID);//这句话有大问题，加了之后会卡
		
		
		osDelay(1);
		
		  
	}


}

void BmiUpdateTask(void const * argument)
{
	while(1)
	{
		
	
		osDelay(1);
	}

}
