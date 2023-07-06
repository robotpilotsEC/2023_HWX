#include "rp_gimbal.h"
#include "rp_shoot.h"
#include "6020_motor.h"
#include "can_protocol.h"


extern motor_6020_t           motor_6020_YAW_structure;
extern motor_6020_t           motor_6020_PIT_structure;
extern Master_Head_t          Master_Head_structure;
extern gimbal_t               gimbal_structure;
void Gimbal_work()
{
	/*µç»úµôÏß¼ì²â*/
	MOTOR_6020_HEART(&motor_6020_YAW_structure);
	MOTOR_6020_HEART(&motor_6020_PIT_structure);
	
	Gimbal_Heart(&gimbal_structure);


}

