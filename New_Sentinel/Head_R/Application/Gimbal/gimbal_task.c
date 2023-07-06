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
	/*电机掉线检测*/
	MOTOR_6020_HEART(&motor_6020_YAW_structure);
	MOTOR_6020_HEART(&motor_6020_PIT_structure);
	
	/*云台状态更新*/
	Gimbal_Heart(&gimbal_structure);

	

	//gimbal_work(&gimbal_structure);
}

void Gimbal_Heart(gimbal_t* gimbal)
{
	if(gimbal->pitch->info->status == _DEV_OFFLINE|| \
	   gimbal->yaw->info->status == _DEV_OFFLINE )//Master_Head_structure.Send_R_Head.gimbal_mode == 0
	{
		gimbal->work.status = Gimbal_Offline;
	}
	else
	{
		gimbal->work.status = Gimbal_Online;
	}
}

