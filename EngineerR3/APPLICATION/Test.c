#include "chassis.h"
#include "shoot.h"
#include "dribble.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "path.h"
#include "tuiqiu.h"

extern RC_ctrl_t *rc_cmd;	



//-----------------------------------------总体的初始化函数----------------------------
void all_init_Task(){
	DribbleInit();
	//shoot_init_Task();
	Chassis_Init();
//	 TuiQiu_Init();
	//path_init();
}

//-----------------------------------------总体的执行函数-------------------------------
void all_cmd_Task(){
	if((uint8_t)rc_cmd->rc.switch_left == (uint8_t)1)
	{
			
		Chassis_Task();
//		shoot_cmd_Task();
    //TuiQiu_cmd_Task();		
	}
		else if((uint8_t)rc_cmd->rc.switch_left == (uint8_t)3)
	{
		Chassis_Task();
		//path_all_cmd_Task();
	}
	else 	if((uint8_t)rc_cmd->rc.switch_left == (uint8_t)2)
	{
		Chassis_Task();
		//DribbleTask();		
	}
		
	Tuiqiuplay();
	DJIMotorControl();
}
