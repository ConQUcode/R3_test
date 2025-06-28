#include "shoot_ex.h"
#include "bsp_dwt.h"
#include "remote.h"

//static RC_ctrl_t *rc_cmd;


//Ctrl_Cmd_s ctrl_cmd;


//void Shoot_ex_Init()
//{
//	//开启高精度计数计时器
//	DWT_Init(64);
//	
//	//开启遥控器接收
//	rc_cmd = RemoteControlInit(&huart3);

//}

//void GetCmd(){
//	ctrl_cmd.vx = ((float)rc_cmd->rc.rocker_l_/660)*100000;
//	ctrl_cmd.vy = ((float)rc_cmd->rc.rocker_l1/660)*100000;
//	
//	ctrl_cmd.vw = ((float)rc_cmd->rc.dial/660)*100000;
//	
//	ctrl_cmd.vshoot = ((float)rc_cmd->rc.rocker_r1/660)*12000;
//	ctrl_cmd.ban_mode = rc_cmd->rc.switch_left;
//}



//void Shoot_ex_Task()
//{

//	GetCmd();

//	HAL_Delay(10);

//}
