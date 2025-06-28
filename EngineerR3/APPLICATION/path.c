#include "path.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"


extern  RC_ctrl_t *rc_cmd;
static  DJIMotor_Instance *M3508_IN_LEFT , *M3508_IN_RIGHT , *M3508_UP , *M2006_ADJUST;
float check_current;//2006 current check 0 point
#define RC_DEADBAND 10
#define CURRENT_LIMIT 2000
// 窮字堀業�渣�
#define MAX_SPEED_01 5000
#define MAX_SPEED_2 5000
#define MAX_SPEED_3 5000

void path_init()
{
	Motor_Init_Config_s M3508_config = {
    .can_init_config = {
        .can_handle = &hcan1,
        .tx_id      = 6,
    },
    .controller_param_init_config = {
        .speed_PID = {
            .Kp = 4,
            .Ki = 0.025,
            .Kd = 0.02,
            .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 10000,
            .MaxOut        = 15000,
        },
        .current_PID = {
            .Kp            = 0.5,
            .Ki            = 0.01,
            .Kd            = 0,
            .Improve       = PID_Integral_Limit,
            .IntegralLimit = 10000,
            .MaxOut        = 15000,
        },
    },
    .controller_setting_init_config = {
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type       = SPEED_LOOP, 
        .close_loop_type       = CURRENT_LOOP | SPEED_LOOP,
        .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
        .feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
    },
    .motor_type = M3508
};
	

	M3508_IN_LEFT = DJIMotorInit(&M3508_config);
	    
  M3508_config.can_init_config.tx_id = 7;
  M3508_IN_RIGHT = DJIMotorInit(&M3508_config);

	M3508_config.can_init_config.can_handle=&hcan2;
	M3508_config.can_init_config.tx_id = 6;//吉棋耶紗液桟才叔業協吶
  M3508_UP = DJIMotorInit(&M3508_config);

	 Motor_Init_Config_s M2006_config = {
     .can_init_config = {
         .can_handle = &hcan1,
         .tx_id      = 8,
     },
     .controller_param_init_config = {
         .speed_PID = {
             .Kp = 4,
             .Ki = 0.025,
             .Kd = 0.02,
             .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
             .IntegralLimit = 10000,
             .MaxOut        = 15000,
         },
         .current_PID = {
             .Kp            = 0.5,
             .Ki            = 0.01,
             .Kd            = 0,
             .Improve       = PID_Integral_Limit,
             .IntegralLimit = 1000,
             .MaxOut        = 15000,
         },

         .current_feedforward_ptr = &check_current,
     },
     .controller_setting_init_config = {
         .speed_feedback_source = OTHER_FEED,
         .outer_loop_type       = SPEED_LOOP, 
         .close_loop_type       = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
         .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
         .feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
     },
     .motor_type = M2006
 };
 M2006_ADJUST = DJIMotorInit(&M2006_config);
 
}
void xianwei_check(DJIMotor_Instance *motor)
{
	if(motor->measure.real_current>=CURRENT_LIMIT)
	{
		DJIMotorStop(motor);
	}
	else
	{
		DJIMotorEnable(motor);
	}
	
}
//！！！！！！！！！！！！！！！！！！！！！！！！！！！！箕幅何蛍陣崙！！！！！！！！！！！！！！！！！！！！！！//
void PATH_CMD_Task()
{
	float speed;
	speed = (float)rc_cmd->rc.rocker_r1;
			if(speed > 0)
		{
			
			DJIMotorEnable(M3508_UP);
			xianwei_check(M3508_UP);
			DJIMotorSetRef(M3508_UP,((float)rc_cmd->rc.rocker_r1/660)*100000);
		}
		else if (speed < 0)
		{
			DJIMotorEnable(M3508_UP);
			xianwei_check(M3508_UP);
			DJIMotorSetRef(M3508_UP,((float)rc_cmd->rc.rocker_r1/660)*100000);
		}


}

//！！！！！！！！！！！！！！！！！！！！！！！！！！！！趣白何蛍陣崙！！！！！！！！！！！！！！！！！！！！！！//
void get_cmd_task()
{
	float speed;
	speed = (float)rc_cmd->rc.rocker_r_;
	if(speed > 0)
		{
			
			DJIMotorEnable(M2006_ADJUST);
			xianwei_check(M2006_ADJUST);
			DJIMotorEnable(M3508_IN_LEFT);
			DJIMotorEnable(M3508_IN_RIGHT);
			
			DJIMotorSetRef(M2006_ADJUST,((float)rc_cmd->rc.rocker_r_/660)*10000);
			
			DJIMotorSetRef(M3508_IN_LEFT,((float)rc_cmd->rc.rocker_r_/660)*50000);
			DJIMotorSetRef(M3508_IN_RIGHT,((float)rc_cmd->rc.rocker_r_/660)*50000);
		}
	else if (speed < 0)
		{
			
			DJIMotorEnable(M2006_ADJUST);
			xianwei_check(M2006_ADJUST);
			DJIMotorEnable(M3508_IN_LEFT);
			DJIMotorEnable(M3508_IN_RIGHT);
			
			DJIMotorSetRef(M2006_ADJUST,((float)rc_cmd->rc.rocker_r_/660)*10000);
			
			DJIMotorSetRef(M3508_IN_LEFT,((float)rc_cmd->rc.rocker_r_/660)*50000);
			DJIMotorSetRef(M3508_IN_RIGHT,((float)rc_cmd->rc.rocker_r_/660)*50000);
		}
		
	
}


//----------------------------------------------------------悳旗鷹陣崙--------------------------------------------------//

void path_all_cmd_Task()
{
	PATH_CMD_Task();
	get_cmd_task();
}





















