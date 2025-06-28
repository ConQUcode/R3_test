#include "chassis.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "miniPC_process.h"


RC_ctrl_t *rc_cmd;
static DJIMotor_Instance *chassis_lf, *chassis_lb, *chassis_rf, *chassis_rb;
attitude_t *Chassis_IMU_data; 

Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
static PID_Instance chassis_follow_pid;  // 底盘跟随PID
static Nac_Recv_s *nac_ctrl; // 视觉控制信息


void Chassis_Init()
{
	//开启高精度计数计时器
	DWT_Init(180);
	
	//开启遥控器接收
	rc_cmd = RemoteControlInit(&huart3);
	
	Motor_Init_Config_s M3508_chassis_config = {
			.can_init_config = {
					.can_handle = &hcan1,
					.tx_id      = 2,
			},
			.controller_param_init_config = {
					.speed_PID = {
							.Kp = 3,  // 7
							.Ki = 0.005,// 0.01
							.Kd = 0.004,//0.008
							// .CoefA         = 0.2,
							// .CoefB         = 0.3,
							.Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
							.IntegralLimit = 10000,
							.MaxOut        = 15000,
					},
					.current_PID = {
							.Kp            = 0.2, // 0.4
							.Ki            = 0.0006, // 0.001
							.Kd            = 0,
							.Improve       = PID_Integral_Limit,
							.IntegralLimit = 10000,
							.MaxOut        = 15000,
							// .DeadBand      = 0.1,
					},
			},
			.controller_setting_init_config = {
					.speed_feedback_source = MOTOR_FEED,
					.outer_loop_type       = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
					.close_loop_type       = CURRENT_LOOP | SPEED_LOOP  ,
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
					.feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
			},
			.motor_type = M3508};
		
		chassis_lf = DJIMotorInit(&M3508_chassis_config);
			
		M3508_chassis_config.can_init_config.tx_id = 4;
		chassis_lb =  DJIMotorInit(&M3508_chassis_config);
			
			
			
		M3508_chassis_config.controller_param_init_config.speed_PID.Kp=8;
		M3508_chassis_config.can_init_config.tx_id = 1;
		chassis_rf =  DJIMotorInit(&M3508_chassis_config);
				
		M3508_chassis_config.can_init_config.tx_id = 3;
		chassis_rb =  DJIMotorInit(&M3508_chassis_config);
		
		//同一电机转向
		chassis_lf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
		chassis_lb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
		chassis_rf->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
		chassis_rb->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

		PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 520, // 6
        .Ki                = 0.1f,
        .Kd                = 17, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 500, // 200
        .MaxOut            = 10000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);
		
		Chassis_IMU_data                 = INS_Init(); // 底盘IMU初始化
		nac_ctrl     = NacInit(&huart1); // 初始化视觉控制
		
}

/**
 * @brief 使舵电机角度最小旋转，取优弧，防止电机旋转不必要的行程
 *          例如：上次角度为0，目标角度为135度，
 *          电机会选择逆时针旋转至-45度，而不是顺时针旋转至135度，
 *          两个角度都会让轮电机处于同一平行线上
 *
 * @param angle 目标角度
 * @param last_angle 上次角度
 *
 */
static void MinmizeRotation(float *angle, const float *last_angle, float *speed)
{
    float rotation = *angle - *last_angle;
    if (rotation > 180) {
        *angle -= 360;
//        *speed = -(*speed);
    } else if (rotation < -180) {
        *angle += 360;
//        *speed = -(*speed);
    }
}

//---------------------------------麦轮底盘运动代码--------------------------------------

void GetCmd(){
	chassis_ctrl_cmd.vx = ((float)rc_cmd->rc.rocker_l_/660)*100000;
	chassis_ctrl_cmd.vy = ((float)rc_cmd->rc.rocker_l1/660)*100000;
	
	chassis_ctrl_cmd.vw = ((float)rc_cmd->rc.dial/660)*100000;
	
	if(chassis_ctrl_cmd.vw != 0){
		chassis_ctrl_cmd.last_yaw = Chassis_IMU_data->Yaw;
		chassis_ctrl_cmd.offset_w = 0;
	}
	if(chassis_ctrl_cmd.vw == 0){
		MinmizeRotation(&Chassis_IMU_data->Yaw,&chassis_ctrl_cmd.last_yaw,&chassis_ctrl_cmd.offset_w);
		chassis_ctrl_cmd.offset_w = PIDCalculate(&chassis_follow_pid,Chassis_IMU_data->Yaw,chassis_ctrl_cmd.last_yaw);
		
	}

}


//------------------------------底盘运动task-----------------------------------------------
void Chassis_Task()
{

	GetCmd();
	
	if(chassis_ctrl_cmd.vx == 0&&chassis_ctrl_cmd.vy == 0&&chassis_ctrl_cmd.vw ==0){
	
		DJIMotorStop(chassis_lf);
		DJIMotorStop(chassis_rf);
		DJIMotorStop(chassis_lb);
		DJIMotorStop(chassis_rb);
	
	}
	else{
	
		DJIMotorEnable(chassis_lf);
		DJIMotorEnable(chassis_rf);
		DJIMotorEnable(chassis_lb);
		DJIMotorEnable(chassis_rb);
	
	}
	DJIMotorSetRef(chassis_lf, chassis_ctrl_cmd.vy + chassis_ctrl_cmd.vx - chassis_ctrl_cmd.vw - chassis_ctrl_cmd.offset_w);
	
	DJIMotorSetRef(chassis_rf, chassis_ctrl_cmd.vy - chassis_ctrl_cmd.vx + chassis_ctrl_cmd.vw + chassis_ctrl_cmd.offset_w);
	
	DJIMotorSetRef(chassis_lb, chassis_ctrl_cmd.vy - chassis_ctrl_cmd.vx - chassis_ctrl_cmd.vw - chassis_ctrl_cmd.offset_w);
	
	DJIMotorSetRef(chassis_rb, chassis_ctrl_cmd.vy + chassis_ctrl_cmd.vx + chassis_ctrl_cmd.vw + chassis_ctrl_cmd.offset_w);
	
//	DJIMotorSetRef(chassis_lf,chassis_ctrl_cmd.vx - chassis_ctrl_cmd.vy + chassis_ctrl_cmd.vw);
//	
//	DJIMotorSetRef(chassis_rf,chassis_ctrl_cmd.vx + chassis_ctrl_cmd.vy + chassis_ctrl_cmd.vw);
//	
//	DJIMotorSetRef(chassis_lb,-chassis_ctrl_cmd.vx - chassis_ctrl_cmd.vy + chassis_ctrl_cmd.vw);
//	
//	DJIMotorSetRef(chassis_rb,-chassis_ctrl_cmd.vx + chassis_ctrl_cmd.vy + chassis_ctrl_cmd.vw);
//	
	//HAL_Delay(5);

}
