#include "dribble.h"
#include "DJI_motor.h"
#include "remote.h"
#include "bsp_dwt.h"
#include "robot_def.h"
static DJIMotor_Instance *motor_3508_1, *motor_3508_2, *motor_3508_3, *motor_3508_4, *motor_6020;
static float totalangle = 0;
static int8_t is_init = 0;

void DribbleInit()
{
		DWT_Init(168);
	
    Motor_Init_Config_s M3508_zhuan_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4,  // 10
                .Ki = 0.025, // 1
                .Kd = 0.02,
                // .CoefA         = 0.2,
                // .CoefB         = 0.3,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
            },
            .current_PID = {
                .Kp            = 0.5, // 0.7
                .Ki            = 0.01, // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 1000,
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
		
		motor_3508_1 = DJIMotorInit(&M3508_zhuan_config);
			
		M3508_zhuan_config.can_init_config.tx_id = 2;
		motor_3508_2 =  DJIMotorInit(&M3508_zhuan_config);
				
		M3508_zhuan_config.can_init_config.tx_id = 3;
		motor_3508_3 =  DJIMotorInit(&M3508_zhuan_config);
				
			Motor_Init_Config_s M3508_jiaodu_config = {
			.can_init_config = {
					.can_handle = &hcan1,
					.tx_id      = 4,
			},
			.controller_param_init_config = {
					.angle_PID = {
                .Kp                = 6,
                .Ki                = 0.1,
                .Kd                = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle,
                .IntegralLimit     = 5000,
                .MaxOut            = 15000,
                .Derivative_LPF_RC = 0.01,
            },
					.speed_PID = {
							.Kp = 9,  // 10
							.Ki = 0.2, // 1
							.Kd = 0.02,
							// .CoefA         = 0.2,
							// .CoefB         = 0.3,
							.Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
							.IntegralLimit = 5000,
							.MaxOut        = 15000,
					},
					.current_PID = {
							.Kp            = 0.5, // 0.7
							.Ki            = 0.01, // 0.1
							.Kd            = 0,
							.Improve       = PID_Integral_Limit,
							.IntegralLimit = 1000,
							.MaxOut        = 15000,
							// .DeadBand      = 0.1,
					},
					.other_angle_feedback_ptr = &totalangle,
			},
					.controller_setting_init_config = {
					.angle_feedback_source = OTHER_FEED,
					.speed_feedback_source = MOTOR_FEED,
					.outer_loop_type       = ANGLE_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
					.close_loop_type       = CURRENT_LOOP | SPEED_LOOP |ANGLE_LOOP,
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
						.feedforward_flag      = FEEDFORWARD_NONE,
			},
			.motor_type = M3508};
				
		
		motor_3508_4 =  DJIMotorInit(&M3508_jiaodu_config);
//		DJIMotorOuterLoop(motor_3508_4, ANGLE_LOOP);		
				

    Motor_Init_Config_s GM6020_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 10,
                .Ki                = 10,
                .Kd                = 0.5,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle,
                .IntegralLimit     = 10000,
                .MaxOut            = 15000,
                .Derivative_LPF_RC = 0.01f,
                .DeadBand          = 1,
            },
            .speed_PID = {
                .Kp            = 1,  // 10
                .Ki            = 60, // 1
                .Kd            = 0.001f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement ,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
            },
            .current_PID = {
                .Kp            = 2.5f,  // 0.7
                .Ki            = 0.12f, // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
							
							
							
							
            },

//            .other_angle_feedback_ptr = &roll_real,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type    = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // ！！！ 只有在只对速度闭环时才能反向 ！！！
        },
        .motor_type = GM6020};
			motor_6020 = DJIMotorInit(&GM6020_config);
			 
}

//收球的M3508初始化，以及加上限幅
static void LiftInit(){
	if(!is_init){
		
		if (abs(motor_3508_4->measure.real_current) <= 4500 && is_init == 0) {
        DJIMotorEnable(motor_3508_4);
        DJIMotorOuterLoop(motor_3508_4, SPEED_LOOP);
        DJIMotorSetRef(motor_3508_4, -1000);
    } else if ((abs(motor_3508_4->measure.real_current) > 4500) && is_init == 0) {
				HAL_Delay(5);
			 if ((motor_3508_4->measure.real_current < -4500) && is_init == 0){	 
					DJIMotorOuterLoop(motor_3508_4, ANGLE_LOOP);
					DJIMotorReset(motor_3508_4);
					DJIMotorStop(motor_3508_4);
					is_init = 1;
			 }
    }

	}
}


void DribbleTask(){
//	int speed =14000;
	DJIMotorSetRef(motor_3508_1,500);
//	DJIMotorSetRef(motor_3508_2,speed+1000);
//	DJIMotorSetRef(motor_3508_3,speed+1000);
	
//	DJIMotorSetRef(motor_6020,20);
	
	DJIMotorControl();
	LiftInit();
	if(is_init){
		DJIMotorEnable(motor_3508_4);
		totalangle = motor_3508_4->measure.total_angle;
		DJIMotorSetRef(motor_3508_4,-50);
	}

}	
void Dribblelift(){
	totalangle = motor_3508_4->measure.total_angle;
	DJIMotorSetRef(motor_3508_4,2500);
	DJIMotorControl();
}
