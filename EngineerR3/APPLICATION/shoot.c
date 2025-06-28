#include "shoot.h"
#include "bujin.h"
#include "daemon.h"
#include "remote.h"
#include "feite_motor.h"
#include "bsp_dwt.h"

#include "DJI_motor.h"


int target_force;
int init_force;
extern int buffer;

StepperMotor* my_motor;
FTMotor_instance *servo_left;
FTMotor_instance *servo_right;
DJIMotor_Instance *shoot_M3508;

extern RC_ctrl_t *rc_cmd;
//！！！！！！！！！！！！！！！！！！！！！！化序窮字廣過參式兜兵晒！！！！！！！！！！！！！！！！！！！！！！！！！！！！//
void Motor_Init() {
    my_motor = MotorInit(GPIOE, GPIO_PIN_11, &htim1, TIM_CHANNEL_1);
}



//！！！！！！！！！！！！！！！！！！！！！！！！化序窮字陣崙 嘔辧庫抱岷圭?鮨慟読蕨? 珊短恂方象啌符！！！！！！！！！！！！//
void Control_Motor() {
    MotorControl(my_motor, MOTOR_UP_BUJIN, 200.0f, 1000);
    MotorControl(my_motor, MOTOR_UP_BUJIN, 0, 0);
}

void bujin_cmd(){
		float control;
		control =(float)rc_cmd->rc.rocker_r1*20000/660;
		if(control < 0)
		{
			MotorControl(my_motor,1,-control,100);
		}
				if(control == 0)
		{
			MotorControl(my_motor,0,0,100);
		} 
				if(control > 0)
		{
			MotorControl(my_motor,0,control,100);
		}
}



//！！！！！！！！！！！！！！！！！！！！！！倶字廣過參式兜兵晒！！！！！！！！！！！！！！！！！！！！！！！！！！！！//
void servo_left_init(){
	FTMotor_Init_Config_s ftmotor_config ={
		.usart_init_config = {
			.usart_handle = &huart6,
		},
		.motor_set = {
			.ID = 1,
			.MemAddr = SMS_STS_ACC,
      .Fun = INST_WRITE,
		},
		.motor_ref = {
			.Position =  0,
			.Speed =  2250,
			.ACC = 50,
		},
	};
	servo_left = FTMotorInit(&ftmotor_config);
}

void servo_right_init(){
	FTMotor_Init_Config_s ftmotor_config1 ={
		.usart_init_config = {
			.usart_handle = &huart6,
		},
		.motor_set = {
			.ID = 2,
			.MemAddr = SMS_STS_ACC,
      .Fun = INST_WRITE,
		},
		.motor_ref = {
			.Position =  0,
			.Speed =  2250,
			.ACC = 50,
		},
	};
	servo_right = FTMotorInit(&ftmotor_config1);
}

void banji_init()
{
	servo_left_init();
	servo_right_init();
}

//！！！！！！！！！！！！！！！！！！！！！！！！謂字陣崙！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！//


void Control_Servo(int id, int start_pos, int end_pos) {
    FTMotor_instance *servo = NULL;
    if (id == 1) {
        servo = servo_left;
    } else if (id == 2) {
        servo = servo_right;
    }    
    servo->motor_ref.Position = start_pos;
    WritePosEx(servo);
//    HAL_Delay(1000);
    servo->motor_ref.Position = end_pos;
    WritePosEx(servo); 
//		HAL_Delay(1000);
}

//諏和謂字
void banji_shoot(){
	
	servo_left->motor_ref.Position = 350;
	servo_right->motor_ref.Position =0;
	FTMotorControl();
//	HAL_Delay(1000);
}

//防蝕謂字
void banji_reset(){
	servo_left->motor_ref.Position = 0;
	servo_right->motor_ref.Position = 350;
	FTMotorControl();
//	HAL_Delay(1000);
}

void banji_cmd(){
		if((uint8_t)rc_cmd->rc.switch_right == (uint8_t)1)
		{
			banji_reset();
		}
		else if ((uint8_t)rc_cmd->rc.switch_right == (uint8_t)2)
		{
			banji_shoot();
		}
}

//！！！！！！！！！！！！！！！！！！！！！！！！寄舟窮字廣過！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！


void shoot_M3508_init()
{
			Motor_Init_Config_s M3508_chassis_config = {
			.can_init_config = {
					.can_handle = &hcan1,
					.tx_id      = 5,
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
					.outer_loop_type       = SPEED_LOOP, // 兜兵晒撹SPEED_LOOP,斑王徒唯壓圻仇,契峭王徒貧窮扮岱廬
					.close_loop_type       = CURRENT_LOOP | SPEED_LOOP  ,
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
					.feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
			},
			.motor_type = M3508};
		
		shoot_M3508 = DJIMotorInit(&M3508_chassis_config);
}

//！！！！！！！！！！！！！！！！！！！！！！！！寄舟窮字陣崙 嘔辧庫邦峠圭?鮨慟童?剿！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！


//void shoot_M3508_cmd_Task(){
//		float control;
//		control =(float)rc_cmd->rc.rocker_r_;
//		if(control > 0)
//		{
//			DJIMotorEnable(shoot_M3508);
//			DJIMotorSetRef(shoot_M3508,((float)rc_cmd->rc.rocker_r_/660)*100000);
//		}
//		else if (control < 0)
//		{
//			DJIMotorEnable(shoot_M3508);
//			DJIMotorSetRef(shoot_M3508,((float)rc_cmd->rc.rocker_r_/660)*100000);
//		}
//		
//}



//！！！！！！！！！！！！！！！！！！！！！！！！悳販暦距業！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
void shoot_init_Task(){
	banji_init();
	Motor_Init();
	shoot_M3508_init();
}

void shoot_cmd_Task()
{
//	GetCmd();
	bujin_cmd();
//	shoot_M3508_cmd_Task();
	banji_cmd();
}






























