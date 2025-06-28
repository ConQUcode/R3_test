#include "dribble.h"
#include "dmmotor.h"
#include "DJI_motor.h"
#include "remote.h"
#include "bsp_dwt.h"
#include "robot_def.h"
extern RC_ctrl_t *rc_cmd;
static DJIMotor_Instance *motor_3508_1, *motor_3508_2, *motor_3508_3, *motor_3508_4, *motor_3508_5, *motor_2006_1, *motor_2006_2;
static float totalangle = 0;
static float totalangle2 = 0;
static int8_t is_init = 0;
static int8_t is_init2 = 0;
static PID_Instance gains_pid;  // ����PID
static float control = 0;

//������� ������ֵ
#define CURRENT_THRESHOLD  3000  // ���������ֵ(����ֵ)
#define SPEED_DROP_THRESH  1000  // �ٶ��½�����ֵ(rad/(ms*s))


void DribbleInit()
{
	//����PID�Ĳ���
		PID_Init_Config_s gains_pid_config = {
        .Kp                = 10, // 6
        .Ki                = 0.15f,
        .Kd                = 0.02, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 500, // 200
        .MaxOut            = 10000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
		
    PIDInit(&gains_pid, &gains_pid_config);
		
	
		
    Motor_Init_Config_s M3508_zhuan_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10,  // 10
                .Ki = 0, // 1
                .Kd = 0.05,
                // .CoefA         = 0.2,
                // .CoefB         = 0.3,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 1000,
                .MaxOut        = 1500,

            },
            .current_PID = {
                .Kp            = 0.5, // 0.7
                .Ki            = 0.01, // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 1000,
                .MaxOut        = 1500,
                // .DeadBand      = 0.1,
            },
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP, // ��ʼ����SPEED_LOOP,�ò���ͣ��ԭ��,��ֹ�����ϵ�ʱ��ת
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
					.can_handle = &hcan2,
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
					.outer_loop_type       = ANGLE_LOOP, // ��ʼ����SPEED_LOOP,�ò���ͣ��ԭ��,��ֹ�����ϵ�ʱ��ת
					.close_loop_type       = CURRENT_LOOP | SPEED_LOOP |ANGLE_LOOP,
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
					.feedforward_flag      = FEEDFORWARD_NONE,
			},
			.motor_type = M3508};
				
		
		motor_3508_4 =  DJIMotorInit(&M3508_jiaodu_config);
		DJIMotorOuterLoop(motor_3508_4, ANGLE_LOOP);		
				
			
			Motor_Init_Config_s M3508_qiulu_config = {
			.can_init_config = {
					.can_handle = &hcan2,
					.tx_id      = 5,
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
					.outer_loop_type       = ANGLE_LOOP, // ��ʼ����SPEED_LOOP,�ò���ͣ��ԭ��,��ֹ�����ϵ�ʱ��ת
					.close_loop_type       = CURRENT_LOOP | SPEED_LOOP |ANGLE_LOOP,
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
					.feedforward_flag      = FEEDFORWARD_NONE,
			},
			.motor_type = M3508};
				
		motor_3508_5 =  DJIMotorInit(&M3508_qiulu_config);
			
			
				motor_3508_4 =  DJIMotorInit(&M3508_jiaodu_config);
		DJIMotorOuterLoop(motor_3508_4, ANGLE_LOOP);		
				
			
			Motor_Init_Config_s M2006_tuiqiu_config = {
			.can_init_config = { 
					.can_handle = &hcan1,
					.tx_id      = 5,
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
					.outer_loop_type       = ANGLE_LOOP, // ��ʼ����SPEED_LOOP,�ò���ͣ��ԭ��,��ֹ�����ϵ�ʱ��ת
					.close_loop_type       = CURRENT_LOOP | SPEED_LOOP |ANGLE_LOOP,
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
					.feedforward_flag      = FEEDFORWARD_NONE,
			},
			.motor_type = M3508};
				
		motor_2006_1 =  DJIMotorInit(&M2006_tuiqiu_config);
			M2006_tuiqiu_config.can_init_config.tx_id = 6;
			motor_2006_2 = DJIMotorInit(&M2006_tuiqiu_config);
}






//��������pid���жϺͼ���
void gainsschedule(DJIMotor_Instance *motor){
	//���ݵ���ĵ������ٶȽ����ж��Ƿ����غ�
	if (motor->measure.real_current > CURRENT_THRESHOLD || motor->measure.speed_aa < -SPEED_DROP_THRESH) {
		//������غɽ���pid��������
		//�ж��Ƿ����ԭ����gains_pid�Ĳ�����������ھ�ֱ�ӽ�gains_pid�Ĳ��������������ٶȻ�pid����������ͽ�����������
		//����0.9��0.1����ϵͳ���غɵķ�Ӧ��ֵԽ�󣬴���ϵͳ��ӦԽ��
		motor->motor_controller.speed_PID.Kp = (motor->motor_controller.speed_PID.Kp <gains_pid.Kp) ? 0.9*motor->motor_controller.speed_PID.Kp  +  0.1* gains_pid.Kp : gains_pid.Kp;
		motor->motor_controller.speed_PID.Ki = (motor->motor_controller.speed_PID.Ki <gains_pid.Ki) ? 0.9*motor->motor_controller.speed_PID.Ki  +  0.1* gains_pid.Kp : gains_pid.Ki;
		motor->motor_controller.speed_PID.Kd = (motor->motor_controller.speed_PID.Kd <gains_pid.Kd) ? 0.9*motor->motor_controller.speed_PID.Kd  +  0.1* gains_pid.Kp : gains_pid.Kd;
		
	} else {
			
		motor->motor_controller.speed_PID.Kp = (motor->motor_controller.speed_PID.Kp >4)     ? (motor->motor_controller.speed_PID.Kp  -  0.1* gains_pid.Kp)/0.9 : 4;
		motor->motor_controller.speed_PID.Ki = (motor->motor_controller.speed_PID.Ki >0.025) ? (motor->motor_controller.speed_PID.Ki  -  0.1* gains_pid.Ki)/0.9 : 0.025;
		motor->motor_controller.speed_PID.Kd = (motor->motor_controller.speed_PID.Kd >0.02)  ? (motor->motor_controller.speed_PID.Kd  -  0.1* gains_pid.Kd)/0.9 : 0.02;

				
  }
}

//�����湦�ܽ��м򵥰�װ
void gains(){
	
	gainsschedule(motor_3508_1);
	gainsschedule(motor_3508_2);
	gainsschedule(motor_3508_3);
	
}

//�����M3508��ʼ�����Լ������޷�
static void LiftInit(){
	if(!is_init){
		if (abs(motor_3508_4->measure.real_current) <= 4500 && is_init == 0) {
        DJIMotorEnable(motor_3508_4);
        DJIMotorOuterLoop(motor_3508_4, SPEED_LOOP);
        DJIMotorSetRef(motor_3508_4, -1000);
    } 
		else if ((abs(motor_3508_4->measure.real_current) > 4500) && is_init == 0) {
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

void Dribblelift(){
	totalangle = motor_3508_4->measure.total_angle;
	DJIMotorSetRef(motor_3508_4,5000);
}

void Dribbledown(){
	totalangle = motor_3508_4->measure.total_angle;
	DJIMotorSetRef(motor_3508_4,-2500);
}

void Dribbleplay(){

	int speed =15000;
	DJIMotorSetRef(motor_3508_1,-speed-600);
	DJIMotorSetRef(motor_3508_2,speed+400);
	DJIMotorSetRef(motor_3508_3,speed+400);  
//	DJIMotorControl();
}
//��·���ֳ�ʼ���Լ��޷�
static void LiftIntit(){
  if(!is_init2){
		if(abs(motor_3508_5->measure.real_current) <= 4500 && is_init2 == 0){
			DJIMotorEnable(motor_3508_5);
			DJIMotorOuterLoop(motor_3508_5,SPEED_LOOP);
			DJIMotorSetRef(motor_3508_5,-1000);
		}
		else if(abs(motor_3508_5->measure.real_current) > 4500 && is_init2 == 0){
			HAL_Delay(5);
			if((motor_3508_5->measure.real_current) < -4500 && is_init2 == 0){
				DJIMotorOuterLoop(motor_3508_5,ANGLE_LOOP);
				DJIMotorReset(motor_3508_5);
				DJIMotorStop(motor_3508_5);
			}
		}
	}
}
void Pathup(){
	totalangle2 =motor_3508_5->measure.total_angle;
	DJIMotorSetRef(motor_3508_5,10000);
};
void Pathdown(){
	totalangle2 =motor_3508_5->measure.total_angle;
	DJIMotorSetRef(motor_3508_5,-10000);
};
void Tuiqiuplay(){
  DJIMotorSetRef(motor_2006_1,-2000);
	
}

void DribbleTask(){
	LiftInit();
	control = (float)rc_cmd->rc.rocker_r_ ;
    	if(is_init){
				DJIMotorEnable(motor_3508_4);
				totalangle = motor_3508_4->measure.total_angle;
				
					if (control > 0){
					Dribblelift();
				if(rc_cmd->rc.switch_right == 3)
				{
						DJIMotorSetRef(motor_3508_1,0);
						DJIMotorSetRef(motor_3508_2,0);
						DJIMotorSetRef(motor_3508_3,0);
				
				
				}
				if(rc_cmd->rc.switch_right ==2){
				 Pathdown();
				}
				if(rc_cmd->rc.switch_right == 1){
					Dribbleplay();
				 Pathup();
				}
					}
					else if (control < 0){
					Dribbledown();	
					}

			}
	//ִ������
	gains();
}	


