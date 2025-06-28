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
static PID_Instance chassis_follow_pid;  // ���̸���PID
static Nac_Recv_s *nac_ctrl; // �Ӿ�������Ϣ


void Chassis_Init()
{
	//�����߾��ȼ�����ʱ��
	DWT_Init(180);
	
	//����ң��������
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
					.outer_loop_type       = SPEED_LOOP, // ��ʼ����SPEED_LOOP,�ò���ͣ��ԭ��,��ֹ�����ϵ�ʱ��ת
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
		
		//ͬһ���ת��
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
		
		Chassis_IMU_data                 = INS_Init(); // ����IMU��ʼ��
		nac_ctrl     = NacInit(&huart1); // ��ʼ���Ӿ�����
		
}

/**
 * @brief ʹ�����Ƕ���С��ת��ȡ�Ż�����ֹ�����ת����Ҫ���г�
 *          ���磺�ϴνǶ�Ϊ0��Ŀ��Ƕ�Ϊ135�ȣ�
 *          �����ѡ����ʱ����ת��-45�ȣ�������˳ʱ����ת��135�ȣ�
 *          �����Ƕȶ������ֵ������ͬһƽ������
 *
 * @param angle Ŀ��Ƕ�
 * @param last_angle �ϴνǶ�
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

//---------------------------------���ֵ����˶�����--------------------------------------

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


//------------------------------�����˶�task-----------------------------------------------
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
