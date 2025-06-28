#include "tuiqiu.h"
#include "daemon.h"
#include "remote.h"
#include "feite_motor.h"
#include "bsp_dwt.h"
#include "MD_tuiqiu.h"




TuiqiuMotor *mymotor1,*mymotor2;
extern RC_ctrl_t *rc_cmd;
//！！！！！！！！！！！！！！！！！！！！！！容庫廣過參式兜兵晒！！！！！！！！！！！！！！！！！！！！！！！！！！！！//
void TuiQiu_Init() {
    mymotor1 = TuiQiuInit(GPIOC,GPIO_PIN_6);
	  mymotor2 = TuiQiuInit(GPIOI,GPIO_PIN_6);
	   }
//！！！！！！！！！！！！！！！！！！！！！！！！容庫陣崙！！！！！！！！！！！！//
void Control_TuiQiu_UP(){
     TuiQiuControl(mymotor1,MOTOR_UP_TUIQIU);
	   TuiQiuControl(mymotor2,MOTOR_DOWN_TUIQIU);
 }

void Control_TuiQiu_DOWN(){
     TuiQiuControl(mymotor1,MOTOR_DOWN_TUIQIU);
	   TuiQiuControl(mymotor2,MOTOR_UP_TUIQIU);
 }

void TuiQiu_cmd_Task(){
		float control;
		control =(float)rc_cmd->rc.rocker_r_;
		if(control > 0)
		{
        Control_TuiQiu_UP();
		}
		else if (control < 0)
		{
        Control_TuiQiu_DOWN();
		}
		
}
