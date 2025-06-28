#ifndef __CHASSIS_H_
#define __CHASSIS_H_


typedef enum
{
    TRANS_STOP = 0, // 停止
    TRANS_DIRECT,   // 正转
    TRANS_REVERSE,  // 反转
} trans_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,pc在云台,遥控器和裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float vw;           // 旋转速度
    float last_yaw;     // 纯前进时需要保持直线的角度
		float offset_w;     //因为直线偏转，pid产生的回正力
    trans_mode_e trans_mode;

	  float real_vx;
    float real_vy;
    float real_wz;

} Chassis_Ctrl_Cmd_s;

//extern Cmd pscmd;
//extern motor_config_s motor[4];
void Chassis_Init(void);
void Chassis_Task(void);
void GetCmd(void);
#endif 
