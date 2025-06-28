#ifndef __SHOOT_EX_H_
#define __SHOOT_EX_H_


//typedef enum
//{
//    TRANS_STOP = 0, // 停止
//    TRANS_DIRECT,   // 正转
//    TRANS_REVERSE,  // 反转
//} trans_mode_e;

//typedef enum
//{
//    BAN_STOP = 3, // 停止
//    BAN_OFF = 1,   // 发射
//    BAN_ON = 2,  // 扣紧
//} ban_mode_e;

///* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
///**
// * @brief 对于双板情况,pc在云台,遥控器和裁判系统在底盘
// *
// */
//// cmd发布的底盘控制数据,由chassis订阅
//typedef struct
//{
//    // 控制部分
//    float vx;           // 前进方向速度
//    float vy;           // 横移方向速度
//    float vw;           // 旋转速度
//    float offset_angle; // 底盘和归中位置的夹角
//    trans_mode_e trans_mode;
//	
//	
//		float vshoot;
//		ban_mode_e ban_mode;

//	  float real_vx;
//    float real_vy;
//    float real_wz;

//} Ctrl_Cmd_s;

//extern Ctrl_Cmd_s ctrl_cmd;

//void Shoot_ex_Init(void);
//void Shoot_ex_Task(void);

#endif 
