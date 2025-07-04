/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   机器人定义,包含机器人的各种参数
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ROBOT_DEF_H__
#define __ROBOT_DEF_H__

#include "stdint.h"
//#include "ins_task.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // ! 单板控制整车，beta选项，建议别选上
// #define CHASSIS_BOARD // 底盘板
//#define ARM_BOARD // 工程手臂板

///* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
//// 底盘参数
//// #define CHASSIS_OMNI_WHEEL // 是否为全向轮底盘
//#define CHASSIS_MCNAMEE_WHEEL // 是否为麦克纳姆轮底盘

//#define VISION_USE_VCP        // 是否使用虚拟串口
//// #define VISION_USE_UART // 是否使用硬件串口

// #define VIDEO_LINKK // 是否有图传链路
//#define REMOTE_LINK   // 是否有常规链路

#define MAXARM_LENGTH 220 // 机械臂最大长度 mm
#define MINARM_LENGTH 220 // 机械臂最小长度 mm

#define MAXARM_ZERO   0.0f
#define MINARM_ZERO   0.0f
#define FINE_ZERO     0.0f
#define PITCH_ZERO    0.0f

#define MAXARM_MIN    -0.24f
#define MAXARM_MAX    1.74f
#define MINARM_MIN    -2.45f
#define MINARM_MAX    2.4f
#define FINE_MIN      -1.16f // -1.16
#define FINE_MAX      1.72f
#define FINE_MIN2     -2.16f
#define FINE_MAX2     1.96f
#define PITCH_MIN     -1.81f
#define PITCH_MAX     1.50f
#define HEIGHT_MIN    -550.f
#define HEIGHT_MAX    560.f // 50
#define ROLL_MIN      -180.f
#define ROLL_MAX      180.f

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(ARM_BOARD)) ||     \
    (defined(CHASSIS_BOARD) && defined(ARM_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

// 检查是否出现底盘类型定义冲突,只允许一个底盘类型定义存在,否则编译会自动报错
#if (defined(CHASSIS_OMNI_WHEEL) && defined(CHASSIS_MCNAMEE_WHEEL))
#error Conflict chassis definition! You can only define one chassis type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0, // 电流零输入
    CHASSIS_FAST,           // 底盘转速快
    CHASSIS_MSEDIUM,        // 底盘转速中等
    CHASSIS_LOW,           // 底盘转速慢
} chassis_mode_e;

typedef enum
{
    TRANS_STOP = 0, // 停止
    TRANS_DIRECT,   // 正转
    TRANS_REVERSE,  // 反转
} trans_mode_e;

typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0, // 停止发射
    LOAD_REVERSE,  // 反转
    LOAD_SLOW,     // 慢速
    LOAD_MEDIUM,   // 中速
    LOAD_FAST,     // 快速
} loader_mode_e;

typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10        = 10,
    SMALL_AMU_15      = 15,
    BIG_AMU_16        = 16,
    SMALL_AMU_18      = 18,
    SMALL_AMU_30      = 30,
} Bullet_Speed_e;

typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

// UI模式设置
typedef enum
{
    UI_KEEP = 0,
    UI_REFRESH,
} ui_mode_e;

// 机械臂模式设置
typedef enum
{
    ARM_ZERO_FORCE = 0,   // 电流零输入
    ARM_HUM_CONTORL,      // 自定义控制器控制
    ARM_VISION_CONTROL,   // 视觉控制
    ARM_SLIGHTLY_CONTROL, // 轻微控制
    ARM_KEY_CONTROL,      // 键盘控制
    ARM_AUTO_CONTORL,     // 自动控制
    ARM_LIFT_INIT,        // 高度初始化
} arm_mode_e;

// 机械臂控制状态设置,注意与机械臂模式区分,这里可以看作机械臂模式的子模式
typedef enum
{
    ARM_NORMAL = 0,  // 正常状态,能够被其他模式正常控制
    ARM_RECYCLE,     // 回收状态,机械臂回收到初始位置
    ARM_GETCARROCK,  // 抓取状态,机械臂抓取石块
    ARM_GETCARROCK2, // 抓取状态2,机械臂抓取石块后取出
    ARM_GETROCK,     // 抓取状态,机械臂抓取石块,一键取矿
} arm_status_e;

typedef enum
{
    SUCKER_OFF = 0, // 涵道风机关
    SUCKER_ON,      // 涵道风机开
    SUCKER_HALF     //涵道风机开一点
} sucker_mode_e;

typedef enum
{
    LIFT_OFF = 0,    // 机械臂升降关闭
    LIFT_ANGLE_MODE, // 机械臂升降角度模式
    LIFT_SPEED_MODE, // 机械臂升降速度模式
    LIFT_KEEP,       // 机械臂升降保持模式
    LIFT_INIT_MODE,  // 机械臂升降初始化模式
} lift_mode_e;

typedef enum
{
    ROLL_OFF = 0,    // 机械臂roll关闭
    ROLL_ANGLE_MODE, // 机械臂roll开
    ROLL_SPEED_MODE, // 机械臂roll开
    ROLL_KEEP,       // 机械臂roll保持
} roll_mode_e;

typedef enum
{
    DOWNLOAD_OFF = 0, // 关闭调试模式
    DOWNLOAD_ON,      // 开启调试模式,用于控制大疆电机的调试，防止下载时电机转动
} download_mode_e;

// 图传的角度设置？先凑合用用
typedef enum
{
    PITCH_0   = 0,
    PITCH_30  = 30,  // 30度
    PITCH_60  = 60,  // 60度
    PITCH_90  = 90,  // 90度
    PITCH_120 = 120, // 120度
    PITCH_150 = 150, // 150度
    PITCH_180 = 180, // 180度
} video_angle_e;

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
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    trans_mode_e trans_mode;
    float chassis_speed_buff;
    // UI部分
    ui_mode_e ui_mode;         //  UI状态
    arm_mode_e arm_mode;       // 机械臂状态
    sucker_mode_e sucker_mode; // 涵道风机状态
    arm_status_e arm_status;   // 机械臂控制状态(状态子模式)
    float max_arm;             // 机械臂大臂目标角度
    float min_arm;             // 机械臂小臂目标角度
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的机械臂控制数据,由arm订阅
typedef struct
{
    float maximal_arm;             // 机械臂大臂目标角度
    float minimal_arm;             // 机械臂小臂目标角度
    float finesse;                 // 机械臂手腕目标角度
    float pitch_arm;               // 机械臂pitch目标角度
    float lift;                    // 机械臂高度
    float roll;                    // 机械臂roll目标角度
    lift_mode_e lift_mode;         // 机械臂上升标志
    roll_mode_e roll_mode;         // 机械臂roll标志
    sucker_mode_e sucker_mode;     // 涵道风机状态
    arm_mode_e arm_mode;           // 机械臂状态
    arm_mode_e arm_mode_last;      // 机械臂上一次状态
    arm_status_e arm_status;       // 机械臂控制状态(状态子模式)
    download_mode_e download_mode; // 下载模式
    video_angle_e video_angle;     // 图传角度,理论上不应该在这，但没必要专门为了一个舵机开一个topic
    int8_t lift_init;              // 机械臂初始化
} Arm_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// 双板时，下板cmd发布控制云台控制数据，由gimbal订阅
typedef struct
{
    float yaw;
    float up_yaw;
    float up_speed;

    uint8_t is_init;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Yaw_Cmd_s;

typedef struct
{
    float pitch;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Pitch_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制

} Chassis_Upload_Data_s;

// 机械臂反馈数据
typedef struct
{
    float maximal_arm;
    float minimal_arm;
    float finesse;
    float pitch_arm;
    float height;
    float roll;
} Arm_Upload_Data_s;

//typedef struct
//{
//    attitude_t gimbal_imu_data;
//    uint16_t yaw_motor_single_round_angle;
//} Gimbal_Upload_Data_s;

#pragma pack() // 取消压缩
#endif

