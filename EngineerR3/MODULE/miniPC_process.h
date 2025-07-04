/**
 * @file miniPC_process.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   用于处理miniPC的数据，包括解析和发送
 * @version 0.1
 * @date 2024-01-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MINIPC_PROCESS_H
#define MINIPC_PROCESS_H

#include "stdint.h"
#include "bsp_usart.h"

#define NAC_RECV_HEADER1 0xAA // 视觉接收数据帧头
#define NAC_RECV_HEADER2 0x3A // 视觉接收数据帧头
#define NAC_RECV_TAIL    0x55 // 视觉发送数据帧尾

#define NAC_SEND_HEADER 0xA5u // 视觉发送数据帧头
#define NAC_SEND_TAIL   0xAAu // 视觉发送数据帧尾

#define VISION_RECV_SIZE   25u // 当前为固定值,25字节
#define VISION_SEND_SIZE   1u

// #pragma pack(1) // 1字节对齐

/* 视觉通信初始化接收结构体 */
typedef struct
{
    uint8_t header1;          // 帧头 0xAA
		uint8_t header2;          // 帧头 0xAA
	  uint8_t tail;            // 帧尾 0x55
} Nac_Recv_Init_Config_s;

/* 视觉通信初始化发送结构体 */
typedef struct
{
    uint8_t header;        // 头帧校验位
    uint8_t detect_color;  // 0-red 1-blue
    uint8_t reset_tracker; // 是否重置追踪器 发0 bool is_shoot; // 是否开启自瞄模式 开发 1
    uint8_t is_shoot;      // 是否开启自瞄模式 开发 1
    uint8_t tail;          // 尾帧校验位
} Nac_Send_Init_Config_s;


/* 视觉实例初始化配置结构体 */
typedef struct
{
    Nac_Recv_Init_Config_s recv_config; // 接收数据结构体
    Nac_Send_Init_Config_s send_config; // 发送数据结构体
    USART_Init_Config_s usart_config;      // 串口实例结构体
} Nac_Init_Config_s;


typedef struct {
    uint8_t header1;          // 帧头 0xAA
		uint8_t header2;          // 帧头 0xAA
    int16_t basket_angle;    // 2字节
    int16_t basket_distance; // 2字节
    int16_t robo_angle;      // 2字节
    int16_t robo_distance;   // 2字节
    uint8_t tail;            // 帧尾 0x55
} Nac_Recv_s;

typedef struct {
    uint8_t header;          // 帧头 0xAA
    int16_t basket_angle;    // 2字节
    int16_t basket_distance; // 2字节
    int16_t robo_angle;      // 2字节
    int16_t robo_distance;   // 2字节
    uint8_t tail;            // 帧尾 0x55
} Nac_Send_s;

/* 视觉通信模块实例 */
typedef struct
{
    Nac_Recv_s *recv_data; // 接收数据结构体指针
    Nac_Send_s *send_data; // 发送数据结构体指针
    USART_Instance *usart;    // 串口实例指针
} Nac_Instance;




/* 是否追踪 */
typedef enum {
    VISION_NO_SHOOTING = 0u,
    VISION_SHOOTING    = 1u,
} VISION_SHOOTING_e;

/* 是否重置追踪 */
typedef enum {
    VISION_RESET_TRACKER_NO  = 0u,
    VISION_RESET_TRACKER_YES = 1u,
} VISION_RESET_TRACKER_e;

/* 目标ID */
typedef enum {
    VISION_OUTPOST = 0u,
    VISION_GUARD   = 6u,
    VISION_BASE    = 7u,
} VISION_ID_e;

/* 装甲板数量 */
typedef enum {
    VISION_ARMORS_NUM_BALANCE = 2u,
    VISION_ARMORS_NUM_OUTPOST = 3u,
    VISION_ARMORS_NUM_NORMAL  = 4u,
} VISION_ARMORS_NUM_e;

/* 敌方装甲板颜色 */
typedef enum {
    VISION_DETECT_COLOR_RED  = 0u,
    VISION_DETECT_COLOR_BLUE = 1u,
} VISION_DETECT_COLOR_e;

/* 视觉通信初始化接收结构体 */
typedef struct
{
    uint8_t header; // 头帧校验位
} Vision_Recv_Init_Config_s;

/* 视觉通信初始化发送结构体 */
typedef struct
{
    uint8_t header;        // 头帧校验位
    uint8_t detect_color;  // 0-red 1-blue
    uint8_t reset_tracker; // 是否重置追踪器 发0 bool is_shoot; // 是否开启自瞄模式 开发 1
    uint8_t is_shoot;      // 是否开启自瞄模式 开发 1
    uint8_t tail;          // 尾帧校验位
} Vision_Send_Init_Config_s;

/* 视觉实例初始化配置结构体 */
typedef struct
{
    Vision_Recv_Init_Config_s recv_config; // 接收数据结构体
    Vision_Send_Init_Config_s send_config; // 发送数据结构体
    USART_Init_Config_s usart_config;      // 串口实例结构体
} Vision_Init_Config_s;

/* minipc -> stm32 (接收结构体) */
#pragma pack(1) // 1字节对齐
typedef struct
{
    uint8_t header1;
		uint8_t header2;
		uint8_t tail;
	
    float is_tracking; // 是否追踪
    float maximal_arm;   // 大臂的目标值
    float minimal_arm;   // 小臂的目标值
    float z_height;      // 机械臂高度
    float finesse;       // 手腕的目标值
    float pitch_arm;     // pitch的目标值
    float yaw;           // yaw的目标值,2006电机角度值
} Vision_Recv_s;


/* stm32 -> minipc (发送结构体) */
typedef struct
{
    uint8_t header;
    /* 下面的数据暂时不需要 */
    // uint8_t detect_color;  // 0-red 1-blue 发1
    // uint8_t reset_tracker; // 是否重置追踪器 发0
    // uint8_t is_shoot;      // 是否开启自瞄模式 开发 1
    // float roll;            // rad
    // float yaw;             // rad
    // float pitch;           //
    // uint16_t checksum;     // crc16校验位 https://blog.csdn.net/ydyuse/article/details/105395368
    // uint8_t tail;          // 尾帧校验位
} Vision_Send_s;


#pragma pack() // 取消1字节对齐
/* 视觉通信模块实例 */
typedef struct
{
    Vision_Recv_s *recv_data; // 接收数据结构体指针
    Vision_Send_s *send_data; // 发送数据结构体指针
    USART_Instance *usart;    // 串口实例指针
} Vision_Instance;

// #pragma pack() // 取消1字节对齐



/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacRecvRegister(Nac_Recv_Init_Config_s *recv_config);


/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Nac_Send_s *NacSendRegister(Nac_Send_Init_Config_s *send_config);


/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Nac_Recv_s *NacInit(UART_HandleTypeDef *Nac_usart_handle);


/**
 * @brief 发送函数
 *
 *
 */
void NacSend();


/**
 * @brief 设置发送给视觉的IMU数据
 *
 * @param yaw
 * @param pitch
 * @param roll
 */
void NacSetAltitude(float yaw, float pitch, float roll);












/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config);



/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config);

/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *video_usart_handle);

/**
 * @brief 发送函数
 *
 *
 */
void VisionSend(uint8_t is_start);

/**
 * @brief 设置发送给视觉的IMU数据
 *
 * @param yaw
 * @param pitch
 * @param roll
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif // MINIPC_PROCESS_H