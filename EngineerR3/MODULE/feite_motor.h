#ifndef __FEITE_MOTOR_H_
#define __FEITE_MOTOR_H_

#include "bsp_usart.h"
#include "daemon.h"
#include "stdlib.h"
#include "string.h"

#define FTMOTOR_MAX_BUFFSIZE 32  // �����/�����ֽ���,��������������Ӵ���ֵ
#define FTMOTOR_OFFSET_BYTES 4


#define FT_MOTOR_CNT 255 // ���ض������

//MemAddr
#define SMS_STS_ACC 41

//Fun
#define INST_WRITE 0x03


/* �����ͣ��־ */
#define FEITE_STOP 0
#define FEITE_ENALBED 1

typedef struct{
	
	uint8_t ID;
	uint8_t MemAddr;
	uint8_t Fun;
	
}Motor_Set_s;

typedef struct{
	
	uint16_t Position;
	uint16_t Speed;
	uint8_t ACC;
	uint8_t stop_flag;
	
}Motor_Ref_s;


#pragma pack(1)
typedef struct{
	USART_Instance *ft_usart_instance;
	Daemon_Instance *ft_daemon;
	
/* ���Ͳ��� */
	uint8_t send_data_len;                                                // �������ݳ���
	uint8_t send_buff_len;                                                // ���ͻ���������
	
	uint8_t wLen;                                                         //wait ���������ݳ���
	
	uint8_t raw_send_buff[FTMOTOR_MAX_BUFFSIZE + FTMOTOR_OFFSET_BYTES]; // ���ͻ�����������Ԥ��4�ֽ�
	uint8_t data_send_buff[FTMOTOR_MAX_BUFFSIZE + FTMOTOR_OFFSET_BYTES];
	
	Motor_Set_s motor_set;
	Motor_Ref_s motor_ref;
}FTMotor_instance;
#pragma pack()


typedef struct{
	USART_Init_Config_s usart_init_config;
	
	Motor_Set_s motor_set; 
	Motor_Ref_s motor_ref;
	
}FTMotor_Init_Config_s;

FTMotor_instance *FTMotorInit(FTMotor_Init_Config_s *config);
void FTMotorControl(void);
void WritePosEx(FTMotor_instance *motor);
#endif

