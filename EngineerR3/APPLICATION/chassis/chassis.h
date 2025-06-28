#ifndef __CHASSIS_H_
#define __CHASSIS_H_


typedef enum
{
    TRANS_STOP = 0, // ֹͣ
    TRANS_DIRECT,   // ��ת
    TRANS_REVERSE,  // ��ת
} trans_mode_e;

/* ----------------CMDӦ�÷����Ŀ�������,Ӧ����gimbal/chassis/shoot����---------------- */
/**
 * @brief ����˫�����,pc����̨,ң�����Ͳ���ϵͳ�ڵ���
 *
 */
// cmd�����ĵ��̿�������,��chassis����
typedef struct
{
    // ���Ʋ���
    float vx;           // ǰ�������ٶ�
    float vy;           // ���Ʒ����ٶ�
    float vw;           // ��ת�ٶ�
    float last_yaw;     // ��ǰ��ʱ��Ҫ����ֱ�ߵĽǶ�
		float offset_w;     //��Ϊֱ��ƫת��pid�����Ļ�����
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
