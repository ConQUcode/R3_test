#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"
#include "stdint.h"
#include "stm32f4xx_hal_rcc.h"
//#include "stm32f407xx.h"
#define PWM_DEVICE_CNT 16 // ���֧�ֵ�PWMʵ������

/* pwmʵ���ṹ�� */
typedef struct pwm_ins_temp
{
    TIM_HandleTypeDef *htim;                 // TIM���
    uint32_t channel;                        // ͨ��
    uint32_t tclk;                           // ʱ��Ƶ��
    float period;                         // ����
    float dutyratio;                      // ռ�ձ�
    void (*callback)(struct pwm_ins_temp *); // DMA������ɻص�����
    void *id;                                // ʵ��ID
} PWMInstance;

typedef struct
{
    TIM_HandleTypeDef *htim;                 // TIM���
    uint32_t channel;                        // ͨ��
    float period;                         // ����
    float dutyratio;                      // ռ�ձ�
    void (*callback)(PWMInstance*); // DMA������ɻص�����
    void *id;                                // ʵ��ID
} PWM_Init_Config_s;


PWMInstance *PWMRegister(PWM_Init_Config_s *config);

void PWMStart(PWMInstance *pwm);


void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);

void PWMStop(PWMInstance *pwm);

void PWMSetPeriod(PWMInstance *pwm, float period);

 
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size);

#endif
