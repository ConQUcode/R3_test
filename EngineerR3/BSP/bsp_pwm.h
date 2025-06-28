#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"
#include "stdint.h"
#include "stm32f4xx_hal_rcc.h"
//#include "stm32f407xx.h"
#define PWM_DEVICE_CNT 16 // 最大支持的PWM实例数量

/* pwm实例结构体 */
typedef struct pwm_ins_temp
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t tclk;                           // 时钟频率
    float period;                         // 周期
    float dutyratio;                      // 占空比
    void (*callback)(struct pwm_ins_temp *); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWMInstance;

typedef struct
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    float period;                         // 周期
    float dutyratio;                      // 占空比
    void (*callback)(PWMInstance*); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWM_Init_Config_s;


PWMInstance *PWMRegister(PWM_Init_Config_s *config);

void PWMStart(PWMInstance *pwm);


void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);

void PWMStop(PWMInstance *pwm);

void PWMSetPeriod(PWMInstance *pwm, float period);

 
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size);

#endif
