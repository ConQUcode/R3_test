#include "bsp_pwm.h"
#include "stdlib.h"
#include "tim.h"

static uint8_t idx;
static PWMInstance *pwm_instance[PWM_DEVICE_CNT] = {NULL}; 
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim );

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < idx; i++)
    { // ����ͬһ����ʱ�����ж���ͨ����ͬ
        if (pwm_instance[i]->htim == htim && htim->Channel == (1<<(pwm_instance[i]->channel/4)))
        {
            if (pwm_instance[i]->callback) // ����лص�����
                pwm_instance[i]->callback(pwm_instance[i]);
            return; // һ��ֻ����һ��ͨ�����ж�,����ֱ�ӷ���
        }
    }
}

PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    if (idx >= PWM_DEVICE_CNT) // �������ʵ����,�������ӻ�鿴�Ƿ����ڴ�й©
        while (1)
            ;
    PWMInstance *pwm = (PWMInstance *)malloc(sizeof(PWMInstance));
    memset(pwm, 0, sizeof(PWMInstance));

    pwm->htim = config->htim;
    pwm->channel = config->channel;
    pwm->period = config->period;
    pwm->dutyratio = config->dutyratio;
    pwm->callback = config->callback;
    pwm->id = config->id;
    pwm->tclk = PWMSelectTclk(pwm->htim);
    // ����PWM
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    PWMSetPeriod(pwm, pwm->period);
    PWMSetDutyRatio(pwm, pwm->dutyratio);
    pwm_instance[idx++] = pwm;
    return pwm;
}

void PWMStart(PWMInstance *pwm)
{
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
}

void PWMStop(PWMInstance *pwm)
{
    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}


void PWMSetPeriod(PWMInstance *pwm, float period)
{
    __HAL_TIM_SetAutoreload(pwm->htim, period*((pwm->tclk)/(pwm->htim->Init.Prescaler+1)));
}

void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio)
{
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, dutyratio * (pwm->htim->Instance->ARR));
}

void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size)
{
    HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
}


static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim )
{
    uintptr_t tclk_temp  = ((uintptr_t)((htim)->Instance));
    if (
            (tclk_temp <= (APB1PERIPH_BASE + 0x2000UL)) &&
            (tclk_temp >= (APB1PERIPH_BASE + 0x0000UL)))
    {
        return (HAL_RCC_GetPCLK1Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
    }
    else if (
            ((tclk_temp <= (APB2PERIPH_BASE + 0x0400UL)) &&
             (tclk_temp >= (APB2PERIPH_BASE + 0x0000UL))) ||
            ((tclk_temp <= (APB2PERIPH_BASE + 0x4800UL)) &&
             (tclk_temp >= (APB2PERIPH_BASE + 0x4000UL))))
    {
        return (HAL_RCC_GetPCLK2Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
    }
    return 0;
}