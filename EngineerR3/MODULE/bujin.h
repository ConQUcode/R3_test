#ifndef __BUJIN_H__
#define __BUJIN_H__

#include "bsp_gpio.h"
#include "bsp_pwm.h"

typedef enum {
    MOTOR_UP_BUJIN = 0,
    MOTOR_DOWN_BUJIN = 1
} MotorDirection;

// ���ʵ���ṹ��
typedef struct {
    GPIOInstance *dir_gpio;  // �������GPIO
    PWMInstance *pwm;        // PWM����ʵ��
    int32_t step_count;      // ��������
} StepperMotor;

StepperMotor* MotorInit(GPIO_TypeDef* dir_gpio_port, uint16_t dir_pin, TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch);

void MotorControl(StepperMotor* motor, MotorDirection dir, float speed_hz, int32_t steps);

#endif
