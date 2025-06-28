#ifndef __BUJIN_H__
#define __BUJIN_H__

#include "bsp_gpio.h"
#include "bsp_pwm.h"

typedef enum {
    MOTOR_UP_BUJIN = 0,
    MOTOR_DOWN_BUJIN = 1
} MotorDirection;

// 电机实例结构体
typedef struct {
    GPIOInstance *dir_gpio;  // 方向控制GPIO
    PWMInstance *pwm;        // PWM控制实例
    int32_t step_count;      // 步进计数
} StepperMotor;

StepperMotor* MotorInit(GPIO_TypeDef* dir_gpio_port, uint16_t dir_pin, TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch);

void MotorControl(StepperMotor* motor, MotorDirection dir, float speed_hz, int32_t steps);

#endif
