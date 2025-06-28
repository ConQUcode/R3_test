#ifndef __TUIQIU_H
#define __TUIQIU_H

#include "bsp_gpio.h"
#include "bsp_pwm.h"

typedef enum {
    MOTOR_UP_TUIQIU = 0,
    MOTOR_DOWN_TUIQIU = 1
} MotorDirection;

// ���ʵ���ṹ��
typedef struct {
    GPIOInstance *dir_gpio ;  // �������GPIO
} TuiqiuMotor;

TuiqiuMotor* TuiQiuInit(GPIO_TypeDef* dir_gpio_port, uint16_t dir_pin);

void TuiQiuControl(TuiqiuMotor* motor, MotorDirection dir);



#endif /* __TUIQIU_H */
