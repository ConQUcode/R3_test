#include "MD_tuiqiu.h"

// �����ʼ��
TuiqiuMotor* TuiQiuInit(GPIO_TypeDef* dir_gpio_port, uint16_t dir_pin) {
    TuiqiuMotor* motor = (TuiqiuMotor*)malloc(sizeof(TuiqiuMotor));
    
    // GPIO��ʼ��
    GPIO_Init_Config_s gpio_conf = {
        .GPIOx = dir_gpio_port,
        .GPIO_Pin = dir_pin,
        .pin_state = GPIO_PIN_RESET,
        .exti_mode = GPIO_MODE_OUTPUT_PP,
        .id = "Motor_DIR",
        .gpio_model_callback = NULL
    };
    motor->dir_gpio = GPIORegister(&gpio_conf);

		return motor;
}

// �������
void TuiQiuControl(TuiqiuMotor* motor, MotorDirection dir) {
    // ���÷���
    dir == MOTOR_UP_TUIQIU ? GPIOSet(motor->dir_gpio) : GPIOReset(motor->dir_gpio);
    
   
}
