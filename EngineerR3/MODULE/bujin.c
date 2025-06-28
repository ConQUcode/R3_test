#include "bujin.h"

// �����ʼ��
StepperMotor* MotorInit(GPIO_TypeDef* dir_gpio_port, uint16_t dir_pin, TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch) {
    StepperMotor* motor = (StepperMotor*)malloc(sizeof(StepperMotor));
    
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

    // PWM��ʼ��
    PWM_Init_Config_s pwm_conf = {
        .htim = pwm_tim,
        .channel = pwm_ch,
        .period = 1.0f,  // Ĭ��1������
        .dutyratio = 0.5f,
        .callback = NULL,
        .id = "Motor_PWM"
    };
    motor->pwm = PWMRegister(&pwm_conf);
    
    motor->step_count = 0;
    return motor;
}

// �������
void MotorControl(StepperMotor* motor, MotorDirection dir, float speed_hz, int32_t steps) {
    // ���÷���
    dir == MOTOR_UP_BUJIN ? GPIOSet(motor->dir_gpio) : GPIOReset(motor->dir_gpio);
    
    // �������ڲ�����PWM
    if(speed_hz > 0) {
        float period = 1.0f / speed_hz;  // Ƶ��ת����
        PWMSetPeriod(motor->pwm, period);
        PWMSetDutyRatio(motor->pwm, 0.5f);  // 50%ռ�ձ�
        PWMStart(motor->pwm);
    } else {
        PWMStop(motor->pwm);
    }
    
    // ���²���ͳ��
    motor->step_count += steps;
}
