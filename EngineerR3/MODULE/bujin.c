#include "bujin.h"

// 电机初始化
StepperMotor* MotorInit(GPIO_TypeDef* dir_gpio_port, uint16_t dir_pin, TIM_HandleTypeDef* pwm_tim, uint32_t pwm_ch) {
    StepperMotor* motor = (StepperMotor*)malloc(sizeof(StepperMotor));
    
    // GPIO初始化
    GPIO_Init_Config_s gpio_conf = {
        .GPIOx = dir_gpio_port,
        .GPIO_Pin = dir_pin,
        .pin_state = GPIO_PIN_RESET,
        .exti_mode = GPIO_MODE_OUTPUT_PP,
        .id = "Motor_DIR",
        .gpio_model_callback = NULL
    };
    motor->dir_gpio = GPIORegister(&gpio_conf);

    // PWM初始化
    PWM_Init_Config_s pwm_conf = {
        .htim = pwm_tim,
        .channel = pwm_ch,
        .period = 1.0f,  // 默认1秒周期
        .dutyratio = 0.5f,
        .callback = NULL,
        .id = "Motor_PWM"
    };
    motor->pwm = PWMRegister(&pwm_conf);
    
    motor->step_count = 0;
    return motor;
}

// 电机控制
void MotorControl(StepperMotor* motor, MotorDirection dir, float speed_hz, int32_t steps) {
    // 设置方向
    dir == MOTOR_UP_BUJIN ? GPIOSet(motor->dir_gpio) : GPIOReset(motor->dir_gpio);
    
    // 计算周期并配置PWM
    if(speed_hz > 0) {
        float period = 1.0f / speed_hz;  // 频率转周期
        PWMSetPeriod(motor->pwm, period);
        PWMSetDutyRatio(motor->pwm, 0.5f);  // 50%占空比
        PWMStart(motor->pwm);
    } else {
        PWMStop(motor->pwm);
    }
    
    // 更新步数统计
    motor->step_count += steps;
}
