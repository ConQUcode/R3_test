#include "gpio.h"
#include "stdint.h"

#define GPIO_MX_DEVICE_NUM 10

/**
 * @brief �����ж��ж���Դ,ע���CUBEMX������һ��
 *
 */
typedef enum
{
    GPIO_EXTI_MODE_RISING,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
    GPIO_EXTI_MODE_NONE,
} GPIO_EXTI_MODE_e;

/**
 * @brief GPIOʵ���ṹ�嶨��
 *
 */
typedef struct tmpgpio
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // ����״̬,Set,Reset;not frequently used
    GPIO_EXTI_MODE_e exti_mode; // �ⲿ�ж�ģʽ not frequently used
    uint16_t GPIO_Pin;          // ���ź�,
    // ��Щ������stm32f4xx_hal_gpio.h�ж���ĺ�!!! һ��Ҫע��
    // ���ȡ�����ֵ���ʱ����
    void (*gpio_model_callback)(struct tmpgpio *); // exti�жϻص�����
    void *id;                                      // ���ֲ�ͬ��GPIOʵ��

} GPIOInstance;

/**
 * @brief GPIO��ʼ�����ýṹ�嶨��
 *
 */
typedef struct
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // ����״̬,Set,Reset not frequently used
    GPIO_EXTI_MODE_e exti_mode; // �ⲿ�ж�ģʽ not frequently used
    uint16_t GPIO_Pin;          // ���ź�,@note ��������ź���GPIO_PIN_0,GPIO_PIN_1...
    // ��Щ������stm32f4xx_hal_gpio.h�ж���ĺ�!!! һ��Ҫע��

    void (*gpio_model_callback)(GPIOInstance *); // exti�жϻص�����
    void *id;                                    // ���ֲ�ͬ��GPIOʵ��

} GPIO_Init_Config_s;

/**
 * @brief ע��GPIOʵ��
 *
 * @param GPIO_config
 * @return GPIOInstance*
 */
GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config);

/**
 * @brief GPIO API,�л�GPIO��ƽ
 *
 * @param _instance
 */
void GPIOToggel(GPIOInstance *_instance);

/**
 * @brief ����GPIO��ƽ
 *
 * @param _instance
 */
void GPIOSet(GPIOInstance *_instance);

/**
 * @brief ��λGPIO��ƽ
 *
 * @param _instance
 */
void GPIOReset(GPIOInstance *_instance);

/**
 * @brief ��ȡGPIO��ƽ
 *
 * @param _instance
 * @return GPIO_PinState
 */
GPIO_PinState GPIORead(GPIOInstance *_instance);
