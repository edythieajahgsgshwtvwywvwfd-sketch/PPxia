#include "jdq.h"
#include "gpio.h"
#include "system.h"

static GPIO_PinState jdq_active(void)
{
    return JDQ_ACTIVE_LEVEL;
}

static GPIO_PinState jdq_inactive(void)
{
    return (JDQ_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

/** 通道 -> 端口与引脚，非法通道返回 pin=0 */
static uint8_t jdq_get_pin(uint8_t ch, GPIO_TypeDef **port, uint16_t *pin)
{
    if (ch == JDQ_CH1) {
        *port = JDQ1_GPIO_PORT;
        *pin = JDQ1_GPIO_PIN;
        return 1u;
    }
    if (ch == JDQ_CH2) {
        *port = JDQ2_GPIO_PORT;
        *pin = JDQ2_GPIO_PIN;
        return 1u;
    }
    if (ch == JDQ_CH3) {
        *port = JDQ3_GPIO_PORT;
        *pin = JDQ3_GPIO_PIN;
        return 1u;
    }
    if (ch == JDQ_CH4) {
        *port = JDQ4_GPIO_PORT;
        *pin = JDQ4_GPIO_PIN;
        return 1u;
    }
    return 0u;
}

static void jdq_set_flag(uint8_t ch, uint8_t on)
{
    uint8_t v = on ? 1u : 0u;
    if (ch == JDQ_CH1) {
        jdq_ch1_detected_flag = v;
    } else if (ch == JDQ_CH2) {
        jdq_ch2_detected_flag = v;
    } else if (ch == JDQ_CH3) {
        jdq_ch3_detected_flag = v;
    } else if (ch == JDQ_CH4) {
        jdq_ch4_detected_flag = v;
    }
}

void jdq_init(void)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    for (uint8_t ch = JDQ_CH1; ch <= JDQ_CH4; ch++) {
        if (jdq_get_pin(ch, &port, &pin)) {
            HAL_GPIO_WritePin(port, pin, jdq_inactive());
        }
    }
    jdq_ch1_detected_flag = 0;
    jdq_ch2_detected_flag = 0;
    jdq_ch3_detected_flag = 0;
    jdq_ch4_detected_flag = 0;
}

void jdq_on(uint8_t ch)
{
    GPIO_TypeDef *port = NULL;
    uint16_t pin = 0;

    if (!jdq_get_pin(ch, &port, &pin)) {
        return;
    }
    HAL_GPIO_WritePin(port, pin, jdq_active());
    jdq_set_flag(ch, 1u);
}

void jdq_off(uint8_t ch)
{
    GPIO_TypeDef *port = NULL;
    uint16_t pin = 0;

    if (!jdq_get_pin(ch, &port, &pin)) {
        return;
    }
    HAL_GPIO_WritePin(port, pin, jdq_inactive());
    jdq_set_flag(ch, 0u);
}

void jdq_set(uint8_t ch, uint8_t on)
{
    if (on) {
        jdq_on(ch);
    } else {
        jdq_off(ch);
    }
}

void jdq_set_all(uint8_t ch1_on, uint8_t ch2_on, uint8_t ch3_on, uint8_t ch4_on)
{
    jdq_set(JDQ_CH1, ch1_on);
    jdq_set(JDQ_CH2, ch2_on);
    jdq_set(JDQ_CH3, ch3_on);
    jdq_set(JDQ_CH4, ch4_on);
}
