#include "gd.h"

static uint8_t gd_get_pin(uint8_t ch, GPIO_TypeDef **port, uint16_t *pin)
{
    if (ch == GD_CH1) {
        *port = GD1_GPIO_PORT;
        *pin = GD1_GPIO_PIN;
        return 1u;
    }
    if (ch == GD_CH2) {
        *port = GD2_GPIO_PORT;
        *pin = GD2_GPIO_PIN;
        return 1u;
    }
    if (ch == GD_CH3) {
        *port = GD3_GPIO_PORT;
        *pin = GD3_GPIO_PIN;
        return 1u;
    }
    if (ch == GD_CH4) {
        *port = GD4_GPIO_PORT;
        *pin = GD4_GPIO_PIN;
        return 1u;
    }
    return 0u;
}

void gd_init(void)
{
    // GPIO 由 CubeMX 统一初始化，这里保留空实现以兼容旧调用。
}

uint8_t gd_read_raw(uint8_t ch)
{
    GPIO_TypeDef *port = NULL;
    uint16_t pin = 0;

    if (!gd_get_pin(ch, &port, &pin)) {
        return 0u;
    }

    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;
}

uint8_t gd_is_detected(uint8_t ch)
{
    // 传感器：检测到物体输出 0V（低电平）
    return (gd_read_raw(ch) == 0u) ? 1u : 0u;
}
