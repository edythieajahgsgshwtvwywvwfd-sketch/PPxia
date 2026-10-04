#ifndef __JDQ_H
#define __JDQ_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// 继电器通道编号（从 1 开始）
#define JDQ_CH1 1u
#define JDQ_CH2 2u
#define JDQ_CH3 3u
#define JDQ_CH4 4u

// 引脚：PA5 / PA8 / PA11 / PA12（CubeMX 配置为 GPIO 输出）
#define JDQ1_GPIO_PORT GPIOA
#define JDQ1_GPIO_PIN  GPIO_PIN_5
#define JDQ2_GPIO_PORT GPIOA
#define JDQ2_GPIO_PIN  GPIO_PIN_8
#define JDQ3_GPIO_PORT GPIOA
#define JDQ3_GPIO_PIN  GPIO_PIN_11
#define JDQ4_GPIO_PORT GPIOA
#define JDQ4_GPIO_PIN  GPIO_PIN_12

// 可按实际硬件改为 GPIO_PIN_RESET（低电平吸合）
#define JDQ_ACTIVE_LEVEL GPIO_PIN_SET

void jdq_init(void);
void jdq_on(uint8_t ch);
void jdq_off(uint8_t ch);
void jdq_set(uint8_t ch, uint8_t on);
void jdq_set_all(uint8_t ch1_on, uint8_t ch2_on, uint8_t ch3_on, uint8_t ch4_on);

#endif
