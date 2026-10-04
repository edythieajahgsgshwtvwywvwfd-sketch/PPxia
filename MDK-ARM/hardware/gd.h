#ifndef __GD_H
#define __GD_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// 光电通道编号
#define GD_CH1 1u
#define GD_CH2 2u
#define GD_CH3 3u
#define GD_CH4 4u

// 光电传感器输入引脚（GPIO 输入，CubeMX 初始化）
#define GD1_GPIO_PORT GPIOB
#define GD1_GPIO_PIN  GPIO_PIN_10
#define GD2_GPIO_PORT GPIOB
#define GD2_GPIO_PIN  GPIO_PIN_11
#define GD3_GPIO_PORT GPIOB
#define GD3_GPIO_PIN  GPIO_PIN_12
#define GD4_GPIO_PORT GPIOB
#define GD4_GPIO_PIN  GPIO_PIN_13

/**
 * @brief 初始化四路光电（PB10~PB13，GPIO 由 CubeMX 配置，此处空实现兼容旧调用）
 */
void gd_init(void);

/**
 * @brief 读取指定通道光电输入原始电平
 * @param ch 光电通道（GD_CH1~GD_CH4）
 * @return 1: 高电平；0: 低电平
 */
uint8_t gd_read_raw(uint8_t ch);

/**
 * @brief 判断指定通道是否检测到物体（传感器低电平有效）
 * @param ch 光电通道（GD_CH1~GD_CH4）
 * @return 1: 检测到物体（0V）；0: 未检测到（高电平）
 */
uint8_t gd_is_detected(uint8_t ch);

#endif
