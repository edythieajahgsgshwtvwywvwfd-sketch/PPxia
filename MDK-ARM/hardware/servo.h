#ifndef SERVO_H_INCLUDED
#define SERVO_H_INCLUDED

#include "stm32f4xx_hal.h"
#include <stdint.h>

// 舵机常用脉宽范围（单位：us）。
#define SERVO_PULSE_MIN_US 500u
#define SERVO_PULSE_MAX_US 2500u

// sita 轨迹参数（可根据实际标定微调）。
#define SITA_START_DEG            45
#define SITA_END_DEG              135
#define STEPPER78_PULSE_PER_UNIT  31.25f  // 51.2 个单位对应一圈（1600 脉冲）时，约为 1600/51.2。
#define STEPPER78_VEL             180
#define STEPPER78_ACC             206

// 舵机通道编号
// 1~4: TIM1_CH1~CH4 -> PE9/PE11/PE13/PE14
// 5~6: TIM5_CH1~CH2 -> PA0/PA1
#define SERVO_CH1 1u
#define SERVO_CH2 2u
#define SERVO_CH3 3u
#define SERVO_CH4 4u
#define SERVO_CH5 5u
#define SERVO_CH6 6u

/**
 * @brief 初始化舵机 PWM。
 * @note 当前仅启动 TIM1_CH1~4；TIM5_CH1~2 已复用为电机 6 编码器。
 */
void servo_init(void);

/**
 * @brief 设置指定通道的 180 度舵机角度。
 * @param ch 舵机通道（SERVO_CH1~SERVO_CH6）。
 * @param angle 角度，范围 0~180 度。
 * @note 默认映射为 0 度 = 500 us，180 度 = 2500 us。
 */
void servo_set_angle(uint8_t ch, uint16_t angle);
void servo_set_angle_270(uint8_t ch, uint16_t angle);
void servo_break(void); // 皮皮虾剥壳动作。
void servo_break_by_servo_angle(int16_t servo_start_deg, int16_t servo_end_deg);
void servo_break_smooth_start(int16_t servo_start_deg, int16_t servo_end_deg, uint8_t mirror_motor7);
void servo_break_smooth_update(void);
/**
 * @brief 设置指定通道的舵机脉宽。
 * @param ch 舵机通道（SERVO_CH1~SERVO_CH6）。
 * @param us 脉宽，建议范围 500~2500 us。
 */
void servo_set_pulse_us(uint8_t ch, uint16_t us);
void servo_formula(int16_t sita_deg);
void servo_start_sync(uint8_t servo_ch, uint8_t motor_id, int16_t start_angle, int16_t end_angle);
void servo_start_sync_fast(uint8_t servo_ch, uint8_t motor_id, int16_t start_angle, int16_t end_angle);
void servo_update_sync(void);

#endif
