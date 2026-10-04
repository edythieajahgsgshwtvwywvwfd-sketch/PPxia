#ifndef __SYSTEM_H
#define __SYSTEM_H

#define subdivision 16
#define PI 3.14159265358979323846f
#define Log 1

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "UART_NVIC.h"
#include "sys.h"
#include "OLED.h"
#include "app_main.h"
#include "delay.h"
#include "timer.h"
#include "task_execute.h"
#include "bsp_motor.h"
#include "uart_execute.h"
#include "Emm_V5.h"
#include "jdq.h"
#include "gd.h"
#include "servo.h"
#include "Encoder.h"
// #include "pressADC.h"  // PRESSADC 当前未使用。
#include "normal_motor.h"
#include "LobotServoController_F4.h"
#include "layser.h"

#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#include "stdbool.h"  

extern uint32_t time;                // 系统运行时间，单位 10 ms。
extern uint16_t Time_100ms;          // 100 ms 分频计数。
extern uint16_t Time_1s;             // 1 s 分频计数。
extern uint8_t time_stop_flag;       // 0 正常计时，1 暂停计时。
extern uint8_t OLED_Update_flag;     // OLED 待更新标志。
extern uint8_t OLED_Update_time;     // OLED 更新分频计数。
extern uint8_t run_mode;             // 当前运行模式。
extern float speed1, speed2, speed3, speed4; // 4 路电机速度值（旧接口）。
extern int16_t Motor6_encoder_Speed; // 电机 6 编码器速度。
extern int16_t Motor6_encoder_Speed; // Motor6 encoder speed
extern uint8_t g_motor6_turn180_done_flag;
extern float zdt_over_time[9];
extern volatile uint8_t zdt_run_flag[9];
extern float zdt_over_time[9];       // 步进电机剩余运行时间。
extern float zdt_over_time_max[9];   // 步进电机最大运行时间。
extern int now_pos[9];               // 软件记录的当前位置。
extern int tar_pos[9];               // 目标位置。
extern uint8_t ctrl_flag;
extern uint8_t motor_flag;
extern uint8_t step_ctrl_flag;
extern uint8_t servo_ctrl_flag;
extern uint8_t servo_finish_flag;
extern uint8_t gd_detected_flag;      // 光电 1（PB10）：1 检测到，0 未检测到。
extern uint8_t gd_detected_flag_2;    // 光电 2（PB11）：1 检测到，0 未检测到。
extern uint8_t gd_detected_flag_3;    // 光电 3（PB12）：1 检测到，0 未检测到。
extern uint8_t gd_detected_flag_4;    // 光电 4（PB13）：1 检测到，0 未检测到。
extern uint8_t jdq_ch1_detected_flag; // 继电器 1（PA5）：1 吸合，0 断开。
extern uint8_t jdq_ch2_detected_flag; // 继电器 2（PA8）：1 吸合，0 断开。
extern uint8_t jdq_ch3_detected_flag; // 继电器 3（PA11）：1 吸合，0 断开。
extern uint8_t jdq_ch4_detected_flag; // 继电器 4（PA12）：1 吸合，0 断开。
extern int16_t normal_motor_speed;    // 直流电机速度。
extern uint16_t distance;             // TOF 测距结果。
extern uint8_t system_ctrl_flag;      // 系统运行使能标志。
extern int Normal_motor_Speed_1;   // Motor1: TIM3_CH1/TIM3_CH2 (PA6/PA7)
extern int Normal_motor_Speed_2;   // Motor2: TIM3_CH3/TIM3_CH4 (PB0/PB1)
extern int Normal_motor_Speed_3;   // Motor3: TIM4_CH1/TIM4_CH2 (PD12/PD13)
extern int Normal_motor_Speed_4;   // Motor4: TIM4_CH3/TIM4_CH4 (PD14/PD15)
extern int Normal_motor_Speed_5;   // Motor5: TIM2_CH3/TIM2_CH4 (PA2/PA3)
extern int Normal_motor_Speed_6;   // Motor6: TIM12_CH1/TIM12_CH2 (PB14/PB15)
extern int Normal_motor_Speed_7;   // Motor7: TIM8_CH1/TIM8_CH2（PC6/PC7，特殊驱动）。
extern int Normal_motor_Speed_8;   // Motor8: TIM8_CH3/TIM8_CH4（PC8/PC9，特殊驱动）。
extern volatile uint8_t g_motor78_pause_decel_active;
extern uint32_t g_motor78_pause_decel_start_ms;
extern int16_t g_motor78_pause_init7;
extern int16_t g_motor78_pause_init8;
extern int32_t g_accel_time;
extern int32_t g_decel_time;
extern float g_max_pro_speed;
extern float g_max_speed;
extern float g_medium_speed;
extern float g_low_speed;
extern int32_t g_elapsed;
extern float g_target_speed;
extern int32_t g_motor1_stop_pulse; // 电机 1 光电触发时的脉冲位置，用于衔接绝对坐标。
extern uint32_t time_b;
extern int16_t g_sita_deg;
extern volatile uint16_t layser_distance;
extern volatile uint8_t layser_distance_update_flag;
extern volatile uint8_t layser_frame_error_count;
extern volatile uint32_t layser_last_update_time;
extern volatile uint32_t layser_last_update_ms;
extern volatile uint32_t layser_frame_count;
extern volatile uint16_t layser_dma_rx_pos;
extern volatile uint32_t layser_start_cmd_count;
extern volatile uint32_t layser_uart_recover_count;
extern volatile uint32_t layser_soft_recover_count;
// system.h
extern float servo_sync_start_time;    // 同步运动的起始时间。
extern float servo_sync_total_time;    // 同步运动的总时长。
extern int16_t servo_sync_start_angle; // 同步运动的起始角度。
extern int16_t servo_sync_end_angle;   // 同步运动的终止角度。
extern uint8_t servo_sync_active;      // 1 表示同步运动进行中。
extern uint8_t servo_sync_channel;
// 串口屏固定指令数组。
extern const uint8_t screen_step1_cmd[];
extern const uint8_t screen_step1_cmd_len;
extern const uint8_t screen_step2_cmd[];
extern const uint8_t screen_step2_cmd_len;
extern const uint8_t screen_step3_cmd[];
extern const uint8_t screen_step3_cmd_len;
extern const uint8_t screen_step4_cmd[];
extern const uint8_t screen_step4_cmd_len;
extern const uint8_t screen_step5_cmd[];
extern const uint8_t screen_step5_cmd_len;
extern const uint8_t screen_step6_cmd[];
extern const uint8_t screen_step6_cmd_len;
extern const uint8_t screen_step7_cmd[];
extern const uint8_t screen_step7_cmd_len;

extern const uint8_t screen_progress_finish_cmd[];
extern const uint8_t screen_progress_finish_cmd_len;
extern const uint8_t screen_finish_cmd[];
extern const uint8_t screen_finish_cmd_len;

extern const uint8_t screen_video_step1_cmd[];
extern const uint8_t screen_video_step1_cmd_len;
extern const uint8_t screen_video_step2_cmd[];
extern const uint8_t screen_video_step2_cmd_len;
extern const uint8_t screen_video_step3_cmd[];
extern const uint8_t screen_video_step3_cmd_len;
extern const uint8_t screen_video_step4_cmd[];
extern const uint8_t screen_video_step4_cmd_len;
extern const uint8_t screen_video_step5_cmd[];
extern const uint8_t screen_video_step5_cmd_len;
extern const uint8_t screen_video_step6_cmd[];
extern const uint8_t screen_video_step6_cmd_len;
extern const uint8_t screen_video_step7_cmd[];
extern const uint8_t screen_video_step7_cmd_len;
extern const uint8_t screen_progress_step1_cmd[];
extern const uint8_t screen_progress_step1_cmd_len;
extern const uint8_t screen_progress_step2_cmd[];
extern const uint8_t screen_progress_step2_cmd_len;
extern const uint8_t screen_progress_step3_cmd[];
extern const uint8_t screen_progress_step3_cmd_len;
extern const uint8_t screen_progress_step4_cmd[];
extern const uint8_t screen_progress_step4_cmd_len;
extern const uint8_t screen_progress_step5_cmd[];
extern const uint8_t screen_progress_step5_cmd_len;
extern const uint8_t screen_progress_step6_cmd[];
extern const uint8_t screen_progress_step6_cmd_len;
extern const uint8_t screen_progress_step7_cmd[];
extern const uint8_t screen_progress_step7_cmd_len;

u32 myabs(long int a);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
#endif 
