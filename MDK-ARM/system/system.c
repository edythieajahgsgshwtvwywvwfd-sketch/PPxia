#include "system.h"

#ifndef SITA_START_DEG
#define SITA_START_DEG 45
#endif

// 定时器节拍计数。
uint32_t time;       // 系统运行时间，单位 10 ms。
uint16_t Time_100ms; // 100 ms 分频计数。
uint16_t Time_1s;    // 1 s 分频计数。
uint32_t time_b;
uint8_t time_stop_flag; // 时间暂停标志：0 正常计时，1 停止递增。

// OLED 更新调度。
uint8_t OLED_Update_flag; // OLED 待更新标志。
uint8_t OLED_Update_time; // OLED 更新分频计数。

// 主流程状态。
uint8_t run_mode = 0; // 当前运行模式。

// 张大头步进电机运行标志，索引为电机地址 1~8。
volatile uint8_t zdt_run_flag[9];

// 张大头步进电机的剩余运行时间及上限。
float zdt_over_time[9];
float zdt_over_time_max[9];
// 软件记录的当前位置。
int now_pos[9];
// 目标位置。
int tar_pos[9];

// 当前模式内部的步骤标志。
uint8_t ctrl_flag;

// 电机、步骤和舵机状态标志。
uint8_t motor_flag;
uint8_t step_ctrl_flag;
uint8_t servo_ctrl_flag;
uint8_t servo_finish_flag;

// 光电 1（PB10）：1 检测到物体，0 未检测到。
uint8_t gd_detected_flag = 0;
// 光电 2（PB11）：1 检测到物体，0 未检测到。
uint8_t gd_detected_flag_2 = 0;
// 光电 3（PB12）：1 检测到物体，0 未检测到。
uint8_t gd_detected_flag_3 = 0;
// 光电 4（PB13）：1 检测到物体，0 未检测到。
uint8_t gd_detected_flag_4 = 0;

// 继电器 1（PA5）状态：1 吸合，0 断开。
uint8_t jdq_ch1_detected_flag = 0;
// 继电器 2（PA8）状态：1 吸合，0 断开。
uint8_t jdq_ch2_detected_flag = 0;
// 继电器 3（PA11）状态：1 吸合，0 断开。
uint8_t jdq_ch3_detected_flag = 0;
// 继电器 4（PA12）状态：1 吸合，0 断开。
uint8_t jdq_ch4_detected_flag = 0;

// 串口屏控制状态。
uint8_t system_ctrl_flag = 0;

// 直流电机速度。
int16_t normal_motor_speed = 0;

// TOF 测距结果。
uint16_t distance = 0;

// system.c
float servo_sync_start_time = 0;
float servo_sync_total_time = 0;
int16_t servo_sync_start_angle = 0;
int16_t servo_sync_end_angle = 0;
uint8_t servo_sync_active = 0;
uint8_t servo_sync_channel = 0; // 0 表示尚未指定同步舵机通道。

// 8 路直流电机速度指令。
int Normal_motor_Speed_1;          // Motor1: TIM3_CH1/TIM3_CH2 (PA6/PA7)
int Normal_motor_Speed_2;          // Motor2: TIM3_CH3/TIM3_CH4 (PB0/PB1)
int Normal_motor_Speed_3;          // Motor3: TIM4_CH1/TIM4_CH2 (PD12/PD13)
int Normal_motor_Speed_4;          // Motor4: TIM4_CH3/TIM4_CH4 (PD14/PD15)
int Normal_motor_Speed_5;          // Motor5: TIM2_CH3/TIM2_CH4 (PA2/PA3)
int Normal_motor_Speed_6;          // Motor6: TIM12_CH1/TIM12_CH2 (PB14/PB15)
int Normal_motor_Speed_7;          // Motor7: TIM8_CH1/TIM8_CH2（PC6/PC7，特殊驱动）。
int Normal_motor_Speed_8;          // Motor8: TIM8_CH3/TIM8_CH4（PC8/PC9，特殊驱动）。

/* 暂停命令下，电机 7/8 做梯形减速；time 暂停后改用 HAL_GetTick() 计时。 */
volatile uint8_t g_motor78_pause_decel_active = 0;
uint32_t g_motor78_pause_decel_start_ms = 0;
int16_t g_motor78_pause_init7 = 0;
int16_t g_motor78_pause_init8 = 0;

// 全局速度规划参数。
int32_t g_accel_time = 50;      // 加速时间 500 ms，以 time 为单位。
int32_t g_decel_time = 50;      // 减速时间 500 ms，以 time 为单位。
float g_max_pro_speed = 80.0f;  // 流程允许的最高速度。
float g_max_speed = 60.0f;      // 高速档。
float g_medium_speed = 25.0f;   // 中速档。
float g_low_speed = 10.0f;      // 低速档。
// 速度规划过程变量。
int32_t g_elapsed;
float g_target_speed;
int32_t g_motor1_stop_pulse = 0;

int16_t g_sita_deg = SITA_START_DEG;

//encoder
int16_t Motor6_encoder_Speed = 0;
uint8_t g_motor6_turn180_done_flag = 0;

//layser
volatile uint16_t layser_distance = 0;
volatile uint8_t layser_distance_update_flag = 0;
volatile uint8_t layser_frame_error_count = 0;
volatile uint32_t layser_last_update_time = 0;
volatile uint32_t layser_last_update_ms = 0;
volatile uint32_t layser_frame_count = 0;
volatile uint16_t layser_dma_rx_pos = 0;
volatile uint32_t layser_start_cmd_count = 0;
volatile uint32_t layser_uart_recover_count = 0;
volatile uint32_t layser_soft_recover_count = 0;

// 串口屏固定指令；文本字段采用 GB2312 字节。

// t1.txt="(当前)步骤一：运输"
const uint8_t screen_step1_cmd[] = {0x74, 0x31, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x28, 0xB5, 0xB1, 0xC7, 0xB0, 0x29, 0xB2, 0xBD, 0xD6, 0xE8, 0xD2, 0xBB, 0xA3, 0xBA, 0xD4, 0xCB, 0xCA, 0xE4, 0x22, 0xFF, 0xFF, 0xFF};
const uint8_t screen_step1_cmd_len = sizeof(screen_step1_cmd);

// t1.txt="(当前)步骤二：穿虾"
const uint8_t screen_step2_cmd[] = {0x74, 0x31, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x28, 0xB5, 0xB1, 0xC7, 0xB0, 0x29, 0xB2, 0xBD, 0xD6, 0xE8, 0xB6, 0xFE, 0xA3, 0xBA, 0xB4, 0xA9, 0xCF, 0xBA, 0x22, 0xFF, 0xFF, 0xFF};
const uint8_t screen_step2_cmd_len = sizeof(screen_step2_cmd);

// t1.txt="(当前)步骤三：切尾"
const uint8_t screen_step3_cmd[] = {0x74, 0x31, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x28, 0xB5, 0xB1, 0xC7, 0xB0, 0x29, 0xB2, 0xBD, 0xD6, 0xE8, 0xC8, 0xFD, 0xA3, 0xBA, 0xC7, 0xD0, 0xCE, 0xB2, 0x22, 0xFF, 0xFF, 0xFF};
const uint8_t screen_step3_cmd_len = sizeof(screen_step3_cmd);

// t1.txt="(当前)步骤四：切头"
const uint8_t screen_step4_cmd[] = {0x74, 0x31, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x28, 0xB5, 0xB1, 0xC7, 0xB0, 0x29, 0xB2, 0xBD, 0xD6, 0xE8, 0xCB, 0xC4, 0xA3, 0xBA, 0xC7, 0xD0, 0xCD, 0xB7, 0x22, 0xFF, 0xFF, 0xFF};
const uint8_t screen_step4_cmd_len = sizeof(screen_step4_cmd);

// t1.txt="(当前)步骤五：切侧身"
const uint8_t screen_step5_cmd[] = {0x74, 0x31, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x28, 0xB5, 0xB1, 0xC7, 0xB0, 0x29, 0xB2, 0xBD, 0xD6, 0xE8, 0xCE, 0xE5, 0xA3, 0xBA, 0xC7, 0xD0, 0xB2, 0xE0, 0xC9, 0xED, 0x22, 0xFF, 0xFF, 0xFF};
const uint8_t screen_step5_cmd_len = sizeof(screen_step5_cmd);

// t1.txt="(当前)步骤六：刺皮皮虾"
const uint8_t screen_step6_cmd[] = {0x74, 0x31, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x28, 0xB5, 0xB1, 0xC7, 0xB0, 0x29, 0xB2, 0xBD, 0xD6, 0xE8, 0xC1, 0xF9, 0xA3, 0xBA, 0xB4, 0xCC, 0xC6, 0xA4, 0xC6, 0xA4, 0xCF, 0xBA, 0x22, 0xFF, 0xFF, 0xFF};
const uint8_t screen_step6_cmd_len = sizeof(screen_step6_cmd);

// t1.txt="(当前)步骤七：去壳"
const uint8_t screen_step7_cmd[] = {0x74, 0x31, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x28, 0xB5, 0xB1, 0xC7, 0xB0, 0x29, 0xB2, 0xBD, 0xD6, 0xE8, 0xC6, 0xDF, 0xA3, 0xBA, 0xC8, 0xA5, 0xBF, 0xC7, 0x22, 0xFF, 0xFF, 0xFF};
const uint8_t screen_step7_cmd_len = sizeof(screen_step7_cmd);

// j0.val=10（步骤一进度 10%）。

// gm0.vid=0~6（按步骤切换演示视频）。
const uint8_t screen_video_step1_cmd[] = {0x67, 0x6D, 0x30, 0x2E, 0x76, 0x69, 0x64, 0x3D, 0x30, 0xFF, 0xFF, 0xFF};
const uint8_t screen_video_step1_cmd_len = sizeof(screen_video_step1_cmd);
const uint8_t screen_video_step2_cmd[] = {0x67, 0x6D, 0x30, 0x2E, 0x76, 0x69, 0x64, 0x3D, 0x31, 0xFF, 0xFF, 0xFF};
const uint8_t screen_video_step2_cmd_len = sizeof(screen_video_step2_cmd);
const uint8_t screen_video_step3_cmd[] = {0x67, 0x6D, 0x30, 0x2E, 0x76, 0x69, 0x64, 0x3D, 0x32, 0xFF, 0xFF, 0xFF};
const uint8_t screen_video_step3_cmd_len = sizeof(screen_video_step3_cmd);
const uint8_t screen_video_step4_cmd[] = {0x67, 0x6D, 0x30, 0x2E, 0x76, 0x69, 0x64, 0x3D, 0x33, 0xFF, 0xFF, 0xFF};
const uint8_t screen_video_step4_cmd_len = sizeof(screen_video_step4_cmd);
const uint8_t screen_video_step5_cmd[] = {0x67, 0x6D, 0x30, 0x2E, 0x76, 0x69, 0x64, 0x3D, 0x34, 0xFF, 0xFF, 0xFF};
const uint8_t screen_video_step5_cmd_len = sizeof(screen_video_step5_cmd);
const uint8_t screen_video_step6_cmd[] = {0x67, 0x6D, 0x30, 0x2E, 0x76, 0x69, 0x64, 0x3D, 0x35, 0xFF, 0xFF, 0xFF};
const uint8_t screen_video_step6_cmd_len = sizeof(screen_video_step6_cmd);
const uint8_t screen_video_step7_cmd[] = {0x67, 0x6D, 0x30, 0x2E, 0x76, 0x69, 0x64, 0x3D, 0x36, 0xFF, 0xFF, 0xFF};
const uint8_t screen_video_step7_cmd_len = sizeof(screen_video_step7_cmd);
const uint8_t screen_progress_step1_cmd[] = {0x6A, 0x30, 0x2E, 0x76, 0x61, 0x6C, 0x3D, 0x31, 0x30, 0xFF, 0xFF, 0xFF};
const uint8_t screen_progress_step1_cmd_len = sizeof(screen_progress_step1_cmd);

// j0.val=25（步骤二进度 25%）。
const uint8_t screen_progress_step2_cmd[] = {0x6A, 0x30, 0x2E, 0x76, 0x61, 0x6C, 0x3D, 0x32, 0x35, 0xFF, 0xFF, 0xFF};
const uint8_t screen_progress_step2_cmd_len = sizeof(screen_progress_step2_cmd);

// j0.val=35（步骤三进度 35%）。
const uint8_t screen_progress_step3_cmd[] = {0x6A, 0x30, 0x2E, 0x76, 0x61, 0x6C, 0x3D, 0x33, 0x35, 0xFF, 0xFF, 0xFF};
const uint8_t screen_progress_step3_cmd_len = sizeof(screen_progress_step3_cmd);

// j0.val=50（步骤四进度 50%）。
const uint8_t screen_progress_step4_cmd[] = {0x6A, 0x30, 0x2E, 0x76, 0x61, 0x6C, 0x3D, 0x35, 0x30, 0xFF, 0xFF, 0xFF};
const uint8_t screen_progress_step4_cmd_len = sizeof(screen_progress_step4_cmd);

// j0.val=65（步骤五进度 65%）。
const uint8_t screen_progress_step5_cmd[] = {0x6A, 0x30, 0x2E, 0x76, 0x61, 0x6C, 0x3D, 0x36, 0x35, 0xFF, 0xFF, 0xFF};
const uint8_t screen_progress_step5_cmd_len = sizeof(screen_progress_step5_cmd);

// j0.val=80（步骤六进度 80%）。
const uint8_t screen_progress_step6_cmd[] = {0x6A, 0x30, 0x2E, 0x76, 0x61, 0x6C, 0x3D, 0x38, 0x30, 0xFF, 0xFF, 0xFF};
const uint8_t screen_progress_step6_cmd_len = sizeof(screen_progress_step6_cmd);

// j0.val=95（步骤七进度 95%）。
const uint8_t screen_progress_step7_cmd[] = {0x6A, 0x30, 0x2E, 0x76, 0x61, 0x6C, 0x3D, 0x39, 0x35, 0xFF, 0xFF, 0xFF};
const uint8_t screen_progress_step7_cmd_len = sizeof(screen_progress_step7_cmd);

// j0.val=100（完成进度 100%）。
const uint8_t screen_progress_finish_cmd[] = {0x6A, 0x30, 0x2E, 0x76, 0x61, 0x6C, 0x3D, 0x31, 0x30, 0x30, 0xFF, 0xFF, 0xFF};
const uint8_t screen_progress_finish_cmd_len = sizeof(screen_progress_finish_cmd);

// page finish（完成页面）。
const uint8_t screen_finish_cmd[] = {0x70, 0x61, 0x67, 0x65, 0x20, 0x66, 0x69, 0x6E, 0x69, 0x73, 0x68, 0xFF, 0xFF, 0xFF};
const uint8_t screen_finish_cmd_len = sizeof(screen_finish_cmd);


u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/*
int fputc(int ch, FILE *f)
{
		uint8_t temp[1] = {ch};
		HAL_UART_Transmit(&huart3, temp, 1, 2); // 旧版 USART3 printf 重定向。
		return ch;
}
*/
// 使用 USART1 轮询发送实现 printf，避免与 HAL UART 状态机互斥。
#include <stdio.h>

// 重定向 printf 输出。
int fputc(int ch, FILE *f)
{
    uint8_t c = (uint8_t)ch;
    usart1_poll_send(&c, 1U);
    return ch;
}
int fgetc(FILE *f)
 {
    // 等待 USART1 接收到一个字节。
    while((USART1->SR & USART_SR_RXNE) == 0);
    return (int)USART1->DR;
}

