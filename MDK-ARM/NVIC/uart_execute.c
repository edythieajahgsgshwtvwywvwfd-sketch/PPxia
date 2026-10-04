#include "system.h"
#include "uart_execute.h"

void usart1_poll_send(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (data == NULL || len == 0U)
    {
        return;
    }
    for (i = 0; i < len; i++)
    {
        while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
        {
        }
        huart1.Instance->DR = (uint8_t)(data[i] & 0xFFU);
    }
    while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC))
    {
    }
}

static void usart1_debug_send_text(const char *text)
{
    usart1_poll_send((const uint8_t *)text, (uint16_t)strlen(text));
}

static void usart1_debug_send_hex_byte(uint8_t b)
{
    char hex[3];
    const char table[] = "0123456789ABCDEF";
    hex[0] = table[(b >> 4) & 0x0F];
    hex[1] = table[b & 0x0F];
    hex[2] = '\0';
    usart1_debug_send_text(hex);
}

static void usart2_debug_print_frame_to_usart1(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    usart1_debug_send_text("[USART2 RX] ");
    for (i = 0; i < len; i++)
    {
        usart1_debug_send_hex_byte(data[i]);
        if (i + 1U < len)
        {
            usart1_debug_send_text(" ");
        }
    }
    usart1_debug_send_text("\r\n");
}

/** 立即把 8 路直流电机 PWM 清零；只改速度变量并不会立刻更新硬件 PWM。 */
static void machine_apply_dc_pwm_all_zero(void)
{
    allmotor(0, 0, 0, 0, 0, 0, 0, 0);
}

/**
 * 通过 UART4 向张大头步进电机发送急停命令。
 * 地址与 task_execute 中使用的地址一致（1、4、5、6、7、8），并同步清除
 * zdt_run_flag，避免任务状态机仍认为电机正在运行。
 */
static void machine_estop_steppers_on_uart4(void)
{
    static const uint8_t kStepperAddrs[] = {1, 4, 5, 6, 7, 8};
    unsigned i;
    for (i = 0; i < sizeof(kStepperAddrs) / sizeof(kStepperAddrs[0]); i++)
    {
        uint8_t addr = kStepperAddrs[i];
        Emm_V5_Stop_Now(addr, false);
        zdt_run_flag[addr] = 0;
    }
}

// 串口屏动作命令。
void usart2_screen_command_01_handler(void)
{
    // 命令 1：进入流程起始状态并开始工作。
    g_motor78_pause_decel_active = 0;
    system_ctrl_flag = 1;
    run_mode = 5;
    ctrl_flag = 0;
    time_b = time;
    time_stop_flag = 0;
}

void usart2_screen_command_02_handler(void)
{
    // 命令 2：暂停；1~6 路立即停止，7/8 路按梯形曲线减速到 0。
    time_stop_flag = 1;

    g_motor78_pause_init7 = (int16_t)Normal_motor_Speed_7;
    g_motor78_pause_init8 = (int16_t)Normal_motor_Speed_8;

    Normal_motor_Speed_1 = 0;
    Normal_motor_Speed_2 = 0;
    Normal_motor_Speed_3 = 0;
    Normal_motor_Speed_4 = 0;
    Normal_motor_Speed_5 = 0;
    Normal_motor_Speed_6 = 0;

    if (g_motor78_pause_init7 == 0 && g_motor78_pause_init8 == 0)
    {
        g_motor78_pause_decel_active = 0;
        Normal_motor_Speed_7 = 0;
        Normal_motor_Speed_8 = 0;
        machine_apply_dc_pwm_all_zero();
    }
    else
    {
        g_motor78_pause_decel_start_ms = HAL_GetTick();
        g_motor78_pause_decel_active = 1;
        allmotor(0, 0, 0, 0, 0, 0, Normal_motor_Speed_7, Normal_motor_Speed_8);
    }

    machine_estop_steppers_on_uart4();
    system_ctrl_flag = 0;
}

void usart2_screen_command_03_handler(void)
{
    // 命令 3：复位控制状态并停止所有电机。
    g_motor78_pause_decel_active = 0;
    Normal_motor_Speed_1 = 0;
    Normal_motor_Speed_2 = 0;
    Normal_motor_Speed_3 = 0;
    Normal_motor_Speed_4 = 0;
    Normal_motor_Speed_5 = 0;
    Normal_motor_Speed_6 = 0;
    Normal_motor_Speed_7 = 0;
    Normal_motor_Speed_8 = 0;
    machine_apply_dc_pwm_all_zero();
    machine_estop_steppers_on_uart4();
    system_ctrl_flag = 0;
    run_mode = 0;
    ctrl_flag = 0;
    time_b = time;
    time_stop_flag = 1;
}

void usart2_screen_command_04_handler(void)
{
    g_motor78_pause_decel_active = 0;
    /*
     * 命令 4：继续工作。仅恢复 system_ctrl_flag 和时间计数，不自动重发
     * zuobiao_single_custom 命令。该函数发令后会立即把 now_pos 更新为目标值；
     * 如果停机时电机尚未到位，软件位置可能与实际位置不一致。应在停机时同步
     * now_pos，或读取驱动器实际位置后再继续。
     */
    system_ctrl_flag = 1;
    time_stop_flag = 0;
}

void usart2_isr_dispatch_screen_rx(void)
{
    if (RXdata_2[1].rx_finall_flag == 0U)
    {
        return;
    }

    /* 把原始命令帧转发到 USART1，便于串口助手以十六进制查看 6C 01 xx 7F。 */
    usart1_poll_send(RXdata_2[1].Rx2, RXdata_2[1].number_finall);
    usart1_poll_send((const uint8_t *)"\r\n", 2U);

    usart2_debug_print_frame_to_usart1(RXdata_2[1].Rx2, RXdata_2[1].number_finall);

    if (RXdata_2[1].number_finall == 4U &&
        RXdata_2[1].Rx2[0] == 0x6CU &&
        RXdata_2[1].Rx2[1] == 0x01U &&
        RXdata_2[1].Rx2[3] == 0x7FU)
    {
        switch (RXdata_2[1].Rx2[2])
        {
        case 0x01:
            //  usart1_screen_command_01_handler();
            usart1_debug_send_text("[USART2 CMD] START\r\n");
            break;
        case 0x02:
            usart2_screen_command_02_handler();
            usart1_debug_send_text("[USART2 CMD] STOP\r\n");
            break;
        case 0x03:
            usart2_screen_command_03_handler();
            usart1_debug_send_text("[USART2 CMD] RESET\r\n");
            break;
        case 0x04:
            usart2_screen_command_04_handler();
            usart1_debug_send_text("[USART2 CMD] RESUME\r\n");
            break;
        default:
            usart1_debug_send_text("[USART2 CMD] UNKNOWN\r\n");
            break;
        }
    }
    else
    {
        usart1_debug_send_text("[USART2 CMD] INVALID FRAME\r\n");
    }

    RXdata_2[1].rx_finall_flag = 0;
    RXdata_2[1].rx_finall_protect = 0;
}

// 处理各串口已接收的数据，由主循环调用。
void process_uart_data(void)
{
    usart2_isr_dispatch_screen_rx();
    /* USART1 的 6C/7F 组帧与回发在 HAL_UART_RxCpltCallback 中完成。 */
    /* USART2 的串口屏协议由 HAL_UART_RxCpltCallback 收帧，再在此处分发。 */

    // USART3 当前未启用，跳过 DMA 发送。

    // UART4 仅接收；UART_NVIC.c 会调用 zdt_stop_read() 处理步进电机回包。
    
    // UART5：无帧头协议，收到完整帧后通过 DMA 回发，并用保护标志防止重入。
    if (RXdata_5[1].rx_finall_flag && !RXdata_5[1].rx_finall_protect)
    {
        RXdata_5[1].rx_finall_protect = 1;
       HAL_UART_Transmit_DMA(&huart5, RXdata_5[1].Rx2, RXdata_5[1].number_finall);
       
    }

    // USART6 的 PC6/PC7 已复用为 TIM8_CH1/CH2，不再执行 DMA 发送。
}

// DMA 发送完成回调：清除标志，允许发送下一帧。
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* USART1/2 未使用 HAL_UART_Transmit_DMA；不要在此清除它们的 RX 完成标志，以免与主循环竞态。 */
    if (huart->Instance == USART3)
    {
        RXdata_3[1].rx_finall_flag = 0;
        RXdata_3[1].rx_finall_protect = 0;
    }
    else if (huart->Instance == UART5)
    {
        RXdata_5[1].rx_finall_flag = 0;
        RXdata_5[1].rx_finall_protect = 0;
    }
    else if (huart->Instance == USART6)
    {
        RXdata_6[1].rx_finall_flag = 0;
        RXdata_6[1].rx_finall_protect = 0;
    }
    // UART4 不发送，无需处理
}

/**
 * @brief 更新串口屏的步骤文字、进度和演示视频。
 * @param step 步骤编号（1~7）。
 */
void screen_update_step(uint8_t step)
{
    const uint8_t *step_cmd = NULL;
    uint8_t step_len = 0;
    const uint8_t *progress_cmd = NULL;
    uint8_t progress_len = 0;
    const uint8_t *video_cmd = NULL;
    uint8_t video_len = 0;
    
    switch(step)
    {
        case 1:
            step_cmd = screen_step1_cmd;
            step_len = screen_step1_cmd_len;
            progress_cmd = screen_progress_step1_cmd;
            progress_len = screen_progress_step1_cmd_len;
            video_cmd = screen_video_step1_cmd;
            video_len = screen_video_step1_cmd_len;
            break;
        case 2:
            step_cmd = screen_step2_cmd;
            step_len = screen_step2_cmd_len;
            progress_cmd = screen_progress_step2_cmd;
            progress_len = screen_progress_step2_cmd_len;
            video_cmd = screen_video_step2_cmd;
            video_len = screen_video_step2_cmd_len;
            break;
        case 3:
            step_cmd = screen_step3_cmd;
            step_len = screen_step3_cmd_len;
            progress_cmd = screen_progress_step3_cmd;
            progress_len = screen_progress_step3_cmd_len;
            video_cmd = screen_video_step3_cmd;
            video_len = screen_video_step3_cmd_len;
            break;
        case 4:
            step_cmd = screen_step4_cmd;
            step_len = screen_step4_cmd_len;
            progress_cmd = screen_progress_step4_cmd;
            progress_len = screen_progress_step4_cmd_len;
            video_cmd = screen_video_step4_cmd;
            video_len = screen_video_step4_cmd_len;
            break;
        case 5:
            step_cmd = screen_step5_cmd;
            step_len = screen_step5_cmd_len;
            progress_cmd = screen_progress_step5_cmd;
            progress_len = screen_progress_step5_cmd_len;
            video_cmd = screen_video_step5_cmd;
            video_len = screen_video_step5_cmd_len;
            break;
        case 6:
            step_cmd = screen_step6_cmd;
            step_len = screen_step6_cmd_len;
            progress_cmd = screen_progress_step6_cmd;
            progress_len = screen_progress_step6_cmd_len;
            video_cmd = screen_video_step6_cmd;
            video_len = screen_video_step6_cmd_len;
            break;
        case 7:
            step_cmd = screen_step7_cmd;
            step_len = screen_step7_cmd_len;
            progress_cmd = screen_progress_step7_cmd;
            progress_len = screen_progress_step7_cmd_len;
            video_cmd = screen_video_step7_cmd;
            video_len = screen_video_step7_cmd_len;
            break;
        default:
            return;
    }
    
    // 使用 USART2 发送步骤文字。
    HAL_UART_Transmit(&huart2, (uint8_t*)step_cmd, step_len, 100);
    delay_ms(3);
    // 使用 USART2 发送进度条
    HAL_UART_Transmit(&huart2, (uint8_t*)progress_cmd, progress_len, 100);
    delay_ms(3);
    // 使用 USART2 切换当前步骤的视频。
    HAL_UART_Transmit(&huart2, (uint8_t*)video_cmd, video_len, 100);
    delay_ms(3);
}

/**
 * @brief 显示完成页面
 */
void screen_show_finish(void)
{
    // 使用 USART2 发送 100% 进度。
    HAL_UART_Transmit(&huart2, (uint8_t*)screen_progress_finish_cmd, screen_progress_finish_cmd_len, 100);
    
    // 延迟后切换到完成页面。
    delay_ms(500);
    HAL_UART_Transmit(&huart2, (uint8_t*)screen_finish_cmd, screen_finish_cmd_len, 100);
}
