#ifndef __UART_EXECUTE_H_
#define __UART_EXECUTE_H_

#include "stdint.h"

/** USART1 调试口：轮询 DR 发送，与 HAL_UART_Receive_DMA 并存且不占用 huart1->gState（避免与 printf/fputc 的 HAL_Transmit 互斥 HAL_BUSY） */
void usart1_poll_send(const uint8_t *data, uint16_t len);

void process_uart_data(void);
/** USART2 串口屏：在 IT 收满一帧 6C..7F 后由 HAL_UART_RxCpltCallback 调用：转发到 USART1 + 执行命令 */
void usart2_isr_dispatch_screen_rx(void);
void usart2_screen_command_01_handler(void);
void usart2_screen_command_02_handler(void);
void usart2_screen_command_03_handler(void);
void usart2_screen_command_04_handler(void);
void screen_update_step(uint8_t step);
void screen_show_finish(void);


#endif
