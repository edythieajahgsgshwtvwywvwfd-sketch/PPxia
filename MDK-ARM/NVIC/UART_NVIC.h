// UART_NVIC.h
#ifndef __UART_NVIC_H__
#define __UART_NVIC_H__

#include "stdint.h"
#include "usart.h"

#define RX_BUFFER_SIZE 256

typedef struct
{
    uint8_t Rx[RX_BUFFER_SIZE];         // ← 保留！用于 timer.c 判断
    uint8_t Rx2[RX_BUFFER_SIZE];        // 接收缓存（DMA 解析用）
    uint8_t number;                     // 兼容旧代码
    uint16_t number_finall;             // 实际帧长度
    volatile uint8_t rx_finall_flag;    // 帧完成标志
    volatile uint8_t rx_finall_protect; // 防重复提交发送
} Serial_RxPacket;

// 帧状态机
typedef enum
{
    WAIT_FRAME_HEAD = 0,
    WAIT_FRAME_DATA
} FrameState;

// 外部声明：每个串口独立
extern Serial_RxPacket RXdata[4];   // USART1
extern Serial_RxPacket RXdata_2[4]; // USART2
extern Serial_RxPacket RXdata_3[4]; // USART3
extern Serial_RxPacket RXdata_4[4]; // UART4
extern Serial_RxPacket RXdata_5[4]; // UART5
extern Serial_RxPacket RXdata_6[4]; // USART6

extern FrameState uart1_frame_state;
extern FrameState uart2_frame_state;
extern FrameState uart3_frame_state;
extern FrameState uart4_frame_state;
extern FrameState uart5_frame_state;
extern FrameState uart6_frame_state;

// DMA 接收缓冲区
extern uint8_t dma_rx_buf1[RX_BUFFER_SIZE];
extern uint8_t dma_rx_buf2[RX_BUFFER_SIZE];
extern uint8_t dma_rx_buf3[RX_BUFFER_SIZE];
extern uint8_t dma_rx_buf4[RX_BUFFER_SIZE];
extern uint8_t dma_rx_buf5[RX_BUFFER_SIZE];
extern uint8_t dma_rx_buf6[RX_BUFFER_SIZE];

// 函数声明
void UART_DMA_Init(void);
void UART_IDLE_Callback(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size);
/** Circular DMA RX：根据 DMA 写指针增量把 [old,pos) 段交给解析；由 USARTx_IRQHandler(IDLE) 调用 */
void UART_IDLE_DmaRingHandler(UART_HandleTypeDef *huart, uint8_t *dma_buf);
void parse_uart_frames(void);
void main_loop(void);

#endif
