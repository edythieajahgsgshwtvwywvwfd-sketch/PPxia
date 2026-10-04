#include "UART_NVIC.h"
#include "system.h"
#include "uart_execute.h"
#include "LobotServoController_F4.h"
#include "layser.h"

#define USART3_DISABLED 1

// 接收包定义（每个串口独立）。
Serial_RxPacket RXdata[4] = {0};
Serial_RxPacket RXdata_2[4] = {0};
Serial_RxPacket RXdata_3[4] = {0};
Serial_RxPacket RXdata_4[4] = {0};
Serial_RxPacket RXdata_5[4] = {0};
Serial_RxPacket RXdata_6[4] = {0};

// 帧状态。
FrameState uart1_frame_state = WAIT_FRAME_HEAD;
FrameState uart2_frame_state = WAIT_FRAME_HEAD;
FrameState uart3_frame_state = WAIT_FRAME_HEAD;
FrameState uart4_frame_state = WAIT_FRAME_HEAD;
FrameState uart5_frame_state = WAIT_FRAME_HEAD;
FrameState uart6_frame_state = WAIT_FRAME_HEAD;

// 临时帧缓冲区
static uint8_t uart1_temp_frame_buffer[RX_BUFFER_SIZE];
static uint8_t uart2_temp_frame_buffer[RX_BUFFER_SIZE];
static uint8_t uart4_temp_frame_buffer[RX_BUFFER_SIZE];
static uint8_t uart6_temp_frame_buffer[RX_BUFFER_SIZE];

// 当前帧索引（必须在帧结束或丢弃时清零）。
static uint16_t uart1_current_frame_index = 0;
static uint16_t uart2_current_frame_index = 0;
static uint16_t uart4_current_frame_index = 0;
static uint16_t uart6_current_frame_index = 0;

// DMA 接收缓冲区。
uint8_t dma_rx_buf1[RX_BUFFER_SIZE];
uint8_t dma_rx_buf2[RX_BUFFER_SIZE];
uint8_t dma_rx_buf3[RX_BUFFER_SIZE];
uint8_t dma_rx_buf4[RX_BUFFER_SIZE];
uint8_t dma_rx_buf5[RX_BUFFER_SIZE];
uint8_t dma_rx_buf6[RX_BUFFER_SIZE];

/* USART1 / USART2：单字节中断接收，不使用 DMA。 */
static uint8_t uart1_it_rx_byte;
static uint8_t uart2_it_rx_byte;

/* UART4 循环 DMA 的软件读指针。 */
static uint16_t uart3_dma_rx_last_pos;
static uint16_t uart4_dma_rx_last_pos;

void layser_restart_uart_rx(void)
{
#if USART3_DISABLED
    return;
#else
    layser_uart_recover_count++;
    (void)HAL_UART_AbortReceive(&huart3);
    uart3_dma_rx_last_pos = 0U;
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    if (HAL_UART_Receive_DMA(&huart3, dma_rx_buf3, RX_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
#endif
}

// 初始化各串口接收。
void UART_DMA_Init(void)
{
    /* USART1：关闭 DMA 与 IDLE，改用逐字节 RXNE 中断；按 6C/7F 组帧后原样回发。 */
    (void)HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    if (HAL_UART_Receive_IT(&huart1, &uart1_it_rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }

    (void)HAL_UART_AbortReceive(&huart2);
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
    if (HAL_UART_Receive_IT(&huart2, &uart2_it_rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }

    // USART3 当前未启用，跳过 DMA 启动。
    // layser_restart_uart_rx();  // USART3 currently disabled

    HAL_UART_Receive_DMA(&huart4, dma_rx_buf4, RX_BUFFER_SIZE);
    // USART6 引脚 PC6/PC7 已复用为 TIM8_CH1/CH2，此处不再启动 DMA。

    // USART3 当前未启用，跳过 IDLE 中断使能。
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    // USART6 未启用，只保留其他串口的 IDLE 中断。
}

// 带帧头、帧尾的解析器（用于 USART1/USART2）。
// byte：当前字节；head/tail：帧头和帧尾；temp_buf：临时缓冲区。
// index：当前帧索引；state：状态机；pkt/idx：目标接收包数组及索引。
static void parse_with_head_tail(
    uint8_t byte, uint8_t head, uint8_t tail,
    uint8_t *temp_buf, uint16_t *index, FrameState *state,
    Serial_RxPacket *pkt, int idx)
{
    switch (*state)
    {
    case WAIT_FRAME_HEAD:
        if (byte == head)
        {
            temp_buf[0] = byte;
            *index = 1;
            *state = WAIT_FRAME_DATA;
        }
        break;
    case WAIT_FRAME_DATA:
        if (byte == head) // 若连续出现多个帧头，以最后一个为准。
        {
            temp_buf[0] = byte;
            *index = 1;
        }
        else if (byte == tail)
        {
            temp_buf[*index] = byte;
            (*index)++;
            memcpy(pkt[idx].Rx2, temp_buf, *index);
            pkt[idx].number_finall = *index;
            pkt[idx].rx_finall_flag = 1;
            *state = WAIT_FRAME_HEAD;
            *index = 0;
        }
        else
        {
            if (*index < RX_BUFFER_SIZE - 1)
            {
                temp_buf[*index] = byte;
                (*index)++;
            }
            else
            {
                *state = WAIT_FRAME_HEAD;
                *index = 0;
            }
        }
        break;
    }
}

// 无帧头、仅以帧尾结束的解析器（用于 UART4/5/6）。
static void parse_no_head(
    uint8_t byte, uint8_t tail,
    uint8_t *temp_buf, uint16_t *index, FrameState *state,
    Serial_RxPacket *pkt, int idx)
{
    switch (*state)
    {
    case WAIT_FRAME_HEAD:
        temp_buf[0] = byte;
        *index = 1;
        if (byte == tail)
        {
            memcpy(pkt[idx].Rx2, temp_buf, 1);
            pkt[idx].number_finall = 1;
            pkt[idx].rx_finall_flag = 1;
            *state = WAIT_FRAME_HEAD;
            *index = 0;
        }
        else
        {
            *state = WAIT_FRAME_DATA;
        }
        break;
    case WAIT_FRAME_DATA:
        if (*index < RX_BUFFER_SIZE - 1)
        {
            temp_buf[*index] = byte;
            (*index)++;
            if (byte == tail)
            {
                memcpy(pkt[idx].Rx2, temp_buf, *index);
                pkt[idx].number_finall = *index;
                pkt[idx].rx_finall_flag = 1;
                *state = WAIT_FRAME_HEAD;
                *index = 0;
            }
        }
        else
        {
            *state = WAIT_FRAME_HEAD;
            *index = 0;
        }
        break;
    }
}

void UART_IDLE_DmaRingHandler(UART_HandleTypeDef *huart, uint8_t *dma_buf)
{
    uint16_t *last_pos;
    uint16_t pos;
    uint32_t ndtr;

    if (huart->hdmarx == NULL)
    {
        return;
    }

    /* USART1 已改为中断接收，不再走 DMA + IDLE。 */
    if (huart->Instance == USART1)
    {
        return;
    }

    /* USART2 已改为中断收 */
    if (huart->Instance == USART2)
    {
        return;
    }

    if (huart->Instance == USART3)
    {
        last_pos = &uart3_dma_rx_last_pos;
    }
    else if (huart->Instance == UART4)
    {
        last_pos = &uart4_dma_rx_last_pos;
    }
    else
    {
        return;
    }

    ndtr = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (ndtr > RX_BUFFER_SIZE)
    {
        ndtr = RX_BUFFER_SIZE;
    }
    pos = (uint16_t)(RX_BUFFER_SIZE - ndtr);
    if (huart->Instance == USART3)
    {
        layser_dma_rx_pos = pos;
    }

    if (pos == *last_pos)
    {
        return;
    }

    if (pos > *last_pos)
    {
        UART_IDLE_Callback(huart, dma_buf + *last_pos, pos - *last_pos);
    }
    else
    {
        if (*last_pos < RX_BUFFER_SIZE)
        {
            UART_IDLE_Callback(huart, dma_buf + *last_pos, RX_BUFFER_SIZE - *last_pos);
        }
        if (pos > 0U)
        {
            UART_IDLE_Callback(huart, dma_buf, pos);
        }
    }

    *last_pos = pos;
}

// IDLE 中断回调，由 stm32f4xx_it.c 调用。
void UART_IDLE_Callback(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        uint8_t byte = buf[i];

        if (huart->Instance == USART3)
        {
            layser_uart_rx_byte(byte);
        }
        else if (huart->Instance == UART4)
        {
            parse_no_head(byte, 0x6B,
                          uart4_temp_frame_buffer, &uart4_current_frame_index, &uart4_frame_state,
                          RXdata_4, 1);
            if (RXdata_4[1].rx_finall_flag)
            {
                // 将解析缓冲区 Rx2 同步到兼容旧代码的 Rx。
                memcpy(RXdata_4[1].Rx, RXdata_4[1].Rx2, RXdata_4[1].number_finall);
                RXdata_4[1].number = (uint8_t)RXdata_4[1].number_finall;
                zdt_stop_read();
            }
        }
        else if (huart->Instance == USART6)
        {
            parse_no_head(byte, 0x21,
                          uart6_temp_frame_buffer, &uart6_current_frame_index, &uart6_frame_state,
                          RXdata_6, 1);
            if (RXdata_6[1].rx_finall_flag)
            {
                memcpy(RXdata_6[1].Rx, RXdata_6[1].Rx2, RXdata_6[1].number_finall);
                RXdata_6[1].number = (uint8_t)RXdata_6[1].number_finall;
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {

         // 直接回显接收到的字节，不检查帧头和帧尾。
        usart1_poll_send(&uart1_it_rx_byte, 1U);

        if (HAL_UART_Receive_IT(&huart1, &uart1_it_rx_byte, 1U) != HAL_OK)
        {
            (void)HAL_UART_AbortReceive(&huart1);
            (void)HAL_UART_Receive_IT(&huart1, &uart1_it_rx_byte, 1U);
        }
        
    }
    else if (huart->Instance == USART2)
    {
        parse_with_head_tail(uart2_it_rx_byte, 0x6C, 0x7F,
                             uart2_temp_frame_buffer, &uart2_current_frame_index, &uart2_frame_state,
                             RXdata_2, 1);


        if (HAL_UART_Receive_IT(&huart2, &uart2_it_rx_byte, 1U) != HAL_OK)
        {
            (void)HAL_UART_AbortReceive(&huart2);
            (void)HAL_UART_Receive_IT(&huart2, &uart2_it_rx_byte, 1U);
        }
    }
    else if (huart->Instance == UART5)
    {
        LobotServoController_UART_RxCpltCallback(huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    if (huart->Instance == USART1)
    {
        (void)HAL_UART_AbortReceive(huart);
        (void)HAL_UART_Receive_IT(huart, &uart1_it_rx_byte, 1U);
    }
    else if (huart->Instance == USART2)
    {
        (void)HAL_UART_AbortReceive(huart);
        (void)HAL_UART_Receive_IT(huart, &uart2_it_rx_byte, 1U);
    }
    else if (huart->Instance == USART3)
    {
        // layser_restart_uart_rx();  // USART3 currently disabled
    }
    else if (huart->Instance == UART5)
    {
        LobotServoController_UART_ErrorCallback(huart);
    }
}

void parse_uart_frames(void)
{
    // UART_IDLE_DmaRingHandler(&huart3, dma_rx_buf3);  // USART3 currently disabled
    UART_IDLE_DmaRingHandler(&huart4, dma_rx_buf4);
}

void main_loop(void)
{
    parse_uart_frames();
    process_uart_data(); // 实现在 uart_execute.c。
}
