#include "LobotServoController_F4.h"
#include "uart_execute.h"
#include "system.h"

#include <stdarg.h>
#include <string.h>

#define GET_LOW_BYTE(A) ((uint8_t)(A))
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

#ifndef LOBOT_RX_BUF_SIZE
#define LOBOT_RX_BUF_SIZE 260U
#endif

static UART_HandleTypeDef *s_huart;
static uint8_t s_rx_byte;
static uint8_t s_rx_frame[LOBOT_RX_BUF_SIZE];
static uint16_t s_rx_idx;
static uint16_t s_rx_remaining;
static uint8_t s_rx_state;
static volatile uint16_t s_battery_mv;
static LobotFrameCallback s_frame_cb;

enum
{
    RX_WAIT_H1 = 0,
    RX_WAIT_H2 = 1,
    RX_WAIT_LEN = 2,
    RX_WAIT_PAYLOAD = 3,
};

__weak void LobotServoController_OnFrame(const uint8_t *frame, uint16_t len)
{
    (void)frame;
    (void)len;
}

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 将 0~180 度线性映射到控制板位置值 500~2500。 */
static uint16_t angle_to_position(float angle_deg)
{
    float a = clamp_f(angle_deg, 0.0f, 180.0f);
    float pos = 500.0f + (a * 2000.0f / 180.0f);
    return clamp_u16((uint16_t)(pos + 0.5f), 500U, 2500U);
}

/* 将 0~270 度线性映射到控制板位置值 500~2500。 */
static uint16_t angle270_to_position(float angle_deg)
{
    float a = clamp_f(angle_deg, 0.0f, 270.0f);
    float pos = 500.0f + (a * 2000.0f / 270.0f);
    return clamp_u16((uint16_t)(pos + 0.5f), 500U, 2500U);
}

static void usart1_send_text(const char *text)
{
    if (text == NULL)
    {
        return;
    }
    usart1_poll_send((const uint8_t *)text, (uint16_t)strlen(text));
}

static void usart1_send_hex_byte(uint8_t b)
{
    char hex[2];
    static const char table[] = "0123456789ABCDEF";
    hex[0] = table[(b >> 4) & 0x0F];
    hex[1] = table[b & 0x0F];
    usart1_poll_send((const uint8_t *)hex, 2U);
}

static void debug_print_tx_frame_to_usart1(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    usart1_send_text("[LOBOT TX] ");
    for (i = 0; i < len; i++)
    {
        usart1_send_hex_byte(buf[i]);
        if (i + 1U < len)
        {
            usart1_send_text(" ");
        }
    }
    usart1_send_text("\r\n");
}

/* 发送协议帧：0x55 0x55 Length Cmd Params... */
static void lobot_send_cmd(uint8_t cmd, const uint8_t *params, uint8_t params_len)
{
    uint8_t frame[2U + 2U + 255U];
    uint16_t total_len;

    if (s_huart == NULL)
    {
        return;
    }

    frame[0] = FRAME_HEADER;
    frame[1] = FRAME_HEADER;
    frame[2] = (uint8_t)(params_len + 2U);
    frame[3] = cmd;
    if (params_len > 0U && params != NULL)
    {
        memcpy(&frame[4], params, params_len);
    }
    total_len = (uint16_t)(2U + frame[2]);

    if (HAL_UART_Transmit(s_huart, frame, total_len, 100) != HAL_OK)
    {
        usart1_send_text("[LOBOT TX] FAIL\r\n");
        return;
    }
    debug_print_tx_frame_to_usart1(frame, total_len);
}

void LobotServoController_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_state = RX_WAIT_H1;
    s_rx_idx = 0;
    s_rx_remaining = 0;
    s_battery_mv = 0;
    s_frame_cb = NULL;

    if (s_huart != NULL)
    {
        (void)HAL_UART_AbortReceive(s_huart);
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
    }
}

void LobotServoController_SetFrameCallback(LobotFrameCallback cb)
{
    s_frame_cb = cb;
}

uint16_t LobotServoController_GetBatteryMv(void)
{
    return (uint16_t)s_battery_mv;
}

static void rx_reset(void)
{
    s_rx_state = RX_WAIT_H1;
    s_rx_idx = 0;
    s_rx_remaining = 0;
}

static void rx_push_byte(uint8_t b)
{
    switch (s_rx_state)
    {
    case RX_WAIT_H1:
        if (b == FRAME_HEADER)
        {
            s_rx_frame[0] = b;
            s_rx_state = RX_WAIT_H2;
        }
        break;
    case RX_WAIT_H2:
        if (b == FRAME_HEADER)
        {
            s_rx_frame[1] = b;
            s_rx_state = RX_WAIT_LEN;
        }
        else
        {
            rx_reset();
        }
        break;
    case RX_WAIT_LEN:
    {
        uint8_t len = b;
        if (len < 2U || (uint16_t)(2U + len) > (uint16_t)LOBOT_RX_BUF_SIZE)
        {
            rx_reset();
            break;
        }
        s_rx_frame[2] = len;
        s_rx_idx = 3U;
        s_rx_remaining = (uint16_t)(len - 1U);
        s_rx_state = RX_WAIT_PAYLOAD;
        break;
    }
    case RX_WAIT_PAYLOAD:
        s_rx_frame[s_rx_idx++] = b;
        if (s_rx_remaining > 0U)
        {
            s_rx_remaining--;
        }
        if (s_rx_remaining == 0U)
        {
            uint16_t frame_len = (uint16_t)(2U + s_rx_frame[2]);
            uint8_t cmd = s_rx_frame[3];
            if (cmd == CMD_GET_BATTERY_VOLTAGE && frame_len >= 6U)
            {
                s_battery_mv = (uint16_t)((uint16_t)s_rx_frame[5] << 8) | (uint16_t)s_rx_frame[4];
            }
            if (s_frame_cb != NULL)
            {
                s_frame_cb(s_rx_frame, frame_len);
            }
            else
            {
                LobotServoController_OnFrame(s_rx_frame, frame_len);
            }
            rx_reset();
        }
        break;
    default:
        rx_reset();
        break;
    }
}

void LobotServoController_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart == NULL || huart != s_huart)
    {
        return;
    }
    rx_push_byte(s_rx_byte);
    if (HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U) != HAL_OK)
    {
        (void)HAL_UART_AbortReceive(s_huart);
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
    }
}

void LobotServoController_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart == NULL || huart != s_huart)
    {
        return;
    }
    (void)HAL_UART_AbortReceive(s_huart);
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
    rx_reset();
}

void LobotServoController_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
    /* 当前使用阻塞发送，不需要在 TX 完成回调中处理状态。 */
}

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
    uint8_t p[6];
    if (servoID > 31U || Time == 0U)
    {
        return;
    }
    Position = clamp_u16(Position, 500U, 2500U);
    p[0] = 1U;
    p[1] = GET_LOW_BYTE(Time);
    p[2] = GET_HIGH_BYTE(Time);
    p[3] = servoID;
    p[4] = GET_LOW_BYTE(Position);
    p[5] = GET_HIGH_BYTE(Position);
    lobot_send_cmd(CMD_SERVO_MOVE, p, (uint8_t)sizeof(p));
}

void moveServoAngle(uint8_t servoID, float angle_deg, uint16_t Time)
{
    moveServo(servoID, angle_to_position(angle_deg), Time);
    delay_ms(3);
}

void moveServoAngle270(uint8_t servoID, float angle_deg, uint16_t Time)
{
    moveServo(servoID, angle270_to_position(angle_deg), Time);
}



void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
    uint8_t p[1 + 2 + 32 * 3];
    uint16_t idx = 3U;
    uint8_t i;
    if (Num < 1U || Num > 32U || Time == 0U)
    {
        return;
    }
    p[0] = Num;
    p[1] = GET_LOW_BYTE(Time);
    p[2] = GET_HIGH_BYTE(Time);
    for (i = 0; i < Num; i++)
    {
        uint16_t pos = clamp_u16(servos[i].Position, 500U, 2500U);
        p[idx++] = servos[i].ID;
        p[idx++] = GET_LOW_BYTE(pos);
        p[idx++] = GET_HIGH_BYTE(pos);
    }
    lobot_send_cmd(CMD_SERVO_MOVE, p, (uint8_t)(1U + 2U + Num * 3U));
}

void moveServos(uint8_t Num, uint32_t Time, ...)
{
    uint8_t p[1 + 2 + 32 * 3];
    uint16_t idx = 3U;
    uint8_t i;
    uint16_t temp;
    va_list arg_ptr;
    if (Num < 1U || Num > 32U)
    {
        return;
    }
    p[0] = Num;
    p[1] = GET_LOW_BYTE(Time);
    p[2] = GET_HIGH_BYTE(Time);
    va_start(arg_ptr, Time);
    for (i = 0; i < Num; i++)
    {
        temp = (uint16_t)va_arg(arg_ptr, int);
        p[idx++] = GET_LOW_BYTE(temp);
        temp = clamp_u16((uint16_t)va_arg(arg_ptr, int), 500U, 2500U);
        p[idx++] = GET_LOW_BYTE(temp);
        p[idx++] = GET_HIGH_BYTE(temp);
    }
    va_end(arg_ptr);
    lobot_send_cmd(CMD_SERVO_MOVE, p, (uint8_t)(1U + 2U + Num * 3U));
}

void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
    uint8_t p[3];
    p[0] = numOfAction;
    p[1] = GET_LOW_BYTE(Times);
    p[2] = GET_HIGH_BYTE(Times);
    lobot_send_cmd(CMD_ACTION_GROUP_RUN, p, (uint8_t)sizeof(p));
}

void stopActionGroup(void)
{
    lobot_send_cmd(CMD_ACTION_GROUP_STOP, NULL, 0U);
}

void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
    uint8_t p[3];
    p[0] = numOfAction;
    p[1] = GET_LOW_BYTE(Speed);
    p[2] = GET_HIGH_BYTE(Speed);
    lobot_send_cmd(CMD_ACTION_GROUP_SPEED, p, (uint8_t)sizeof(p));
}

void setAllActionGroupSpeed(uint16_t Speed)
{
    setActionGroupSpeed(0xFFU, Speed);
}

void getBatteryVoltage(void)
{
    lobot_send_cmd(CMD_GET_BATTERY_VOLTAGE, NULL, 0U);
}
