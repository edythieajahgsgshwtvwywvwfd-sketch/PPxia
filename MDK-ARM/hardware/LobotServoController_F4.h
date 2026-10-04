#ifndef LOBOT_SERVO_CONTROLLER_F4_H
#define LOBOT_SERVO_CONTROLLER_F4_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * LSC 16 路 PWM 舵机控制板（Hiwonder/幻尔）UART 协议（V1.1）：
 * 帧格式：0x55 0x55 Length Cmd Params...
 * - Length = 参数个数 N + 2（Length 本身 1 字节 + Cmd 1 字节 + N 个参数）
 * - 整帧字节数 = 2(帧头) + Length
 *
 * 本驱动：
 * - 发送：使用 HAL_UART_Transmit 阻塞发送（逻辑最直接，便于链路调试）
 * - 接收：使用 HAL_UART_Receive_IT 中断逐字节收包，不使用 DMA
 * - 接收端按帧头/长度解析完整帧，可通过回调上报
 */

#define FRAME_HEADER 0x55U

#define CMD_SERVO_MOVE 0x03U
#define CMD_ACTION_GROUP_RUN 0x06U
#define CMD_ACTION_GROUP_STOP 0x07U
#define CMD_ACTION_GROUP_SPEED 0x0BU
#define CMD_ACTION_GROUP_COMPLETE 0x08U
#define CMD_GET_BATTERY_VOLTAGE 0x0FU

typedef struct
{
    uint8_t ID;
    uint16_t Position;
} LobotServo;

typedef void (*LobotFrameCallback)(const uint8_t *frame, uint16_t len);

void LobotServoController_Init(UART_HandleTypeDef *huart);
void LobotServoController_SetFrameCallback(LobotFrameCallback cb);
uint16_t LobotServoController_GetBatteryMv(void);

/**
 * 控制单个舵机转动（位置单位：500~2500，通常对应 0~180°，具体与舵机型号/安装有关）
 * @param servoID: 0~31
 * @param Position: 500~2500（LSC 控制板位置值）
 * @param Time: 运动时间（ms），必须 >0
 */
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);

/**
 * 以“角度”方式控制单个舵机（更直观）：
 * - angle_deg: 0~180（超出会截断）
 * - 内部线性映射为 Position：500~2500
 */
void moveServoAngle(uint8_t servoID, float angle_deg, uint16_t Time);
void moveServoAngle270(uint8_t servoID, float angle_deg, uint16_t Time);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint32_t Time, ...);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);
void getBatteryVoltage(void);

/* 由工程的 HAL 回调转发调用（UART5 使用） */
void LobotServoController_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void LobotServoController_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void LobotServoController_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* 可选：弱回调，未注册函数指针时调用 */
void LobotServoController_OnFrame(const uint8_t *frame, uint16_t len);

#endif

