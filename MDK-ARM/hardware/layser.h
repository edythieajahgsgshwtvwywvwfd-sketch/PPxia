#ifndef __LAYSER_H
#define __LAYSER_H

#include "stdint.h"

typedef enum
{
    LAYSER_MOTOR1_ADJUST_WAIT_DATA = 0,
    LAYSER_MOTOR1_ADJUST_RUNNING,
    LAYSER_MOTOR1_ADJUST_DONE,
    LAYSER_MOTOR1_ADJUST_LIMITED
} LayserMotor1AdjustStatus;


void layser_init(void);
void layser_start_measure(void);
void layser_stop_measure(void);
void layser_restart_uart_rx(void);
void layser_parser_reset(void);
void layser_uart_rx_byte(uint8_t byte);
void layser_motor1_adjust_reset(void);
uint8_t layser_motor1_calibrate_to_base(void);
int32_t layser_motor1_get_calibrate_target_pos(void);
LayserMotor1AdjustStatus layser_motor1_adjust_to_base(uint8_t tolerance);

#endif
