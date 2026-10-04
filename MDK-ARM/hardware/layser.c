#include "layser.h"
#include "usart.h"
#include "system.h"
#include "Emm_V5.h"

#define LAYSER_FRAME_HEAD 0x5CU
#define LAYSER_FRAME_LEN  4U
#define LAYSER_BASE_DISTANCE 83
#define LAYSER_MOTOR1_LIMIT_DISTANCE 27
#define LAYSER_MOTOR1_VEL 280
#define LAYSER_MOTOR1_ACC 236
#define LAYSER_MOTOR1_DIR_DOWN 0
#define LAYSER_MOTOR1_DIR_UP   1
#define LAYSER_MOTOR1_CALIB_VEL 280
#define LAYSER_MOTOR1_CALIB_ACC 206
#define LAYSER_MOTOR1_PULSE_PER_MM 1600


static uint8_t layser_rx_buf[LAYSER_FRAME_LEN];
static uint8_t layser_rx_index = 0;
static uint8_t layser_motor1_adjust_active = 0;
static uint8_t layser_motor1_adjust_dir = LAYSER_MOTOR1_DIR_UP;
static uint16_t layser_motor1_start_distance = 0;
static int32_t layser_motor1_calibrate_target_pos = 0;

static void layser_send_cmd(const uint8_t *cmd)
{
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)cmd, 6U, 100U);
}

void layser_init(void)
{
    static const uint8_t cmd_disable_switch[] = {0x5AU, 0x0AU, 0x02U, 0x05U, 0x00U, 0xEEU};

    layser_send_cmd(cmd_disable_switch);
    layser_start_measure();
}

void layser_start_measure(void)
{
    static const uint8_t cmd_start[] = {0x5AU, 0x0AU, 0x02U, 0x02U, 0x00U, 0xF1U};

    layser_start_cmd_count++;
    layser_send_cmd(cmd_start);
}

void layser_stop_measure(void)
{
    static const uint8_t cmd_stop[] = {0x5AU, 0x0AU, 0x02U, 0x00U, 0x00U, 0xF3U};

    layser_send_cmd(cmd_stop);
}
void layser_uart_rx_byte(uint8_t byte)
{
    uint8_t checksum;

    if (layser_rx_index == 0U)
    {
        if (byte != LAYSER_FRAME_HEAD)
        {
            return;
        }
    }
    layser_rx_buf[layser_rx_index] = byte;
    layser_rx_index++;

    if (layser_rx_index < LAYSER_FRAME_LEN)
    {
        return;
    }

    checksum = (uint8_t)(~(uint8_t)(layser_rx_buf[1] + layser_rx_buf[2]));
    if (checksum == layser_rx_buf[3])
    {
        layser_distance = (uint16_t)(((uint16_t)layser_rx_buf[2] << 8) | layser_rx_buf[1]);
        layser_distance_update_flag = 1U;
        layser_last_update_time = time;
        layser_last_update_ms = HAL_GetTick();
        layser_frame_count++;
    }
    else
    {
        layser_frame_error_count++;
    }

    layser_rx_index = 0U;
}

void layser_parser_reset(void)
{
    layser_rx_index = 0U;
}

void layser_motor1_adjust_reset(void)
{
    layser_motor1_adjust_active = 0U;
    Emm_V5_Stop_Now(1, false);
}

uint8_t layser_motor1_calibrate_to_base(void)
{
    int32_t diff_mm;
    int32_t diff_pulse;

    if (layser_distance_update_flag == 0U)
    {
        return 0U;
    }

    diff_mm = (int32_t)layser_distance - (int32_t)LAYSER_BASE_DISTANCE;
    diff_pulse = -diff_mm * LAYSER_MOTOR1_PULSE_PER_MM;
    layser_motor1_calibrate_target_pos = now_pos[1] + diff_pulse;
    zuobiao_single_custom(1, layser_motor1_calibrate_target_pos,
                          LAYSER_MOTOR1_CALIB_VEL, LAYSER_MOTOR1_CALIB_ACC);
    return 1U;
}

int32_t layser_motor1_get_calibrate_target_pos(void)
{
    return layser_motor1_calibrate_target_pos;
}

LayserMotor1AdjustStatus layser_motor1_adjust_to_base(uint8_t tolerance)
{
    uint16_t distance = layser_distance;
    uint16_t upper_done;
    uint16_t lower_done;
    uint16_t upper_limit;
    uint16_t lower_limit;

    if (tolerance == 0U)
    {
        tolerance = 1U;
    }

    upper_done = (uint16_t)(LAYSER_BASE_DISTANCE + tolerance);
    lower_done = (uint16_t)(LAYSER_BASE_DISTANCE - tolerance);

    if (layser_distance_update_flag == 0U && layser_motor1_adjust_active == 0U)
    {
        return LAYSER_MOTOR1_ADJUST_WAIT_DATA;
    }

    if (layser_motor1_adjust_active != 0U &&
        (layser_last_update_ms == 0U || (HAL_GetTick() - layser_last_update_ms) > 200U))
    {
        layser_motor1_adjust_active = 0U;
        layser_distance_update_flag = 0U;
        Emm_V5_Stop_Now(1, false);
        delay_ms(3U);
        return LAYSER_MOTOR1_ADJUST_WAIT_DATA;
    }

    if (layser_motor1_adjust_active == 0U)
    {
        layser_motor1_start_distance = distance;

        if (distance >= lower_done && distance <= upper_done)
        {
            Emm_V5_Stop_Now(1, false);
            return LAYSER_MOTOR1_ADJUST_DONE;
        }

        if (distance < lower_done)
        {
            layser_motor1_adjust_dir = LAYSER_MOTOR1_DIR_UP;
        }
        else
        {
            layser_motor1_adjust_dir = LAYSER_MOTOR1_DIR_DOWN;
        }

        Emm_V5_Vel_Control(1, layser_motor1_adjust_dir, LAYSER_MOTOR1_VEL, LAYSER_MOTOR1_ACC, false);
        delay_ms(3U);
        layser_motor1_adjust_active = 1U;
    }

    upper_limit = (uint16_t)(layser_motor1_start_distance + LAYSER_MOTOR1_LIMIT_DISTANCE);
    lower_limit = (layser_motor1_start_distance > LAYSER_MOTOR1_LIMIT_DISTANCE) ?
                  (uint16_t)(layser_motor1_start_distance - LAYSER_MOTOR1_LIMIT_DISTANCE) : 0U;

    if (layser_motor1_adjust_dir == LAYSER_MOTOR1_DIR_UP)
    {
        if (distance >= lower_done)
        {
            layser_motor1_adjust_active = 0U;
            Emm_V5_Stop_Now(1, false);
            delay_ms(3U);
            return LAYSER_MOTOR1_ADJUST_DONE;
        }

        if (distance >= upper_limit)
        {
            layser_motor1_adjust_active = 0U;
            Emm_V5_Stop_Now(1, false);
            delay_ms(3U);
            return LAYSER_MOTOR1_ADJUST_LIMITED;
        }
    }
    else
    {
        if (distance <= upper_done)
        {
            layser_motor1_adjust_active = 0U;
            Emm_V5_Stop_Now(1, false);
            delay_ms(3U);
            return LAYSER_MOTOR1_ADJUST_DONE;
        }

        if (distance <= lower_limit)
        {
            layser_motor1_adjust_active = 0U;
            Emm_V5_Stop_Now(1, false);
            delay_ms(3U);
            return LAYSER_MOTOR1_ADJUST_LIMITED;
        }
    }

    return LAYSER_MOTOR1_ADJUST_RUNNING;
}
