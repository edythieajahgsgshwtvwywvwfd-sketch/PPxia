#include "servo.h"
#include "tim.h"
#include "system.h"


#ifndef SERVO_PULSE_MIN_US
#define SERVO_PULSE_MIN_US 500u
#endif
#ifndef SERVO_PULSE_MAX_US
#define SERVO_PULSE_MAX_US 2500u
#endif
#ifndef SITA_START_DEG
#define SITA_START_DEG 45
#endif

#define SERVO_BREAK_CMD_INTERVAL_TICK  1u
#define SERVO_BREAK_SITA_STEP          1
#define SMOOTH_TOTAL_TICKS             1500u
#define SMOOTH_CMD_INTERVAL_TICKS      2u
#define SMOOTH_MAX_SITA_STEP           1
#define SMOOTH_END_FINE_WINDOW         30u
#define SMOOTH_SERVO_LAG_GAIN          0.6f
#define SMOOTH_SERVO_LAG_MAX           3
#define SMOOTH_SERVO_STEP_CAP          1

static uint32_t servo_break_next_cmd_tick = 0;

typedef struct
{
    uint8_t active;
    int16_t start_sita;
    int16_t end_sita;
    int16_t last_cmd_sita;
    int16_t last_servo_sita;
    uint8_t mirror_motor7;
    uint32_t start_tick;
    uint32_t next_cmd_tick;
    uint32_t total_ticks;
} ServoBreakSmoothState;

static ServoBreakSmoothState g_servo_break_smooth = {0};

static uint32_t servo_us_to_ccr(TIM_TypeDef *tim, uint16_t us)
{
    uint32_t arr = (uint32_t)(tim->ARR);
    uint32_t max_ccr = arr + 1u;

    uint32_t ccr = (uint32_t)us;
    if (ccr > max_ccr) {
        ccr = max_ccr;
    }
    return ccr;
}

void servo_init(void)
{
    // 启动 TIM1 的 4 路 PWM。
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PE9  -> TIM1_CH1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // PE11 -> TIM1_CH2
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // PE13 -> TIM1_CH3
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // PE14 -> TIM1_CH4

    // TIM5_CH1/CH2 is used by Motor6 encoder now, so do not start it as servo PWM.


    servo_set_angle_270(SERVO_CH1, 0);
    servo_set_angle(SERVO_CH2, 90);
    servo_set_angle(SERVO_CH3, 90);
    servo_set_angle(SERVO_CH4, 0);

}

void servo_set_pulse_us(uint8_t ch, uint16_t us)
{
    if (us < SERVO_PULSE_MIN_US) us = SERVO_PULSE_MIN_US;
    if (us > SERVO_PULSE_MAX_US) us = SERVO_PULSE_MAX_US;

    switch (ch) {
    case SERVO_CH1:
        TIM1->CCR1 = servo_us_to_ccr(TIM1, us);
        break;
    case SERVO_CH2:
        TIM1->CCR2 = servo_us_to_ccr(TIM1, us);
        break;
    case SERVO_CH3:
        TIM1->CCR3 = servo_us_to_ccr(TIM1, us);
        break;
    case SERVO_CH4:
        TIM1->CCR4 = servo_us_to_ccr(TIM1, us);
        break;
    case SERVO_CH5:
        // TIM5_CH1 is used by the encoder.
        break;
    case SERVO_CH6:
        // TIM5_CH2 is used by the encoder.
        break;
    default:
        break;
    }
}

void servo_set_angle(uint8_t ch, uint16_t angle)
{
    if (angle > 180u) angle = 180u;

    // 0~180 -> 500~2500us
    uint32_t span = (uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US;
    uint16_t us = (uint16_t)(SERVO_PULSE_MIN_US + (span * (uint32_t)angle) / 180u);
    servo_set_pulse_us(ch, us);
}

/**
 * @brief 设置 270 度舵机角度。
 * @param ch 舵机通道（SERVO_CH1~SERVO_CH6）。
 * @param angle 角度，范围 0~270 度。
 * @note 0 度对应 500 us，270 度对应 2500 us，135 度对应 1500 us。
 */
void servo_set_angle_270(uint8_t ch, uint16_t angle)
{
    if (angle > 270u) angle = 270u;

    // 0~270 -> 500~2500us
    uint32_t span = (uint32_t)SERVO_PULSE_MAX_US - (uint32_t)SERVO_PULSE_MIN_US;
    uint16_t us = (uint16_t)(SERVO_PULSE_MIN_US + (span * (uint32_t)angle) / 270u);
    servo_set_pulse_us(ch, us);
}

void servo_formula(int16_t sita_deg)
{
    float a_deg = 156.0f - (float)sita_deg;
    float b_deg = 157.0f - (float)sita_deg;
    float a_rad = a_deg * PI / 180.0f;
    float b_rad = b_deg * PI / 180.0f;

    // 相对位移公式
    float x = -51.2f * (cosf(a_rad) - cosf(b_rad)) / 4.0f;
    float y = 51.2f * (sinf(a_rad) - sinf(b_rad)) / 2.0f - (1.0f / 34.0f);// - ( 2.0f / 34.0f);

    // int32_t dx_pulse = (int32_t)lroundf(x * STEPPER78_PULSE_PER_UNIT);
    // int32_t dy_pulse = (int32_t)lroundf(y * STEPPER78_PULSE_PER_UNIT);

    int32_t target7 = now_pos[7] - (int32_t)(x * 3200.0f);
    int32_t target8 = now_pos[8] + (int32_t)(y * 3200.0f);

    // 先下发电机 7/8 的目标位置，再由状态机等待到位。
    zuobiao_single_custom(7, target7, STEPPER78_VEL, STEPPER78_ACC);
    zuobiao_single_custom(8, target8, STEPPER78_VEL, STEPPER78_ACC);
    servo_set_angle_270(SERVO_CH1, 165 - sita_deg);
}

static void servo_formula_to_targets(int16_t sita_deg, uint8_t mirror_motor7, int32_t *target7, int32_t *target8, int16_t *servo_angle)
{
    float a_deg = 156.0f - (float)sita_deg;
    float b_deg = 157.0f - (float)sita_deg;
    float a_rad = a_deg * PI / 180.0f;
    float b_rad = b_deg * PI / 180.0f;
    float x = -51.2f * (cosf(a_rad) - cosf(b_rad)) / 4.0f;
    float y = 51.2f * (sinf(a_rad) - sinf(b_rad)) / 2.0f - (1.0f / 34.0f);
    int32_t dx_pulse = (int32_t)(x * 3200.0f);
    int32_t dy_pulse = (int32_t)(y * 3200.0f);

    // 镜像时只翻转“水平增量”方向，避免把增量轨迹误当成绝对坐标导致乱动
    *target7 = now_pos[7] + (mirror_motor7 ? dx_pulse : -dx_pulse);
    *target8 = now_pos[8] + dy_pulse;
    *servo_angle = (int16_t)(165 - sita_deg);
}

void servo_break_smooth_start(int16_t servo_start_deg, int16_t servo_end_deg, uint8_t mirror_motor7)
{
    int16_t start_sita = 165 - servo_start_deg;
    int16_t end_sita = 165 - servo_end_deg;

    g_servo_break_smooth.active = 1;
    g_servo_break_smooth.start_sita = start_sita;
    g_servo_break_smooth.end_sita = end_sita;
    g_servo_break_smooth.last_cmd_sita = start_sita;
    g_servo_break_smooth.last_servo_sita = start_sita;
    g_servo_break_smooth.mirror_motor7 = mirror_motor7 ? 1u : 0u;
    g_servo_break_smooth.start_tick = time;
    g_servo_break_smooth.next_cmd_tick = time + SMOOTH_CMD_INTERVAL_TICKS;
    g_servo_break_smooth.total_ticks = SMOOTH_TOTAL_TICKS;

    servo_finish_flag = 0;

    if (g_servo_break_smooth.mirror_motor7)
    {
        int16_t start_angle;
        int32_t start_target7;
        int32_t start_target8;
        servo_formula_to_targets(start_sita, 1, &start_target7, &start_target8, &start_angle);
        zuobiao_single_custom(7, start_target7, STEPPER78_VEL, STEPPER78_ACC);
        zuobiao_single_custom(8, start_target8, STEPPER78_VEL, STEPPER78_ACC);
        servo_set_angle_270(SERVO_CH1, (uint16_t)start_angle);
    }
    else
    {
        servo_formula(start_sita);
    }
}

void servo_break_smooth_update(void)
{
    uint32_t elapsed_ticks;
    float progress;
    float smooth_progress;
    int16_t sita_span;
    int16_t sita_ref;
    int16_t sita_delta;
    int16_t sita_cmd;
    int16_t sita_servo_target;
    int16_t sita_servo_delta;
    int16_t sita_servo_cmd;
    int16_t lag_ref;
    int16_t lag_comp;
    int16_t step_cap = SMOOTH_MAX_SITA_STEP;
    int16_t angle_cmd;
    int32_t target7;
    int32_t target8;
    int16_t remaining_to_end;

    if (g_servo_break_smooth.active == 0)
    {
        return;
    }

    elapsed_ticks = time - g_servo_break_smooth.start_tick;
    if (elapsed_ticks >= g_servo_break_smooth.total_ticks)
    {
        elapsed_ticks = g_servo_break_smooth.total_ticks;
    }

    progress = (float)elapsed_ticks / (float)g_servo_break_smooth.total_ticks;
    // smoothstep: 3t^2 - 2t^3
    smooth_progress = progress * progress * (3.0f - 2.0f * progress);

    sita_span = g_servo_break_smooth.end_sita - g_servo_break_smooth.start_sita;
    sita_ref = g_servo_break_smooth.start_sita + (int16_t)lroundf((float)sita_span * smooth_progress);

    if (progress >= (100.0f - (float)SMOOTH_END_FINE_WINDOW) / 100.0f)
    {
        step_cap = 1;
    }

    if (time < g_servo_break_smooth.next_cmd_tick)
    {
        goto maybe_finish;
    }

    sita_delta = sita_ref - g_servo_break_smooth.last_cmd_sita;
    if (sita_delta > step_cap)
    {
        sita_delta = step_cap;
    }
    else if (sita_delta < -step_cap)
    {
        sita_delta = -step_cap;
    }

    sita_cmd = g_servo_break_smooth.last_cmd_sita + sita_delta;
    if (progress >= 1.0f)
    {
        sita_cmd = g_servo_break_smooth.end_sita;
    }

    if (sita_cmd != g_servo_break_smooth.last_cmd_sita)
    {
        servo_formula_to_targets(sita_cmd, g_servo_break_smooth.mirror_motor7, &target7, &target8, &angle_cmd);
        zuobiao_single_custom(7, target7, STEPPER78_VEL, STEPPER78_ACC);
        zuobiao_single_custom(8, target8, STEPPER78_VEL, STEPPER78_ACC);

        lag_ref = sita_ref - sita_cmd;
        lag_comp = (int16_t)lroundf((float)lag_ref * SMOOTH_SERVO_LAG_GAIN);
        if (lag_comp > SMOOTH_SERVO_LAG_MAX)
        {
            lag_comp = SMOOTH_SERVO_LAG_MAX;
        }
        else if (lag_comp < -SMOOTH_SERVO_LAG_MAX)
        {
            lag_comp = -SMOOTH_SERVO_LAG_MAX;
        }

        sita_servo_target = sita_cmd - lag_comp;
        if (g_servo_break_smooth.start_sita <= g_servo_break_smooth.end_sita)
        {
            if (sita_servo_target < g_servo_break_smooth.start_sita)
            {
                sita_servo_target = g_servo_break_smooth.start_sita;
            }
            else if (sita_servo_target > g_servo_break_smooth.end_sita)
            {
                sita_servo_target = g_servo_break_smooth.end_sita;
            }
        }
        else
        {
            if (sita_servo_target > g_servo_break_smooth.start_sita)
            {
                sita_servo_target = g_servo_break_smooth.start_sita;
            }
            else if (sita_servo_target < g_servo_break_smooth.end_sita)
            {
                sita_servo_target = g_servo_break_smooth.end_sita;
            }
        }

        sita_servo_delta = sita_servo_target - g_servo_break_smooth.last_servo_sita;
        if (sita_servo_delta > SMOOTH_SERVO_STEP_CAP)
        {
            sita_servo_delta = SMOOTH_SERVO_STEP_CAP;
        }
        else if (sita_servo_delta < -SMOOTH_SERVO_STEP_CAP)
        {
            sita_servo_delta = -SMOOTH_SERVO_STEP_CAP;
        }
        sita_servo_cmd = g_servo_break_smooth.last_servo_sita + sita_servo_delta;
        angle_cmd = 165 - sita_servo_cmd;
        servo_set_angle_270(SERVO_CH1, (uint16_t)angle_cmd);

        g_servo_break_smooth.last_servo_sita = sita_servo_cmd;
        g_servo_break_smooth.last_cmd_sita = sita_cmd;
        g_servo_break_smooth.next_cmd_tick = time + SMOOTH_CMD_INTERVAL_TICKS;
    }

maybe_finish:
    remaining_to_end = g_servo_break_smooth.end_sita - g_servo_break_smooth.last_cmd_sita;
    if (remaining_to_end < 0)
    {
        remaining_to_end = -remaining_to_end;
    }

    if (progress >= 1.0f && remaining_to_end == 0)
    {
        if (zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
        {
            servo_set_angle_270(SERVO_CH1, (uint16_t)(165 - g_servo_break_smooth.end_sita));
            g_servo_break_smooth.active = 0;
            servo_finish_flag = 1;
            time_b = time;
        }
    }
}

void servo_break(void)
{
    if (servo_ctrl_flag == 0)
    {
        g_sita_deg = SITA_START_DEG;
        servo_set_angle_270(SERVO_CH1, 120);
        servo_ctrl_flag = 1;
        time_b = time;
    }
    else if (servo_ctrl_flag == 1 && time > time_b + 100)
    {
        if (g_sita_deg <= 79)            //servo: 120 -> 86  step: 45 -> 79
        {
            servo_formula(g_sita_deg);
            servo_ctrl_flag = 2;
        }
        else
        {
            // 本次扫描结束
            servo_ctrl_flag = 0;
            time_b = time;
            servo_finish_flag = 1;
        }
    }
    else if (servo_ctrl_flag == 2)
    {
        // 等待电机 7/8 都到位后，再进入下一个 sita 角度。
        if (zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
        {
            g_sita_deg++;
            servo_ctrl_flag = 1;
            time_b = time;
        }
    }
}

/**
 * @brief 皮皮虾剥壳动作（直接传入舵机角度版）
 * @param servo_start_deg 舵机起始角度 (例如 120)
 * @param servo_end_deg   舵机终止角度 (例如 86)
 * @note 该函数会自动反推 sita，以保持步进电机轨迹连续。
 */

void servo_break_by_servo_angle(int16_t servo_start_deg, int16_t servo_end_deg)
{
    // 根据舵机角度反推 sita: sita = 165 - servo_angle
    int16_t start_sita = 165 - servo_start_deg;
    int16_t end_sita = 165 - servo_end_deg;
    int16_t dir = (start_sita <= end_sita) ? 1 : -1;

    if (servo_ctrl_flag == 0)
    {
        g_sita_deg = start_sita;
        // 舵机直接到达起始位置
        servo_set_angle_270(SERVO_CH1, servo_start_deg); 
        servo_ctrl_flag = 1;
        servo_break_next_cmd_tick = time;
    }
    else if (servo_ctrl_flag == 1)
    {
        // 限制命令下发频率，避免总线过快堆积
        if (time < servo_break_next_cmd_tick) {
            return;
        }

        // 判断是否到达终止 sita
        if ((start_sita < end_sita && g_sita_deg <= end_sita) || 
            (start_sita > end_sita && g_sita_deg >= end_sita))
        {
            servo_formula(g_sita_deg);
            servo_ctrl_flag = 2;
        }
        else
        {
            // 扫描结束
            servo_ctrl_flag = 0;
            time_b = time;
            servo_finish_flag = 1;
        }
    }
    else if (servo_ctrl_flag == 2)
    {
        // 等待电机 7/8 都到位后，再进入下一个 sita 角度。
        if (zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
        {
            // 更新 sita：保持轨迹方向不变，同时减少一次次1度更新的开销
            if (dir > 0) {
                g_sita_deg += SERVO_BREAK_SITA_STEP;
                if (g_sita_deg > end_sita) {
                    g_sita_deg = end_sita;
                }
            } else {
                g_sita_deg -= SERVO_BREAK_SITA_STEP;
                if (g_sita_deg < end_sita) {
                    g_sita_deg = end_sita;
                }
            }
            servo_ctrl_flag = 1;
            servo_break_next_cmd_tick = time + SERVO_BREAK_CMD_INTERVAL_TICK;
        }
    }
}

/**
 * @brief 启动舵机与步进电机的同步运动
 */
void servo_start_sync(uint8_t servo_ch, uint8_t motor_id, int16_t start_angle, int16_t end_angle)
{
    if (motor_id < 1 || motor_id > 8) return;

    servo_sync_channel = servo_ch; // 记录舵机通道
    servo_sync_active = 1;
    servo_sync_start_time = time; 
    servo_sync_total_time = zdt_over_time[motor_id]; 
    
    if (servo_sync_total_time <= 0.001f) {
        servo_sync_total_time = 0.001f; 
    }

    servo_sync_start_angle = start_angle;
    servo_sync_end_angle = end_angle;
}
void servo_start_sync_fast(uint8_t servo_ch, uint8_t motor_id, int16_t start_angle, int16_t end_angle)
{
    servo_start_sync(servo_ch, motor_id, start_angle, end_angle);
    servo_sync_total_time *= 0.5f;
    if (servo_sync_total_time <= 0.001f) {
        servo_sync_total_time = 0.001f;
    }
}

/**
 * @brief 更新同步过程中的舵机角度
 */
void servo_update_sync(void)
{
    if (servo_sync_active == 1 && servo_sync_total_time > 0)
    {
        uint32_t elapsed_ticks = time - servo_sync_start_time;
        float progress = (elapsed_ticks * 0.01f) / servo_sync_total_time;
        
        if (progress >= 1.0f) 
        {
            progress = 1.0f;
            servo_sync_active = 0; 
        }
        
        int16_t current_angle = servo_sync_start_angle - (int16_t)((servo_sync_start_angle - servo_sync_end_angle) * progress);
        
        // 【关键】使用记录的通道下发指令
        if (servo_sync_channel == SERVO_CH1)
        {
            servo_set_angle_270(servo_sync_channel, current_angle);
        }
        else
        {
            servo_set_angle(servo_sync_channel, current_angle);
        }
    }
}
