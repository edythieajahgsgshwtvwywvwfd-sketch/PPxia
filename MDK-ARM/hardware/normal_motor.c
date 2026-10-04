#include "normal_motor.h"
#include "system.h"

#include "stm32f4xx_hal.h" // 示例，具体请根据您的芯片型号选择
#include <math.h>

#define PWM_MAX 100    // 最大速度指令；定时器 ARR 为 102。
#define PWM_MIN -100   // 最小速度指令。

#define MOTOR6_ENCODER_PPR             7L
#define MOTOR6_ENCODER_MULTIPLY        4L
#define MOTOR6_REDUCTION_RATIO         1030L
#define MOTOR6_COUNTS_PER_REV          (MOTOR6_ENCODER_PPR * MOTOR6_ENCODER_MULTIPLY * MOTOR6_REDUCTION_RATIO)
#define MOTOR6_POS_DEADBAND_COUNT      30L
#define MOTOR6_KP                      6L
#define MOTOR6_MAX_PWM                 55
#define MOTOR6_MIN_RUN_PWM             18
#define MOTOR6_ENCODER_DIR             -1L

static int32_t motor6_position_count = 0;
static int32_t motor6_target_count = 0;
static uint8_t motor6_position_active = 0;
static uint8_t motor6_position_done = 0;

static int16_t motor6_limit_pwm(int32_t pwm)
{
    if (pwm > MOTOR6_MAX_PWM) {
        pwm = MOTOR6_MAX_PWM;
    } else if (pwm < -MOTOR6_MAX_PWM) {
        pwm = -MOTOR6_MAX_PWM;
    }

    if (pwm > 0 && pwm < MOTOR6_MIN_RUN_PWM) {
        pwm = MOTOR6_MIN_RUN_PWM;
    } else if (pwm < 0 && pwm > -MOTOR6_MIN_RUN_PWM) {
        pwm = -MOTOR6_MIN_RUN_PWM;
    }

    return (int16_t)pwm;
}

/**
 * @brief 初始化直流电机 PWM。
 * @note  启动 8 路电机对应的全部 PWM 通道，需在定时器初始化后调用。
 */
void normal_motor_init(void)
{
    // 启动 PWM 通道 - 确保启动 motor_set_speed() 函数中使用的所有通道
    // TIM3 用于 Motor1 (CH1/CH2) + Motor2 (CH3/CH4)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);     // PA6  - Motor 1 正转/反转控制
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);     // PA7  - Motor 1 反转/正转控制
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);     // PB0  - Motor 2 正转/反转控制
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);     // PB1  - Motor 2 反转/正转控制

    // TIM4 用于 Motor3 (CH1/CH2) + Motor4 (CH3/CH4)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);     // PD12 - Motor 3 正转/反转控制
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);     // PD13 - Motor 3 反转/正转控制
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);     // PD14 - Motor 4 正转/反转控制
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);     // PD15 - Motor 4 反转/正转控制

    // TIM2 用于 Motor5 (CH3/CH4)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);     // PA2  - Motor 5 正转/反转控制
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);     // PA3  - Motor 5 反转/正转控制

    // TIM12 用于 Motor6 (CH1/CH2)
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);    // PB14 - Motor 6 正转/反转控制
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);    // PB15 - Motor 6 反转/正转控制

    // TIM8 用于 Motor7 (CH1/CH2) + Motor8 (CH3/CH4)
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);     // PC6  - Motor 7 正转/反转控制（特殊驱动）
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);     // PC7  - Motor 7 反转/正转控制（特殊驱动）
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);     // PC8  - Motor 8 正转/反转控制（特殊驱动）
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);     // PC9  - Motor 8 反转/正转控制（特殊驱动）

    Normal_motor_Speed_7 = 0;
    Normal_motor_Speed_8 = 0;
    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;
    TIM8->CCR3 = 0;
    TIM8->CCR4 = 0;    
}

void Motor6_Turn180_Start(void)
{
    Motor6_TurnAngle_Start(180);
}

void Motor6_TurnAngle_Start(int32_t angle_deg)
{
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    motor6_position_count = 0;
    motor6_target_count = (MOTOR6_COUNTS_PER_REV * angle_deg) / 360L;
    motor6_position_active = 1;
    motor6_position_done = 0;
    Motor6_encoder_Speed = 0;
    Normal_motor_Speed_6 = 0;
}

void Motor6_PositionControl_10ms(void)
{
    int32_t encoder_delta = (int32_t)Read_Encoder(5) * MOTOR6_ENCODER_DIR;
    int32_t error;
    int16_t pwm;

    motor6_position_count += encoder_delta;
    Motor6_encoder_Speed = (int16_t)encoder_delta;

    if (motor6_position_active == 0) {
        return;
    }

    error = motor6_target_count - motor6_position_count;
    if (myabs(error) <= MOTOR6_POS_DEADBAND_COUNT) {
        motor6_position_active = 0;
        motor6_position_done = 1;
        Normal_motor_Speed_6 = 0;
        motor_set_speed(6, 0);
        return;
    }

    pwm = motor6_limit_pwm(error * MOTOR6_KP);
    Normal_motor_Speed_6 = pwm;
    motor_set_speed(6, Normal_motor_Speed_6);
}

int32_t Motor6_GetPositionCount(void)
{
    return motor6_position_count;
}

int16_t Motor6_GetAngleDeg(void)
{
    if (MOTOR6_COUNTS_PER_REV == 0) {
        return 0;
    }

    return (int16_t)((motor6_position_count * 360L) / MOTOR6_COUNTS_PER_REV);
}

uint8_t Motor6_IsPositionDone(void)
{
    return motor6_position_done;
}

/**
 * @brief 设置单个直流电机速度
 * @param motor_number 电机编号，范围 1~8。
 * @param speed        速度指令，范围 [-100, 100]。
 * @note  编号与硬件映射：
 *        1->TIM3(CH1/CH2)  (PA6/PA7)
 *        2->TIM3(CH3/CH4)  (PB0/PB1)
 *        3->TIM4(CH1/CH2)  (PD12/PD13)
 *        4->TIM4(CH3/CH4)  (PD14/PD15)
 *        5->TIM2(CH3/CH4)  (PA2/PA3)
 *        6->TIM12(CH1/CH2) (PB14/PB15)
 *        7->TIM8(CH1/CH2)  (PC6/PC7, 特殊驱动)
 *        8->TIM8(CH3/CH4)  (PC8/PC9, 特殊驱动)
 */
void motor_set_speed(uint8_t motor_number, int16_t speed)
{
    uint16_t abs_speed;
    uint32_t reverse_pwm;

    if (speed > PWM_MAX) speed = PWM_MAX;
    else if (speed < PWM_MIN) speed = PWM_MIN;

    abs_speed = (uint16_t)((speed >= 0) ? speed : -speed);
    reverse_pwm = (uint32_t)(PWM_MAX - abs_speed);

    switch (motor_number)
    {
        case 1:
            // Motor1: TIM3 CH1/CH2 (PA6/PA7)
            if (speed == 0) {
                // 电机 1 停止时强制关闭两路 PWM，避免上电误动作。
                TIM3->CCR1 = 0;            // PA6
                TIM3->CCR2 = 0;            // PA7
            } else if (speed > 0) {
                TIM3->CCR1 = PWM_MAX;      // PA6
                TIM3->CCR2 = reverse_pwm;  // PA7
            } else {
                TIM3->CCR1 = reverse_pwm;  // PA6
                TIM3->CCR2 = PWM_MAX;      // PA7
            }
            break;
        case 2:
            // Motor2: TIM3 CH3/CH4 (PB0/PB1)
            if (speed >= 0) {
                TIM3->CCR3 = PWM_MAX;      // PB0
                TIM3->CCR4 = reverse_pwm;  // PB1
            } else {
                TIM3->CCR3 = reverse_pwm;  // PB0
                TIM3->CCR4 = PWM_MAX;      // PB1
            }
            break;
        case 3:
            // Motor3: TIM4 CH1/CH2 (PD12/PD13)
            if (speed >= 0) {
                TIM4->CCR1 = PWM_MAX;      // PD12
                TIM4->CCR2 = reverse_pwm;  // PD13
            } else {
                TIM4->CCR1 = reverse_pwm;  // PD12
                TIM4->CCR2 = PWM_MAX;      // PD13
            }
            break;
        case 4:
            // Motor4: TIM4 CH3/CH4 (PD14/PD15)
            if (speed >= 0) {
                TIM4->CCR3 = PWM_MAX;      // PD14
                TIM4->CCR4 = reverse_pwm;  // PD15
            } else {
                TIM4->CCR3 = reverse_pwm;  // PD14
                TIM4->CCR4 = PWM_MAX;      // PD15
            }
            break;
        case 5:
            // Motor5: TIM2 CH3/CH4 (PA2/PA3)
            if (speed >= 0) {
                TIM2->CCR3 = PWM_MAX;      // PA2
                TIM2->CCR4 = reverse_pwm;  // PA3
            } else {
                TIM2->CCR3 = reverse_pwm;  // PA2
                TIM2->CCR4 = PWM_MAX;      // PA3
            }
            break;
        case 6:
            // Motor6: TIM12 CH1/CH2 (PB14/PB15)
            if (speed >= 0) {
                TIM12->CCR1 = PWM_MAX;      // PB14
                TIM12->CCR2 = reverse_pwm;  // PB15
            } else {
                TIM12->CCR1 = reverse_pwm;  // PB14
                TIM12->CCR2 = PWM_MAX;      // PB15
            }
            break;
        case 7:
            // Motor7：特殊驱动模式，一个通道输出 PWM，另一个通道固定为 0。
            // PC6 -> TIM8_CH1, PC7 -> TIM8_CH2
            if (speed == 0) {
                TIM8->CCR1 = 0;
                TIM8->CCR2 = 0;
            } else if (speed > 0) {
                TIM8->CCR1 = (uint32_t)speed; // PC6 输出PWM
                TIM8->CCR2 = 0;               // PC7 关闭
            } else {
                TIM8->CCR1 = 0;               // PC6 关闭
                TIM8->CCR2 = (uint32_t)(-speed); // PC7 输出PWM
            }
            break;
        case 8:
            // Motor8：特殊驱动模式，一个通道输出 PWM，另一个通道固定为 0。
            // PC8 -> TIM8_CH3, PC9 -> TIM8_CH4
            if (speed == 0) {
                TIM8->CCR3 = 0;
                TIM8->CCR4 = 0;
            } else if (speed > 0) {
                TIM8->CCR3 = (uint32_t)speed; // PC8 输出PWM
                TIM8->CCR4 = 0;               // PC9 关闭
            } else {
                TIM8->CCR3 = 0;               // PC8 关闭
                TIM8->CCR4 = (uint32_t)(-speed); // PC9 输出PWM
            }
            break;
        default:
            break;
    }
}

void Motor_Set(uint8_t motor_number, int16_t speed)
{
    if(speed >= 0)
    {
        if(speed>100) speed=100;
        switch (motor_number)
        {
        case 7:
            TIM8->CCR1 = speed;
            TIM8->CCR2 = 0;
            break;
        
        case 8:
            TIM8->CCR3 = speed;
            TIM8->CCR4 = 0;
            break;
        }
    }

    else
    {
        if(speed < -100) speed=-100;
        switch (motor_number)
        {
        case 7:
            TIM8->CCR1 = 0;
            TIM8->CCR2 = -speed;
            break;
        
        case 8:
            TIM8->CCR3 = 0;
            TIM8->CCR4 = -speed;
            break;
        }
    }
}
/**
 * @brief 同时设置 8 个直流电机速度
 * @param speed_1 电机 1 速度，范围 [-100, 100]。
 * @param speed_2 电机 2 速度，范围 [-100, 100]。
 * @param speed_3 电机 3 速度，范围 [-100, 100]。
 * @param speed_4 电机 4 速度，范围 [-100, 100]。
 * @param speed_5 电机 5 速度，范围 [-100, 100]。
 * @param speed_6 电机 6 速度，范围 [-100, 100]。
 * @param speed_7 电机 7 速度，范围 [-100, 100]。
 * @param speed_8 电机 8 速度，范围 [-100, 100]。
 */
void allmotor(int speed_1, int speed_2, int speed_3, int speed_4, int speed_5, int speed_6, int speed_7, int speed_8)
{
    motor_set_speed(1, (int16_t)speed_1);
    motor_set_speed(2, (int16_t)speed_2);
    motor_set_speed(3, (int16_t)speed_3);
    motor_set_speed(4, (int16_t)speed_4);
    motor_set_speed(5, (int16_t)speed_5);
    motor_set_speed(6, (int16_t)speed_6);
    Motor_Set(7, (int16_t)speed_7);
    Motor_Set(8, (int16_t)speed_8);
}

/**
 * @brief 速度规划算法，实现加速、匀速、减速梯形曲线。
 * @param elapsed 当前已用时间（time 单位）。
 * @param total_duration 总时间（time 单位）。
 * @param max_speed 最大速度
 * @param accel_time 加速时间（time 单位）。
 * @param decel_time 减速时间（time 单位）。
 * @return 当前时刻的目标速度
 */
float trapezoidal_speed_profile(int32_t elapsed, int32_t total_duration, 
                               float max_speed, int32_t accel_time, int32_t decel_time)
{
    if (elapsed < 0) return 0.0f;
    
    if (elapsed < accel_time) {
        // 加速阶段。
        return (float)elapsed / accel_time * max_speed;
    }
    else if (elapsed <= total_duration - decel_time) {
        // 匀速阶段。
        return max_speed;
    }
    else if (elapsed <= total_duration) {
        // 减速阶段。
        int32_t decel_elapsed = elapsed - (total_duration - decel_time);
        return max_speed * (1.0f - (float)decel_elapsed / decel_time);
    }
    else {
        // 超时后停止。
        return 0.0f;
    }
}

int motor78_pause_decel_tick(void)
{
    int32_t elapsed_10ms;
    float mag7;
    float mag8;
    float out7;
    float out8;
    const int32_t T = (int32_t)MOTOR78_PAUSE_DECEL_TICKS;

    if (!g_motor78_pause_decel_active)
    {
        return 0;
    }

    elapsed_10ms = (int32_t)((HAL_GetTick() - g_motor78_pause_decel_start_ms) / 10U);
    if (elapsed_10ms < 0)
    {
        elapsed_10ms = 0;
    }
    if (elapsed_10ms > T)
    {
        elapsed_10ms = T;
    }

    mag7 = fabsf((float)g_motor78_pause_init7);
    mag8 = fabsf((float)g_motor78_pause_init8);
    out7 = trapezoidal_speed_profile(elapsed_10ms, T, mag7, 0, T);
    out8 = trapezoidal_speed_profile(elapsed_10ms, T, mag8, 0, T);

    Normal_motor_Speed_7 = (g_motor78_pause_init7 >= 0) ? (int)out7 : -(int)out7;
    Normal_motor_Speed_8 = (g_motor78_pause_init8 >= 0) ? (int)out8 : -(int)out8;

    if (elapsed_10ms >= T)
    {
        Normal_motor_Speed_7 = 0;
        Normal_motor_Speed_8 = 0;
        g_motor78_pause_decel_active = 0;
    }

    allmotor(Normal_motor_Speed_1, Normal_motor_Speed_2,
             Normal_motor_Speed_3, Normal_motor_Speed_4, Normal_motor_Speed_5, Normal_motor_Speed_6,
             Normal_motor_Speed_7, Normal_motor_Speed_8);
    return 1;
}

