// #include "bsp_motor.h"
// #include "system.h"
// //脉冲驱动步进电机，串口控制的话在EMMV5文件
// #define IN1_TIM &htim2
// #define IN1_CHANNEL TIM_CHANNEL_1
// #define IN2_TIM &htim3
// #define IN2_CHANNEL TIM_CHANNEL_1
// #define IN3_TIM &htim4
// #define IN3_CHANNEL TIM_CHANNEL_2
// #define IN4_TIM &htim1
// #define IN4_CHANNEL TIM_CHANNEL_4
// #define MOTOR_MAX_SPEED 100
// #define MAX_SPEED 100 // 最大速度

// int dir_flag[4] = {0, 0, 0, 0}; // 方向标志位数组，0表示正转，1表示反转
// void Motor_Init(void)
// {
//     HAL_TIM_PWM_Start(IN1_TIM, IN1_CHANNEL); // 启动PWM
//     HAL_TIM_PWM_Start(IN2_TIM, IN2_CHANNEL); // 启动PWM
//     HAL_TIM_PWM_Start(IN3_TIM, IN3_CHANNEL); // 启动PWM
//     HAL_TIM_PWM_Start(IN4_TIM, IN4_CHANNEL); // 启动PWM
// }

// void Motor1_enable(void)
// {
//     MOTOR1_EN(ON);
// }
// void Motor2_enable(void)
// {
//     MOTOR2_EN(ON);
// }
// void Motor3_enable(void)
// {
//     MOTOR3_EN(ON);
// }
// void Motor4_enable(void)
// {
//     MOTOR4_EN(ON);
// }
// void motor_all_enable(void)
// {
//     Motor1_enable();
//     Motor2_enable();
//     Motor3_enable();
//     Motor4_enable();
// }
// void Motor1_dir(uint8_t dir)
// {
//     if (dir == 1)
//         MOTOR1_DIR(CW);
//     else if (dir == 0)
//         MOTOR1_DIR(CCW);
// }

// void Motor2_dir(uint8_t dir)
// {
//     if (dir == 1)
//         MOTOR2_DIR(CW);
//     else if (dir == 0)
//         MOTOR2_DIR(CCW);
// }

// void Motor3_dir(uint8_t dir)
// {
//     if (dir == 1)
//         MOTOR3_DIR(CW);
//     else if (dir == 0)
//         MOTOR3_DIR(CCW);
// }

// void Motor4_dir(uint8_t dir)
// {
//     if (dir == 1)
//         MOTOR4_DIR(CW);
//     else if (dir == 0)
//         MOTOR4_DIR(CCW);
// }

// void dir_control(void)
// {
//     if (dir_flag[0] == 0)
//     {
//         //    Motor1_dir(1);
//         Motor1_dir(1);
//     }
//     else if (dir_flag[0] == 1)
//     {
//         Motor1_dir(0);
//     }

//     if (dir_flag[1] == 0)
//     {
//         Motor2_dir(0);
//     }
//     else if (dir_flag[1] == 1)
//     {
//         Motor2_dir(1);
//     }

//     if (dir_flag[2] == 0)
//     {
//         Motor3_dir(1);
//     }
//     else if (dir_flag[2] == 1)
//     {
//         Motor3_dir(0);
//     }

//     if (dir_flag[3] == 0)
//     {
//         Motor4_dir(0);
//     }
//     else if (dir_flag[3] == 1)
//     {
//         Motor4_dir(1);
//     }
// }

// // 内部辅助函数，用于通用速度计算
// static void motor_speed_calculate_internal(TIM_TypeDef *tim, uint32_t channel, float speed)
// {
//     float pulse_val = 0.0f;
//     const float max_speed = (float)MOTOR_MAX_SPEED;
//     const float half_max_speed = max_speed * 0.5f;

//     if (fabsf(speed) == 0)
//     {
//         pulse_val = 0;
//     }
//     else
//     {
//         if (speed > half_max_speed && speed < max_speed)
//         {
//             pulse_val = 312.5f * 1.28f * (1.0f - speed / max_speed);
//         }
//         else if (speed > 0.0f && speed <= half_max_speed)
//         {
//             pulse_val = 312.5f * 0.32f * max_speed / speed;
//         }
//         else if (speed < 0.0f && speed >= -half_max_speed)
//         {
//             pulse_val = 312.5f * 0.32f * max_speed / fabsf(speed);
//         }
//         else if (speed < -half_max_speed && speed > -max_speed)
//         {
//             pulse_val = 312.5f * 1.28f * (1.0f - fabsf(speed) / max_speed);
//         }
//         else if (speed >= max_speed || speed <= -max_speed)
//         {
//             pulse_val = 0.32f;
//         }
//     }

//     if (pulse_val < 1.0f)
//         pulse_val = 1.0f;
//     if (pulse_val > 65535.0f)
//         pulse_val = 65535.0f;

//     uint32_t arr_val = (uint32_t)pulse_val;
//     uint32_t ccr_val = (uint32_t)(arr_val / 2);

//     // 根据通道设置相应的寄存器
//     tim->ARR = arr_val;
//     switch (channel)
//     {
//     case TIM_CHANNEL_1:
//         tim->CCR1 = ccr_val;
//         break;
//     case TIM_CHANNEL_2:
//         tim->CCR2 = ccr_val;
//         break;
//     case TIM_CHANNEL_3:
//         tim->CCR3 = ccr_val;
//         break;
//     case TIM_CHANNEL_4:
//         tim->CCR4 = ccr_val;
//         break;
//     }
// }

// void Motor1_Speed_Calculate(float speed)
// {
//     uint8_t direction = 0; // 0表示逆时针(CCW)，1表示顺时针(CW)

//     // 判断方向
//     if (speed > 0)
//     {
//         direction = 1; // 顺时针
//     }
//     else if (speed < 0)
//     {
//         direction = 0; // 逆时针
//     }

//     // 设置电机方向
//     Motor1_dir(direction);

//     // 调用通用计算函数
//     motor_speed_calculate_internal(TIM2, TIM_CHANNEL_1, speed);
// }

// void Motor2_Speed_Calculate(float speed)
// {
//     uint8_t direction = 0; // 0表示逆时针(CCW)，1表示顺时针(CW)

//     // 判断方向
//     if (speed > 0)
//     {
//         direction = 1; // 顺时针
//     }
//     else if (speed < 0)
//     {
//         direction = 0; // 逆时针
//     }

//     // 设置电机方向
//     Motor2_dir(direction);

//     // 调用通用计算函数
//     motor_speed_calculate_internal(TIM3, TIM_CHANNEL_1, speed);
// }

// void Motor3_Speed_Calculate(float speed)
// {
//     uint8_t direction = 0; // 0表示逆时针(CCW)，1表示顺时针(CW)

//     // 判断方向
//     if (speed > 0)
//     {
//         direction = 1; // 顺时针
//     }
//     else if (speed < 0)
//     {
//         direction = 0; // 逆时针
//     }

//     // 设置电机方向
//     Motor3_dir(direction);

//     // 调用通用计算函数
//     motor_speed_calculate_internal(TIM4, TIM_CHANNEL_2, speed);
// }

// void Motor4_Speed_Calculate(float speed)
// {
//     uint8_t direction = 0; // 0表示逆时针(CCW)，1表示顺时针(CW)

//     // 判断方向
//     if (speed > 0)
//     {
//         direction = 1; // 顺时针
//     }
//     else if (speed < 0)
//     {
//         direction = 0; // 逆时针
//     }

//     // 设置电机方向
//     Motor4_dir(direction);

//     // 调用通用计算函数
//     motor_speed_calculate_internal(TIM1, TIM_CHANNEL_4, speed);
// }

// void motor_speed_calculate(float speed1, float speed2, float speed3, float speed4)
// {
//     Motor1_Speed_Calculate(speed1);
//     Motor2_Speed_Calculate(speed2);
//     Motor3_Speed_Calculate(speed3);
//     Motor4_Speed_Calculate(speed4);
// }

