#ifndef NORMAL_MOTOR_H
#define NORMAL_MOTOR_H 

#include <stdint.h>

/**
 * @brief 初始化 8 路直流电机 PWM 通道
 */
void normal_motor_init(void);

/**
 * @brief 设置单个直流电机速度
 * @param motor_number 电机编号，1~8
 * @param speed 速度范围 [-100, 100]
 */
void motor_set_speed(uint8_t motor_number, int16_t speed);
void Motor6_Turn180_Start(void);
void Motor6_TurnAngle_Start(int32_t angle_deg);
void Motor6_PositionControl_10ms(void);
int32_t Motor6_GetPositionCount(void);
int16_t Motor6_GetAngleDeg(void);
uint8_t Motor6_IsPositionDone(void);

/**
 * @brief 一次性设置 8 个直流电机速度
 */
void allmotor(int speed_1, int speed_2, int speed_3, int speed_4, int speed_5, int speed_6, int speed_7, int speed_8);
void Motor_Set(uint8_t motor_number, int16_t speed); // 特殊驱动（motor7、motor8）
/**
 * @brief 梯形速度规划（加速-匀速-减速）
 */
float trapezoidal_speed_profile(int32_t elapsed, int32_t total_duration, 
                               float max_speed, int32_t accel_time, int32_t decel_time);

/** 暂停时电机 7/8 梯形减速总档数：每档 10ms（与 HAL_GetTick/10 一致），默认 300 = 3s */
#ifndef MOTOR78_PAUSE_DECEL_TICKS
#define MOTOR78_PAUSE_DECEL_TICKS 300
#endif

/** 串口屏暂停后 7/8 路减速刷新；返回 1 表示本周期已执行 allmotor（勿再与主任务分支重复刷 PWM） */
int motor78_pause_decel_tick(void);

#endif


