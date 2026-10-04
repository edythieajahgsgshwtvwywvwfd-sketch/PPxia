#include "motor.h"
#include "system.h"
#include "tim.h"  

void motor_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    TIM1->CCR1 = 1500;
}

//1500静止，小于1500反转，大于1500正转
void motor1_pwm_set(uint16_t duty)
{
    // 直接设置占空比寄存器
    TIM1->CCR1 = duty;
}