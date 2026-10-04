#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"
/*PWM通道定义*/
#define IN1_TIM                               &htim1
#define MOTOR1_PUL_CHAANNEL                   TIM_CHANNEL_1

/*螺旋桨引脚定义*/
#define MOTOR1_PUL_PIN                         GPIO_PIN_8
#define MOTOR1_PUL_PORT                        GPIOA
#define MOTOR1_PUL_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()

#define MOTOR1_PUL_TIM                         TIM1
#define MOTOR1_PUL_CLK_ENABLE()                __TIM1_CLK_ENABLE()

#define MOTOR1_PUL_TIM_CHANNEL                 TIM_CHANNEL_1

void motor_init(void);
void motor1_pwm_set(uint16_t duty);


#endif



