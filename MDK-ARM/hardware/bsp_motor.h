// #ifndef __BSP_MOTOR_H
// #define __BSP_MOTOR_H

// #include "stdint.h"
// #include "stm32f4xx_hal.h"

// /**************************脉冲电机引脚定义****************************
// EN          DIR          PUL
// PA4        PC2          PA0         M1        T2_CH1
// PA5        PB13         PA6         M2        T3_CH1
// PB3        PB5          PB7         M3        T4_CH2
// PD13       PC9          PA11        M4        T1_CH4
// *******************************************************************/

// /* Motor Control Pins  DIR*/
// #define MOTOR1_DIR_PIN                             GPIO_PIN_2
// #define MOTOR1_DIR_GPIO_PORT                       GPIOC
// #define MOTOR1_DIR_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()

// #define MOTOR2_DIR_PIN                             GPIO_PIN_13
// #define MOTOR2_DIR_GPIO_PORT                       GPIOB
// #define MOTOR2_DIR_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()

// #define MOTOR3_DIR_PIN                             GPIO_PIN_5
// #define MOTOR3_DIR_GPIO_PORT                       GPIOB
// #define MOTOR3_DIR_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()

// #define MOTOR4_DIR_PIN                             GPIO_PIN_9
// #define MOTOR4_DIR_GPIO_PORT                       GPIOC
// #define MOTOR4_DIR_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()

// /* Motor Control Pins  EN*/
// #define MOTOR1_EN_PIN                             GPIO_PIN_4
// #define MOTOR1_EN_GPIO_PORT                       GPIOA
// #define MOTOR1_EN_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()

// #define MOTOR2_EN_PIN                              GPIO_PIN_5
// #define MOTOR2_EN_GPIO_PORT                       GPIOA
// #define MOTOR2_EN_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()

// #define MOTOR3_EN_PIN                              GPIO_PIN_3
// #define MOTOR3_EN_GPIO_PORT                       GPIOB
// #define MOTOR3_EN_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()

// #define MOTOR4_EN_PIN                              GPIO_PIN_13
// #define MOTOR4_EN_GPIO_PORT                       GPIOD
// #define MOTOR4_EN_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()

// /*Motor Control Pins  PULSE*/
// #define MOTOR1_PUL_TIM                             TIM2
// #define MOTOR1_PUL_CLK_ENABLE()                    __TIM2_CLK_ENABLE()
// #define MOTOR1_PUL_GPIO_PORT                      GPIOA
// #define MOTOR1_PUL_PIN                            GPIO_PIN_0
// #define MOTOR1_PUL_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
// #define MOTOR1_PUL_CHANNEL                       TIM_CHANNEL_1

// #define MOTOR2_PUL_TIM                             TIM3
// #define MOTOR2_PUL_CLK_ENABLE()                    __TIM3_CLK_ENABLE()
// #define MOTOR2_PUL_GPIO_PORT                      GPIOA
// #define MOTOR2_PUL_PIN                            GPIO_PIN_6
// #define MOTOR2_PUL_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
// #define MOTOR2_PUL_CHANNEL                       TIM_CHANNEL_1

// #define MOTOR3_PUL_TIM                             TIM4
// #define MOTOR3_PUL_CLK_ENABLE()                    __TIM4_CLK_ENABLE()
// #define MOTOR3_PUL_GPIO_PORT                      GPIOB
// #define MOTOR3_PUL_PIN                            GPIO_PIN_7
// #define MOTOR3_PUL_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
// #define MOTOR3_PUL_CHANNEL                       TIM_CHANNEL_2

// #define MOTOR4_PUL_TIM                             TIM1
// #define MOTOR4_PUL_CLK_ENABLE()                    __TIM1_CLK_ENABLE()
// #define MOTOR4_PUL_GPIO_PORT                      GPIOA
// #define MOTOR4_PUL_PIN                            GPIO_PIN_11
// #define MOTOR4_PUL_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
// #define MOTOR4_PUL_CHANNEL                       TIM_CHANNEL_4


// #define HIGH                                      GPIO_PIN_SET
// #define LOW                                       GPIO_PIN_RESET

// #define ON                                        HIGH
// #define OFF                                       LOW

// #define CW                                        HIGH         //顺时针
// #define CCW                                       LOW         //逆时针

// #define MOTOR1_EN(x)                 HAL_GPIO_WritePin(MOTOR1_EN_GPIO_PORT,MOTOR1_EN_PIN,x)
// #define MOTOR1_DIR(x)                HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_PORT,MOTOR1_DIR_PIN,x)
// #define MOTOR1_PUL(x)                HAL_GPIO_WritePin(MOTOR1_PUL_GPIO_PORT,MOTOR1_PUL_PIN,x)

// #define MOTOR2_EN(x)                 HAL_GPIO_WritePin(MOTOR2_EN_GPIO_PORT,MOTOR2_EN_PIN,x)
// #define MOTOR2_DIR(x)                HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_PORT,MOTOR2_DIR_PIN,x)
// #define MOTOR2_PUL(x)                HAL_GPIO_WritePin(MOTOR2_PUL_GPIO_PORT,MOTOR2_PUL_PIN,x)

// #define MOTOR3_EN(x)                 HAL_GPIO_WritePin(MOTOR3_EN_GPIO_PORT,MOTOR3_EN_PIN,x)
// #define MOTOR3_DIR(x)                HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_PORT,MOTOR3_DIR_PIN,x)
// #define MOTOR3_PUL(x)                HAL_GPIO_WritePin(MOTOR3_PUL_GPIO_PORT,MOTOR3_PUL_PIN,x)

// #define MOTOR4_EN(x)                 HAL_GPIO_WritePin(MOTOR4_EN_GPIO_PORT,MOTOR4_EN_PIN,x)
// #define MOTOR4_DIR(x)                HAL_GPIO_WritePin(MOTOR4_DIR_GPIO_PORT,MOTOR4_DIR_PIN,x)
// #define MOTOR4_PUL(x)                HAL_GPIO_WritePin(MOTOR4_PUL_GPIO_PORT,MOTOR4_PUL_PIN,x)

// void Motor_Init(void);
// void Motor1_enable(void);
// void Motor2_enable(void);
// void Motor3_enable(void);
// void Motor4_enable(void);
// void motor_all_enable(void);
// void Motor1_dir(uint8_t dir);
// void Motor2_dir(uint8_t dir);
// void Motor3_dir(uint8_t dir);
// void Motor4_dir(uint8_t dir);
// void dir_control(void);
// void Motor1_Speed_Calculate(float speed);
// void Motor2_Speed_Calculate(float speed);
// void Motor3_Speed_Calculate(float speed);
// void Motor4_Speed_Calculate(float speed);
// void motor_speed_calculate(float speed1, float speed2, float speed3, float speed4);

// #endif
