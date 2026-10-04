#include "Encoder.h"
#include "tim.h"
#include "system.h"

/**************************************************************************
Function: Read the encoder count
Input   : The timer
Output  : Encoder value (representing speed)
函数功能：读取编码器计数
入口参数：定时器
返回  值：编码器数值(代表速度)
**************************************************************************/
void Encoder_Init(void)
{
    //初始化编码器2(占用TIM5)
    HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
}

//接口为编码器对应定时器编号,返回值即编码器数值，有正有负
int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;
    switch(TIMX)
    {
//        case 2:  Encoder_TIM= (short)TIM2 -> CNT;   TIM2 -> CNT=0;  break;//编码器1对应定时器
        case 5:  Encoder_TIM= (short)TIM5 -> CNT;   TIM5 -> CNT=0;  break;//编码器2对应定时器
        default: Encoder_TIM=0;
    }
    Encoder_TIM=-Encoder_TIM;
    return Encoder_TIM;
}

//获取编码器数值(获取速度)
void Get_Speed(void) 
{
    Motor6_encoder_Speed = Read_Encoder(5);//获取Motor6编码器速度
}
