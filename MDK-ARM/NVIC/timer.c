#include "timer.h"
#include "system.h"

void timer_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim6.Instance)
    {
        // 10 ms 周期任务。
        if (time_stop_flag == 0)
        {
            time++;
            Time_100ms++;
            Time_1s++;
        }
        

        // 100 ms 周期任务。
        if (Time_100ms >= 10)
        {
					
            Time_100ms = 0;
           
        }

        // 1 s 周期任务。
        if (Time_1s >= 100)
        {
					
            Time_1s = 0;
        }

        // OLED 刷新频率：20 Hz。
        if(OLED_Update_flag != 1 && OLED_Update_time < 5)
        {
            OLED_Update_time++;
        }

        else if(OLED_Update_flag != 1 && OLED_Update_time >= 5)
        {
            OLED_Update_flag = 1;
            OLED_Update_time = 0;
        }
        //张大头超时检测
        zdt_over_time_jc();
        Motor6_PositionControl_10ms();
    }
   
}

void zdt_over_time_jc(void) // 张大头步进电机超时检测。
{
    for(int i = 1; i <= 8; i++) // 循环处理 8 台电机（地址 1~8）。
    {
        if(zdt_over_time[i] >= 0 && zdt_run_flag[i] == 1)
        {
            zdt_over_time[i] -= 0.01;
        }
        else if(zdt_over_time[i] < 0 && zdt_run_flag[i] == 1)
        {
            zdt_run_flag[i] = 0;
        }
    }
}

uint8_t zdt_motor_stop_wait(uint8_t motor_1, uint8_t motor_2, uint8_t motor_3, uint8_t motor_4, 
                           uint8_t motor_5, uint8_t motor_6, uint8_t motor_7, uint8_t motor_8)                    
{
    uint8_t wait = 1;
    wait = ( (motor_1 & zdt_run_flag[1] ) | (motor_2 & zdt_run_flag[2] ) | 
            (motor_3 & zdt_run_flag[3] ) | (motor_4 & zdt_run_flag[4] ) |
            (motor_5 & zdt_run_flag[5] ) | (motor_6 & zdt_run_flag[6] ) | 
            (motor_7 & zdt_run_flag[7] ) | (motor_8 & zdt_run_flag[8] ) );
    return wait;
}

// 电机 1~4 的绝对坐标控制。
void zuobiao_group1(int32_t motor_pos_1, int32_t motor_pos_2, int32_t motor_pos_3, int32_t motor_pos_4)
{
    if(motor_pos_1 > now_pos[1])
    {
        zdt_Pos_mode(1, 250, 236, motor_pos_1 - now_pos[1]);
        delay_ms(3);
    }
    else if(motor_pos_1 < now_pos[1])
    {
        zdt_Pos_mode(1, -250, 236, now_pos[1] - motor_pos_1);
        delay_ms(3);
    }
    
    if(motor_pos_2 > now_pos[2])
    {
        zdt_Pos_mode(2, 100, 206, motor_pos_2 - now_pos[2]);
        delay_ms(3);
    }
    else if(motor_pos_2 < now_pos[2])
    {
        zdt_Pos_mode(2, -100, 206, now_pos[2] - motor_pos_2);
        delay_ms(3);
    }
    
    if(motor_pos_3 > now_pos[3])
    {
        zdt_Pos_mode(3, 100, 206, motor_pos_3 - now_pos[3]);
        delay_ms(3);
    }
    else if(motor_pos_3 < now_pos[3])
    {
        zdt_Pos_mode(3, -100, 206, now_pos[3] - motor_pos_3);
        delay_ms(3);
    }

    if(motor_pos_4 > now_pos[4])
    {
        zdt_Pos_mode(4, 100, 206, motor_pos_4 - now_pos[4]);
        delay_ms(3);
    }
    else if(motor_pos_4 < now_pos[4])
    {
        zdt_Pos_mode(4, -100, 206, now_pos[4] - motor_pos_4);
        delay_ms(3);
    }
    
    now_pos[1] = motor_pos_1;
    now_pos[2] = motor_pos_2;
    now_pos[3] = motor_pos_3;
    now_pos[4] = motor_pos_4;
}

void zuobiao_group2(int32_t motor_pos_5, int32_t motor_pos_6, int32_t motor_pos_7, int32_t motor_pos_8)
{
    if(motor_pos_5 > now_pos[5])
    {
        zdt_Pos_mode(5, 100, 206, motor_pos_5 - now_pos[5]);
        delay_ms(3);
    }
    else if(motor_pos_5 < now_pos[5])
    {
        zdt_Pos_mode(5, -100, 206, now_pos[5] - motor_pos_5);
        delay_ms(3);
    }
    
    if(motor_pos_6 > now_pos[6])
    {
        zdt_Pos_mode(6, 100, 206, motor_pos_6 - now_pos[6]);
        delay_ms(3);
    }
    else if(motor_pos_6 < now_pos[6])
    {
        zdt_Pos_mode(6, -100, 206, now_pos[6] - motor_pos_6);
        delay_ms(3);
    }
    
    if(motor_pos_7 > now_pos[7])
    {
        zdt_Pos_mode(7, 100, 206, motor_pos_7 - now_pos[7]);
        delay_ms(3);
    }
    else if(motor_pos_7 < now_pos[7])
    {
        zdt_Pos_mode(7, -100, 206, now_pos[7] - motor_pos_7);
        delay_ms(3);
    }
    
    if(motor_pos_8 > now_pos[8])
    {
        zdt_Pos_mode(8, 100, 206, motor_pos_8 - now_pos[8]);
        delay_ms(3);
    }
    else if(motor_pos_8 < now_pos[8])
    {
        zdt_Pos_mode(8, -100, 206, now_pos[8] - motor_pos_8);
        delay_ms(3);
    }
    
    now_pos[5] = motor_pos_5;
    now_pos[6] = motor_pos_6;
    now_pos[7] = motor_pos_7;
    now_pos[8] = motor_pos_8;
}

// 单台电机（1~8）的绝对坐标控制。
void zuobiao_single(uint8_t motor_id, int32_t motor_pos)
{
    int16_t vel = 100;
    uint8_t acc = 206;

    if (motor_id < 1 || motor_id > 8) {
        return;
    }

    switch(motor_id)
    {
        case 1: vel = 250; acc = 236;break;      
        case 2: vel = 100; acc = 206;break;
        case 3: vel = 100; acc = 206;break;
        case 4: vel = 60; acc = 106;break;
        case 5: vel = 100; acc = 206;break;
        case 6: vel = 250; acc = 236;break;
        case 7: vel = 100; acc = 206;break;
        case 8: vel = 100; acc = 206;break;
    }
   

    if (motor_pos > now_pos[motor_id]) {
        // 电机 4 向正方向运动时速度较快。
        if (motor_id == 4) { vel = 200; acc = 156; }
        else if(motor_id == 5){ vel = 100; acc = 206; }
        zdt_Pos_mode(motor_id, vel, acc, (uint32_t)(motor_pos - now_pos[motor_id]));
        delay_ms(3);
    } else if (motor_pos < now_pos[motor_id]) {
        // 电机 4 向负方向运动时速度较慢。
        if (motor_id == 4) { vel = 60; acc = 106; }
        else if(motor_id == 5){ vel = 250; acc = 236; }
        zdt_Pos_mode(motor_id, (int16_t)(-vel), acc, (uint32_t)(now_pos[motor_id] - motor_pos));
        delay_ms(3);
    }

    now_pos[motor_id] = motor_pos;
}

void zuobiao_single_custom(uint8_t motor_id, int32_t motor_pos, int16_t vel, uint8_t acc)
{
    if (motor_id < 1 || motor_id > 8) {
        return;
    }

    // 限制速度范围。
    if (vel < 0) vel = -vel;   // 取绝对值。
    if (vel > 300) vel = 300;  // 最大速度限制。
    if (vel < 10) vel = 10;    // 最小速度限制。

    if (motor_pos > now_pos[motor_id]) {
        // 正向运动。
        zdt_Pos_mode(motor_id, vel, acc, (uint32_t)(motor_pos - now_pos[motor_id]));
        delay_ms(3);
    } else if (motor_pos < now_pos[motor_id]) {
        // 反向运动。
        zdt_Pos_mode(motor_id, (int16_t)(-vel), acc, (uint32_t)(now_pos[motor_id] - motor_pos));
        delay_ms(3);
    }
    // 目标位置等于当前位置时，不发送运动命令。

    now_pos[motor_id] = motor_pos;
}

void zuobiao_single_relative_custom(uint8_t motor_id, int32_t delta_pulse, int16_t vel, uint8_t acc)
{
    if (motor_id < 1 || motor_id > 8 || delta_pulse == 0) {
        return;
    }

    if (vel < 0) vel = -vel;
    if (vel > 300) vel = 300;
    if (vel < 10) vel = 10;

    if (delta_pulse > 0) {
        zdt_Pos_mode(motor_id, vel, acc, (uint32_t)delta_pulse);
    } else {
        zdt_Pos_mode(motor_id, (int16_t)(-vel), acc, (uint32_t)(-delta_pulse));
    }
    delay_ms(3);
    now_pos[motor_id] += delta_pulse;
}
void zuobiao(int32_t motor_pos_1, int32_t motor_pos_2, int32_t motor_pos_3, int32_t motor_pos_4,
             int32_t motor_pos_5, int32_t motor_pos_6, int32_t motor_pos_7, int32_t motor_pos_8)
{
    zuobiao_group1(motor_pos_1, motor_pos_2, motor_pos_3, motor_pos_4);
    zuobiao_group2(motor_pos_5, motor_pos_6, motor_pos_7, motor_pos_8);
}
void zdt_stop_read(void)
{
    if (RXdata_4[1].number_finall < 4U)
    {
        return;
    }

    if (RXdata_4[1].Rx[RXdata_4[1].number_finall - 2] == 0x9F && RXdata_4[1].Rx[RXdata_4[1].number_finall - 3] == 0xFD)
    {
        zdt_run_flag[RXdata_4[1].Rx[RXdata_4[1].number_finall - 4]] = 0;
    }
}


