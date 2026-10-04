#include "task_execute.h"
#include "jdq.h"
#include "system.h"
#include "timer.h"
#include "layser.h"

#define STEPPER78_ABS_PULSE(pulse) ((pulse) / 2)
#define M7_MIRROR_ABS(pulse) STEPPER78_ABS_PULSE((pulse) - (41 * 3200))

uint8_t g_peel_upper_first_mode = 0;   // 0: old flow, 1: upper shell first
static uint8_t upper_shell_first_flow = 0;
static uint8_t thirteen_bg_enable = 0;
static uint8_t thirteen_bg_ctrl = 0;
static uint32_t thirteen_bg_time_b = 0;
static uint32_t mode2_motor1_arrive_wait_ms = 0;

// 流程状态机：run_mode 选择工序，ctrl_flag 选择工序内的子步骤。
// 步进电机位置以 3200 脉冲为一单位；直流电机速度由 Normal_motor_Speed_* 保存。

// 上料与定位。
void run_mode_zero(void)
{
    // 启动电机 3/4 输送，等待光电 2 检测到物料。
    if(ctrl_flag == 0)
    {
        Normal_motor_Speed_3 = 60;
        Normal_motor_Speed_4 = -60;
        screen_update_step(1); // 串口屏显示步骤 1。
        ctrl_flag = 1;
    }
    else if(ctrl_flag == 1 && gd_detected_flag_2 == 1)
    {
        Normal_motor_Speed_3 = 0;
        Normal_motor_Speed_4 = 0;
        time_stop_flag = 0; // 保持系统计时运行。
        ctrl_flag = 2;
    }

    // 光电触发后微调步进电机 1，再进入下一工序。
    else if(ctrl_flag == 2)
    {
        zuobiao_single_custom(1, -1200, 250, 206);
        Normal_motor_Speed_3 = 0;
        Normal_motor_Speed_4 = 0;
        ctrl_flag = 3;
        time_b = time;
    }

    else if(ctrl_flag == 3 && time > time_b + 200)
    {   
        run_mode = 1;
        ctrl_flag = 0;
        time_b = time;
    }
}

void run_mode_one(void)
{
   // 显示步骤 2 并启动直流电机 1。
   if(ctrl_flag == 0 && time > time_b && time < time_b + 100)
   {
        screen_update_step(2); // 串口屏显示步骤 2。
        Normal_motor_Speed_1 = 92;
        ctrl_flag = 1;
   }
   // 延时后切换继电器 3/4。
   else if (ctrl_flag == 1 && time > time_b + 100 && time < time_b + 1000)
   {
   //     Normal_motor_Speed_1 = 0;     
        ctrl_flag = 2;
       jdq_on(JDQ_CH3);
       jdq_off(JDQ_CH4);
   }
   else if(time > time_b + 1000 && ctrl_flag == 2)
   { 
        run_mode = 2;
        Normal_motor_Speed_1 = 0;
        ctrl_flag = 0;
        time_b = time;
   }
    
}

void run_mode_two(void)
{
     // 先驱动步进电机 1 到目标位置，并监测光电 3。
     
     if (ctrl_flag == 0)
     {
         // 电机 1 向 8 圈对应的绝对坐标运动。
         zuobiao_single(1, 8 * 3200);
         ctrl_flag = 1;   
         time_stop_flag = 1; // 暂停流程时基，等待电机或光电事件。
         time_b = time;
     }
     else if (ctrl_flag == 1)
     {
        if (gd_detected_flag_3 == 0)
        {
            // 未触发光电 3：电机到位后再等待 1.5 s，然后继续流程。
            if(zdt_motor_stop_wait(1, 0, 0, 0, 0, 0, 0, 0) == 0)
            {
                if(mode2_motor1_arrive_wait_ms == 0U)
                {
                    mode2_motor1_arrive_wait_ms = HAL_GetTick();
                }
                else if(HAL_GetTick() - mode2_motor1_arrive_wait_ms > 1500U)
                {
                    time_stop_flag = 0;
                    ctrl_flag = 2;
                    time_b = time;
                    mode2_motor1_arrive_wait_ms = 0U;
                }
            }
        }
        else if (gd_detected_flag_3 == 1)
        {
            // 光电 3 已触发：立即停止电机 1，并转入短暂缓冲状态。
            Emm_V5_Stop_Now(1, false);
            zdt_run_flag[1] = 0;
            ctrl_flag = 20;
            time_stop_flag = 0;
            time_b = time;
            mode2_motor1_arrive_wait_ms = 0U;
        }
        else
         {
            // 保留异常状态分支。
         }
     }     
 
    else if(ctrl_flag == 20)
    {
 //       Emm_V5_Stop_Now(1, false);
        zdt_run_flag[1] = 0;
        if(time > time_b + 5)
        {
            ctrl_flag = 2;
            time_b = time;
        }
    }
     
 
    // 电机 5 前进 9 圈，完成后切换串口屏到步骤 3。
    else if(ctrl_flag == 2)
     {
         zuobiao_single_custom(5, now_pos[5] + 9 * 3200, 250, 206); // 相对当前位置前进 9 圈。
//         delay_ms(1000);
         ctrl_flag = 3;
         time_b = time;
        
     }

     else if(ctrl_flag == 3)
     {
         if(zdt_motor_stop_wait(0, 0, 0, 0, 1, 0, 0, 0) == 0)
         {
             zdt_run_flag[5] = 0;
			ctrl_flag = 4;
             screen_update_step(3); // 串口屏显示步骤 3。
             
             time_b = time;
         }
     }

     // 电机 4 下行时，让直流电机 8 按梯形曲线加速。
     else if (ctrl_flag == 4)
     {
         g_elapsed = time - time_b;
 
         // 仅下发一次电机 4 的第一段目标位置。
         if (step_ctrl_flag == 0)
         {
             zuobiao_single_custom(4, -6 * 3200, 280, 236); // 电机 4 到 -6 圈。
             step_ctrl_flag = 1;
         }
         
         // 直流电机 8 加速阶段。
         if (g_elapsed < 200)
         {
             g_target_speed = trapezoidal_speed_profile(g_elapsed, 200, 60, 200, 0);
             Normal_motor_Speed_8 = (int)g_target_speed;
         }
         
         // 加速结束后维持速度，等待电机 4 到位。
         else
         {
             // 电机 8 保持当前工艺速度。
             Normal_motor_Speed_8 = 60;
             
             // 电机 4 到位后进入下一子步骤。
             if(zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
             {
                 ctrl_flag = 5;
                 time_b = time;
                 step_ctrl_flag = 0;
             }
         }
     }
     // 电机 4 继续下行到 -12 圈。
     else if(ctrl_flag == 5)
     {
         zuobiao_single_custom(4, -12 * 3200, 180, 106); // 从 -6 圈继续到 -12 圈。
         ctrl_flag = 6;
         time_b = time;
     }

     else if(ctrl_flag == 6)
     {
         if(zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
         {
             ctrl_flag = 7;
             time_b = time;
         }
     }

     // 延时后让电机 8 减速，同时让电机 4 回到零位。
     else if (ctrl_flag == 7 && time > time_b + 100)
     {
         g_elapsed = time - (time_b + 100);
         
         // 电机 8 减速阶段。
         if (g_elapsed < 200)
         {
             g_target_speed = trapezoidal_speed_profile(g_elapsed, 200, 60, 0, 200);
             Normal_motor_Speed_8 = (int)g_target_speed;
         }
         else
         {
             Normal_motor_Speed_8 = 0;
         }
         
         // 仅下发一次电机 4 回零命令。
         if (step_ctrl_flag == 0)
         {
             zuobiao_single_custom(4, 0, 280, 236); // 电机 4 回到软件零位。
             step_ctrl_flag = 1;
         }
         
         // 电机 8 停止且电机 4 到位后结束本工序。
         if (g_elapsed >= 200 && zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
         {
             ctrl_flag = 8;
             time_b = time;
             step_ctrl_flag = 0;
         }
     }
 
     else if(ctrl_flag == 8)
     {   
         run_mode = 3;
         ctrl_flag = 0;
         time_b = time;
     }
}


// 切割与输送流程。
void run_mode_three(void)
{
    // 电机 5 先移动到 27 圈位置，然后重复电机 4 与电机 8 的配合动作。
    if(ctrl_flag == 0 && time > time_b)
    {
        zuobiao_single_custom(5, 27 * 3200, 280, 236); // 电机 5 到 27 圈绝对位置。
        ctrl_flag = 1;
        time_b = time;
    }

    else if(ctrl_flag == 1)
    {
        if(zdt_motor_stop_wait(0, 0, 0, 0, 1, 0, 0, 0) == 0)
        {
            screen_update_step(4); // 串口屏显示步骤 4。
            ctrl_flag = 2;
            time_b = time;
        }
    }

    // 电机 4 下行时，电机 8 加速。
    else if (ctrl_flag == 2)
    {
        g_elapsed = time - time_b;

        if (step_ctrl_flag == 0)
        {
            zuobiao_single_custom(4, -6 * 3200, 280, 236); // 电机 4 到 -6 圈。
            step_ctrl_flag = 1;
        }
 
        // 电机 8 加速阶段。
        if (g_elapsed < 200)
        {
            g_target_speed = trapezoidal_speed_profile(g_elapsed, 200, 60, 200, 0);
            Normal_motor_Speed_8 = (int)g_target_speed;
        }
        // 加速结束后保持速度并等待电机 4。
         else
        {
            // 电机 8 保持工艺速度。
            Normal_motor_Speed_8 = 60;
            
            // 电机 4 到位后继续。
            if(zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
            {
                ctrl_flag = 3;
                time_b = time;
                step_ctrl_flag = 0;
            }
        }
    }
    // 电机 4 继续下行到 -12 圈。
    else if(ctrl_flag == 3 && time > time_b)
    {
        zuobiao_single_custom(4, -12 * 3200, 180, 106); // 从 -6 圈继续到 -12 圈。
        ctrl_flag = 4;
        time_b = time;
    }

    else if(ctrl_flag == 4)
    {
        if(zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
        {
            ctrl_flag = 5;
            time_b = time;
        }
    }

     //step5
     else if (ctrl_flag == 5 && time > time_b + 100)
     {
         g_elapsed = time - (time_b + 100);
         if (g_elapsed < 200)
         {
             g_target_speed = trapezoidal_speed_profile(g_elapsed, 200, 60, 0, 200);
             Normal_motor_Speed_8 = (int)g_target_speed;
         }
         else
         {
             Normal_motor_Speed_8 = 0;
         }

         if(step_ctrl_flag == 0)
         {
             zuobiao_single_custom(4, 0, 280, 236); // 电机 4 回到软件零位。
             step_ctrl_flag = 1;
         }
         if(g_elapsed >= 200 && zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
         {
            screen_update_step(5); // 串口屏显示步骤 5。
             ctrl_flag = 6;
             time_b = time;
             step_ctrl_flag = 0;
         }
     }

     // 延时后让直流电机 7 加速，同时移动步进电机 5。
     else if(ctrl_flag == 6 && time > time_b + 50)
     {
         // 电机 7 的目标速度为 -40，使用 500 个 10 ms 节拍加速。
         g_elapsed = time - (time_b + 50);

         // 电机 7 加速阶段。
         if (g_elapsed < 500)
         {
             g_target_speed = trapezoidal_speed_profile(g_elapsed, 500, 40, 500, 0);
             Normal_motor_Speed_7 = -(int)g_target_speed;
         }

         if(step_ctrl_flag == 0)
         {
             zuobiao_single_custom(5, 67 * 3200, 150, 136);
             step_ctrl_flag = 1;
         }

         else
         {
            Normal_motor_Speed_7 = -40; // 保持恒定速度并等待电机 5 到位。
            ctrl_flag = 7;
            step_ctrl_flag = 0;
            time_b = time;
         }
     }

    else if(ctrl_flag == 7)
    {
        if(zdt_motor_stop_wait(0, 0, 0, 0, 1, 0, 0, 0) == 0)
        {
            ctrl_flag = 8;
            time_b = time;
        }
    }

    // 电机 5 到位后，让电机 7 平滑减速到 0。
    else if(ctrl_flag == 8)
    {
        g_elapsed = time - time_b;
        if (g_elapsed < 500)
        {
            g_target_speed = trapezoidal_speed_profile(g_elapsed, 500, 40, 0, 500);
            Normal_motor_Speed_7 = -(int)g_target_speed;
        }
        else
        {
            Normal_motor_Speed_7 = 0;
            ctrl_flag = 9;
            time_b = time;
        }
    }

    else if(ctrl_flag == 9)
    {
        run_mode_thirteen_start();
        run_mode = 4;
//        zuobiao_single_custom(5, 0, 280, 236);
        ctrl_flag = 0;
        time_b = time;
    }

    // 本工序结束后进入 run_mode_four。
}

void run_mode_four(void)
{
    time_stop_flag = 0;
    if(ctrl_flag == 0 && time > time_b)
    {
        screen_update_step(6); // 串口屏显示步骤 6。
        
        // 设置第一组继电器状态。
        jdq_on(JDQ_CH1);
        jdq_off(JDQ_CH2);
        jdq_on(JDQ_CH3);
        jdq_off(JDQ_CH4);
        ctrl_flag = 1;
    }

    else if(ctrl_flag == 1 && time > time_b + 150)
    {
        // 电机 5 移动到 97.5 圈位置。
        zuobiao_single_custom(5, 97.5 * 3200, 280, 206);  //30.5     96.5
        ctrl_flag = 2;
        time_b = time;
    }
    
    else if(ctrl_flag == 2)
    {
        if(zdt_motor_stop_wait(0, 0, 0, 0, 1, 0, 0, 0) == 0)
        {
            ctrl_flag = 3;
            time_b = time;
        }
    }
    
    else if(ctrl_flag == 3 && time > time_b && time < time_b + 250)
    {
        // 切换继电器 1/2 状态。
        jdq_off(JDQ_CH1);
        jdq_on(JDQ_CH2);
        ctrl_flag = 4;
    }

    else if(ctrl_flag == 4 && time > time_b + 250)
    {
        screen_update_step(6); // 保持步骤 6 页面。
        // 切换继电器后，电机 6 向负方向运动。
        jdq_off(JDQ_CH1);
        jdq_on(JDQ_CH2);
        zuobiao_single_custom(6, -29 * 3200, 280, 156);   //-29.5
        ctrl_flag = 5;
        time_b = time;
    }
    
    else if(ctrl_flag == 5)
    {
        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 1, 0, 0) == 0)
        {
            ctrl_flag = 6;
            time_b = time;
            Normal_motor_Speed_1 = 85;
        }
    }

    else if(ctrl_flag == 6 && time > time_b + 50 && time < time_b + 150)
    {
        jdq_off(JDQ_CH3);
        jdq_on(JDQ_CH4);
        jdq_off(JDQ_CH2);
        jdq_on(JDQ_CH1);
        Normal_motor_Speed_1 = 0;

        ctrl_flag = 7;
    }

    else if(ctrl_flag == 7 && time > time_b + 150 && time < time_b + 200)
    {
        jdq_off(JDQ_CH3);
        jdq_on(JDQ_CH4);
        zuobiao_single_custom(5, 0, 280, 206); 
        ctrl_flag = 8;
        time_b = time;
    }


    else if(ctrl_flag == 8)
    {
        jdq_off(JDQ_CH3);
        jdq_on(JDQ_CH4);         
        upper_shell_first_flow = 0;
        if(g_peel_upper_first_mode == 0)
        {
            run_mode = 9;      // old flow: 9 -> 5 -> 10 -> 6
        }
        else
        {
            run_mode = 8;      // upper shell first: 8 -> 6 -> 10 -> 5
        }
        ctrl_flag = 0;
        time_b = time;
    }

    // 根据 g_peel_upper_first_mode 选择后续剥壳顺序。
}

void run_mode_five(void)
{
     time_stop_flag = 0;

    if(ctrl_flag == 0)
    {
        screen_update_step(7);
        // 设置剥壳机构的继电器状态和舵机初始角度。
        jdq_off(JDQ_CH1);
        jdq_on(JDQ_CH2);
        ctrl_flag = 1;
        servo_set_angle_270(SERVO_CH1, 120);
    }

    else if(ctrl_flag == 1 && time > time_b + 200)
    {
         
        jdq_off(JDQ_CH1);
        jdq_on(JDQ_CH2);
        // 电机 7/8 移动到第一组剥壳轨迹起点。
        zuobiao_single_custom(7, STEPPER78_ABS_PULSE(-16 * 3200), 280, 206);
        zuobiao_single_custom(8, STEPPER78_ABS_PULSE(-62 * 3200), 280, 206);
        ctrl_flag = 2;
        time_b = time;
    }

    else if(ctrl_flag == 2)
    {
        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
       {
            servo_set_angle_270(SERVO_CH1, 120);
            servo_break_smooth_start(120, 86, 0);
            ctrl_flag = 3;
            time_b = time;
        }
    }

    else if(ctrl_flag == 3 && servo_finish_flag == 0)
    {
        servo_break_smooth_update();

        if(servo_finish_flag == 1)
        {
            ctrl_flag = 4;
            time_b = time;
        }
    }

    // 舵机从 86 度同步运动到 79 度，同时更新电机 7/8 位置。
    else if(ctrl_flag == 4)
    {
        zuobiao_single_custom(8, STEPPER78_ABS_PULSE(-54 * 3200), 250, 206);
 //       zuobiao_single_custom(7, STEPPER78_ABS_PULSE(-8 * 3200), 120, 36);
        zuobiao_single_custom(7, STEPPER78_ABS_PULSE(-18 * 3200), 120, 36);
        // 让 SERVO_CH1 与电机 7 同步。
        servo_start_sync(SERVO_CH1, 7, 86, 79);
        
        ctrl_flag = 5;
        time_b = time;
    }

    // 持续更新同步运动，等待电机 7/8 到位。
    else if(ctrl_flag == 5)
    {
        servo_update_sync(); 

        
        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
        {
            ctrl_flag = 6;
            time_b = time;
        }
    }

    // 第二段同步轨迹：舵机 79 -> 35 度，电机 7/8 同步移动。
    else if(ctrl_flag == 6)
    {
        zuobiao_single_custom(8, STEPPER78_ABS_PULSE(-25 * 3200), 250, 206);
        zuobiao_single_custom(7, STEPPER78_ABS_PULSE(-26 * 3200), 120, 36);
        
        servo_start_sync(SERVO_CH1, 7, 79, 35);
        
        ctrl_flag = 7;
        time_b = time;
    }

    // 更新第二段同步运动并等待到位。
    else if(ctrl_flag == 7)
    {
        servo_update_sync(); 

        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
        {
            ctrl_flag = 8;
            zuobiao_single_custom(7, STEPPER78_ABS_PULSE(-41 * 3200), 280, 206);
            zuobiao_single_custom(8, STEPPER78_ABS_PULSE(0 * 3200), 280, 206);
            
            // 启动最后一段舵机同步轨迹。
            servo_start_sync(SERVO_CH1, 7, 35, 0);
            time_b = time;
        }
    }

    // 更新最后一段同步运动并等待到位。
    else if(ctrl_flag == 8)
    {
        servo_update_sync(); 

        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
        {
            ctrl_flag = 9;
            time_b = time;
            servo_set_angle_270(SERVO_CH1, 120);
        }
    }

    else if (ctrl_flag == 9 && time > time_b + 100)
    {
        if(g_peel_upper_first_mode == 1 && upper_shell_first_flow == 2)
        {
            upper_shell_first_flow = 0;
            ctrl_flag = 10;
            time_b = time;
        }
        else
        {
            run_mode = 10; // 默认流程：进入中间换向工序，再转到 run_mode_six。
            ctrl_flag = 0;
            time_b = time;
        }
    }

     // 左侧剥壳轨迹结束。
 }

void run_mode_six(void)
{
    time_stop_flag = 0;

    if(ctrl_flag == 0)
    {
        moveServoAngle270(0, 187.0f, 1000);
        ctrl_flag = 1;
    }

    else if(ctrl_flag == 1)
    {
        servo_set_angle_270(SERVO_CH1, 120);
        // 电机 7/8 移动到镜像剥壳轨迹起点。
        zuobiao_single_custom(7, M7_MIRROR_ABS(16 * 3200), 280, 206);
        zuobiao_single_custom(8, STEPPER78_ABS_PULSE(-62 * 3200), 280, 206);
        ctrl_flag = 2;
        time_b = time;
    }

    else if(ctrl_flag == 2)
    {
        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
       {
            servo_set_angle_270(SERVO_CH1, 120);
            servo_break_smooth_start(120, 86, 1);
            ctrl_flag = 3;
            time_b = time;
        }
    }

    else if(ctrl_flag == 3 && servo_finish_flag == 0)
    {
        servo_break_smooth_update();

        if(servo_finish_flag == 1)
        {
            ctrl_flag = 4;
            time_b = time;
        }
    }

    // 镜像轨迹的主同步段。
    else if(ctrl_flag == 4)
    {
        zuobiao_single_custom(8, STEPPER78_ABS_PULSE(-25 * 3200), 280, 206);
        zuobiao_single_custom(7, M7_MIRROR_ABS(14 * 3200), 120, 36);
        
        // 让 SERVO_CH1 与电机 7 快速同步。
        servo_start_sync_fast(SERVO_CH1, 7, 86, 35);
        
        ctrl_flag = 5;
        time_b = time;
    }

    // 更新主同步段并等待到位。
    else if(ctrl_flag == 5)
    {
        servo_update_sync(); 

        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
        {
            ctrl_flag = 6;
            zuobiao_single_custom(7, M7_MIRROR_ABS(41 * 3200), 280, 206);
            zuobiao_single_custom(8, STEPPER78_ABS_PULSE(0 * 3200), 280, 206);
            
            // 启动镜像轨迹的收尾同步段。
            servo_start_sync(SERVO_CH1, 7, 35, 0);
            
            time_b = time;
        }
    }

    // 更新收尾同步段，完成后复位相关机构。
    else if(ctrl_flag == 6)
    {
        servo_update_sync(); 

        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 1) == 0)
        {
           // ctrl_flag = 7;
            time_b = time;
            moveServoAngle270(0, 7, 1000);
            servo_set_angle_270(SERVO_CH1, 120);
            zuobiao_single_custom(6, 0, 280, 206);
            if(g_peel_upper_first_mode == 1 && upper_shell_first_flow == 1)
            {
                run_mode = 10;
                ctrl_flag = 0;
                time_b = time;
            }
        }
    }

    // 右侧（镜像）剥壳轨迹结束。

}

void run_mode_seven(void)
{
    time_stop_flag = 0;
     // 重复电机 4 与直流电机 8 的切割配合动作。
     if (ctrl_flag == 0)
     {
         g_elapsed = time - time_b;
 
         // 仅下发一次电机 4 的第一段目标。
         if (step_ctrl_flag == 0)
         {
             zuobiao_single_custom(4, -6 * 3200, 250, 236); // 电机 4 到 -6 圈。
             step_ctrl_flag = 1;
         }
         
         // 电机 8 加速阶段。
         if (g_elapsed < 200)
         {
             g_target_speed = trapezoidal_speed_profile(g_elapsed, 200, g_max_pro_speed, 200, 0);
             Normal_motor_Speed_8 = (int)g_target_speed;
         }
         
         // 加速结束后保持速度，并等待电机 4 到位。
         else
         {
             // 电机 8 保持工艺速度。
             Normal_motor_Speed_8 = (int)g_max_pro_speed;
             
             // 电机 4 到位后继续。
             if(zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
             {
                 ctrl_flag = 5;
                 time_b = time;
                 step_ctrl_flag = 0;
             }
         }
     }
     // 电机 4 继续下行到 -12 圈。
     else if(ctrl_flag == 5)
     {
         zuobiao_single_custom(4, -12 * 3200, 100, 106); // 从 -6 圈继续到 -12 圈。
         ctrl_flag = 6;
         time_b = time;
     }

     else if(ctrl_flag == 6)
     {
         if(zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
         {
             ctrl_flag = 7;
             time_b = time;
         }
     }

     // 延时后让电机 8 减速，同时让电机 4 回零。
     else if (ctrl_flag == 7 && time > time_b + 100)
     {
         g_elapsed = time - (time_b + 100);
         
         // 电机 8 减速阶段。
         if (g_elapsed < 200)
         {
             g_target_speed = trapezoidal_speed_profile(g_elapsed, 200, g_max_pro_speed, 0, 200);
             Normal_motor_Speed_8 = (int)g_target_speed;
         }
         else
         {
             Normal_motor_Speed_8 = 0;
         }
         
         // 仅下发一次电机 4 回零命令。
         if (step_ctrl_flag == 0)
         {
             zuobiao_single_custom(4, 0, 250, 236); // 电机 4 回到软件零位。
             step_ctrl_flag = 1;
         }
         
         // 电机 8 停止且电机 4 到位后完成该动作。
         if (g_elapsed >= 200 && zdt_motor_stop_wait(0, 0, 0, 1, 0, 0, 0, 0) == 0)
         {
             ctrl_flag = 8;
             time_b = time;
             step_ctrl_flag = 0;
         }
     }


}

void run_mode_eight(void)
{
    time_stop_flag = 0;

    if(g_peel_upper_first_mode == 0)
    {
        upper_shell_first_flow = 0;
        run_mode = 9;
        ctrl_flag = 0;
        time_b = time;
        return;
    }

    if(ctrl_flag == 0)
    {
        zuobiao_single_custom(7, STEPPER78_ABS_PULSE(-41 * 3200), 280, 206);
        ctrl_flag = 1;
        time_b = time;
    }
    else if(ctrl_flag == 1)
    {
        if(zdt_motor_stop_wait(0, 0, 0, 0, 0, 0, 1, 0) == 0)
        {
            moveServoAngle270(0, 187.0f, 1000);
            ctrl_flag = 2;
            time_b = time;
        }
    }
    else if(ctrl_flag == 2 && time > time_b + 100)
    {
        upper_shell_first_flow = 1;
        run_mode = 6;
        ctrl_flag = 0;
        time_b = time;
    }

    // 上壳优先流程的预定位完成，随后进入镜像剥壳轨迹。
}
void run_mode_nine(void)
{
   time_stop_flag = 0;
    if(ctrl_flag == 0 && time > time_b + 100 && time < time_b + 250)
    {
        // 切换继电器，为电机 6 翻转做准备。
        jdq_off(JDQ_CH2);
        jdq_on(JDQ_CH1);
        ctrl_flag = 1;
    }

     if(ctrl_flag == 1 && time > time_b + 250)
    {
        jdq_off(JDQ_CH2);
        jdq_on(JDQ_CH1);
        g_motor6_turn180_done_flag = 0;
        Motor6_TurnAngle_Start(180);
        ctrl_flag = 2;
        time_b = time;
    }
    else if(ctrl_flag == 2)
    {
        if(Motor6_IsPositionDone() == 1)
        {
            g_motor6_turn180_done_flag = 1;
            ctrl_flag = 3;
            time_b = time;
        }
    }

    else if(ctrl_flag == 3)
    {
        g_motor6_turn180_done_flag = 0;
        run_mode = 5;
        time_b = time;
        ctrl_flag = 0;
    }

    // 电机 6 翻转 180 度后进入 run_mode_five。
}

void run_mode_ten(void)
{
     time_stop_flag = 0;

    if(ctrl_flag == 0)
    {
        servo_set_angle_270(SERVO_CH1, 0);
        ctrl_flag = 1;
    }

    else if(ctrl_flag == 1 && time > time_b + 100 && time < time_b + 150)
    {
        // 切换继电器，为反向翻转做准备。
        
        jdq_off(JDQ_CH2);
        jdq_on(JDQ_CH1);
        ctrl_flag = 2;
    }

    else if(ctrl_flag == 2 && time > time_b + 150)
    {
        g_motor6_turn180_done_flag = 0;
        Motor6_TurnAngle_Start(179);
        ctrl_flag = 3;
        time_b = time;
    }
    else if(ctrl_flag == 3)
    {
        if(Motor6_IsPositionDone() == 1)
        {
            g_motor6_turn180_done_flag = 1;
            ctrl_flag = 4;
            time_b = time;
        }
    }

    else if(ctrl_flag == 4)
    {
        jdq_off(JDQ_CH1);
        jdq_on(JDQ_CH2);
        g_motor6_turn180_done_flag = 0;
        if(g_peel_upper_first_mode == 1 && upper_shell_first_flow == 1)
        {
            moveServoAngle270(0, 7, 1000);
            servo_set_angle_270(SERVO_CH1, 120);
            upper_shell_first_flow = 2;
            run_mode = 5;
        }
        else
        {
            run_mode = 6;
        }
        time_b = time;
        ctrl_flag = 0;
    }
    // 电机 6 翻转约 180 度后，根据剥壳顺序选择下一工序。
}

void run_mode_eleven(void)
{
    time_stop_flag = 0;
    if(ctrl_flag == 0)
    {
   //     moveServoAngle(1, 10.0f, 5000);
        ctrl_flag = 1;
    }
  
    //servo_set_angle_270(SERVO_CH1, 20);
   
}
void run_mode_twelve(void)
{
  time_stop_flag = 0;
    if(ctrl_flag == 0 && time > time_b + 100 && time < time_b + 250)
    {
        ctrl_flag = 1;
    }

     if(ctrl_flag == 1 && time > time_b + 250)
    {
        // jdq_off(JDQ_CH2);
        // jdq_on(JDQ_CH1);
        g_motor6_turn180_done_flag = 0;
        Motor6_TurnAngle_Start(-180);
        ctrl_flag = 2;
        time_b = time;
    }
    else if(ctrl_flag == 2)
    {
        if(Motor6_IsPositionDone() == 1)
        {
             g_motor6_turn180_done_flag = 0;
          //  Motor6_TurnAngle_Start(180);
            ctrl_flag = 30;
            time_b = time;
        }
    }

    else if(ctrl_flag == 3)
    {
       if(Motor6_IsPositionDone() == 1)
       {
             g_motor6_turn180_done_flag = 1;
            ctrl_flag = 4;
            time_b = time;
 
       }
    }


}

void run_mode_thirteen_start(void)
{
    if(thirteen_bg_enable == 0)
    {
        thirteen_bg_enable = 1;
        thirteen_bg_ctrl = 0;
        thirteen_bg_time_b = time;
    }
}

void run_mode_thirteen_update(void)
{
    if(thirteen_bg_enable == 0)
    {
        return;
    }

    if(thirteen_bg_ctrl == 0)
    {
        moveServoAngle(1, 35, 1000);
        zuobiao_single_custom(3, 16 * 3200, 250, 206);
        thirteen_bg_ctrl = 1;
        thirteen_bg_time_b = time;
    }
    else if(thirteen_bg_ctrl == 1)
    {
        if(zdt_motor_stop_wait(0, 0, 1, 0, 0, 0, 0, 0) == 0)
        {
            Normal_motor_Speed_2 = -60;
            thirteen_bg_time_b = time;
            thirteen_bg_ctrl = 2;
        }
    }
    else if(thirteen_bg_ctrl == 2 && time > thirteen_bg_time_b + 300)
    {
        Normal_motor_Speed_2 = 0;
        thirteen_bg_ctrl = 3;
        thirteen_bg_time_b = time;
    }
    else if(thirteen_bg_ctrl == 3)
    {
        zuobiao_single_custom(3, 0, 250, 206);
        Normal_motor_Speed_2 = 60;
        thirteen_bg_ctrl = 4;
        thirteen_bg_time_b = time;
    }
    else if(thirteen_bg_ctrl == 4 && time > thirteen_bg_time_b + 300)
    {
        Normal_motor_Speed_2 = 0;
        moveServoAngle(1, 90, 1000);
        thirteen_bg_ctrl = 5;
        thirteen_bg_time_b = time;
    }
    else if(thirteen_bg_ctrl == 5)
    {
        if(zdt_motor_stop_wait(0, 0, 1, 0, 0, 0, 0, 0) == 0)
        {
            thirteen_bg_enable = 0;
            thirteen_bg_ctrl = 0;
        }
    }
}

void run_mode_thirteen(void)
{
    time_stop_flag = 0;
    run_mode_thirteen_start();
    run_mode_thirteen_update();
}
void run_mode_stop(void)
{

}

void run_mode_start(void)
{
    time_stop_flag = 0;

    if(ctrl_flag == 0)
    {
        if(layser_motor1_calibrate_to_base() != 0U)
        {
            ctrl_flag = 1;
            time_b = time;
        }
    }
    else if(ctrl_flag == 1)
    {
        if(zdt_motor_stop_wait(1, 0, 0, 0, 0, 0, 0, 0) == 0)
        {
            ctrl_flag = 2;
            time_b = time;
        }
    }
    else if(ctrl_flag == 2 && time > time_b + 100)
    {
        run_mode = 0;
        ctrl_flag = 0;
        time_b = time;
    }
}
