#include "APP_MAIN.h"
#include "OLED.h"
#include "jdq.h"
#include "servo.h"
#include "system.h"
#include "layser.h"


void APP_Init(void) // 应用层初始化。
{
    time_stop_flag = 0; // 允许 10 ms 系统时间正常递增。
    timer_init();
	UART_DMA_Init();
    // layser_init();  // USART3 currently disabled
    LobotServoController_Init(&huart5);
    OLED_Init();
    normal_motor_init();
    Encoder_Init();
    jdq_init();
    servo_init();
    delay_ms(100);
    moveServoAngle(1, 90, 1000);          //35
    moveServoAngle270(0, 7, 1000);        // 7 度垂直位置。
//    moveServo(0, 500, 1000);
//    servo_set_angle_270(SERVO_CH1, 20);
//    servo_set_angle_270(SERVO_CH4, 20);
    delay_ms(100);
	zdt_Pos_mode(1, 0, 0, 0);
//    motor_set_speed(Normal_motor_Speed_3, 60);
//     Motor_Set(7, -5); // 以恒定速度运行电机 7。
	delay_ms(2000);
 //   zuobiao_single_custom(8, -16 * 3200, 150, 106);
//    zuobiao_single_custom(1, 3 * 3200, 250, 206);
}

void APP_MAIN(void) // 应用层主循环。
{
    gd_detected_flag = gd_is_detected(GD_CH1);
    gd_detected_flag_2 = gd_is_detected(GD_CH2);
    gd_detected_flag_3 = gd_is_detected(GD_CH3);
    gd_detected_flag_4 = gd_is_detected(GD_CH4);


    if(system_ctrl_flag == 1)
   {
        // 光电传感器：检测到物体为 1，未检测到为 0（PB10~PB13）。
      
        test_all_task();
        allmotor(Normal_motor_Speed_1, Normal_motor_Speed_2,
            Normal_motor_Speed_3, Normal_motor_Speed_4, Normal_motor_Speed_5, Normal_motor_Speed_6,
            Normal_motor_Speed_7, Normal_motor_Speed_8);
    }
    OLED_Show();
    main_loop();
    
    // jdq_on(JDQ_CH1);
    // jdq_off(JDQ_CH2);
    //motor_set_speed(2, 30);
   // servo_set_angle_270(SERVO_CH1, 20);
}

void test_all_task(void) // 根据 run_mode 分发流程状态。
{
    switch (run_mode)
    {
    case 0:
        run_mode_zero();
        break;
    
    case 1:
        run_mode_one();
        break;

    case 2:
        run_mode_two();
        break;
    case 3:
        run_mode_three();
        break;
    case 4:
        run_mode_four();
        break;
        
    case 5:
        run_mode_five();
        break;
    case 6:
        run_mode_six();
        break;
    case 7:
        run_mode_seven();
        break;
    case 8:
        run_mode_eight();
        break;
    case 9:
        run_mode_nine();
        break;
    case 10:
        run_mode_ten();
        break;
    case 11:
        run_mode_eleven();
        break;
    case 12:
        run_mode_twelve();
        break;
    case 13:
        run_mode_thirteen();
        break;
    case 20:
        run_mode_start();
        break;
    }
    run_mode_thirteen_update();
}

// 刷新 OLED 调试信息。
void OLED_Show(void)
{
    if(OLED_Update_flag == 1)
    {
        OLED_Clear();
        OLED_ShowString(0, 0, "M3R:", OLED_6X8);
        OLED_ShowNum(30, 0, zdt_run_flag[3], 1, OLED_6X8);
        OLED_ShowString(48, 0, "STEP:", OLED_6X8);
        OLED_ShowNum(84, 0, step_ctrl_flag, 1, OLED_6X8);
        OLED_ShowString(0, 8, "TOx10:", OLED_6X8);
        OLED_ShowSignedNum(42, 8, (int32_t)(zdt_over_time[3] * 10.0f), 4, OLED_6X8);
        OLED_ShowString(0, 16, "RXN:", OLED_6X8);
        OLED_ShowNum(30, 16, RXdata_4[1].number_finall, 3, OLED_6X8);
        OLED_ShowString(54, 16, "CTL:", OLED_6X8);
        OLED_ShowNum(78, 16, ctrl_flag, 2, OLED_6X8);
        OLED_ShowString(0, 24, "B0:", OLED_6X8);
        OLED_ShowHexNum(24, 24, RXdata_4[1].Rx[0], 2, OLED_6X8);
        OLED_ShowHexNum(42, 24, RXdata_4[1].Rx[1], 2, OLED_6X8);
        OLED_ShowHexNum(60, 24, RXdata_4[1].Rx[2], 2, OLED_6X8);
        OLED_ShowHexNum(78, 24, RXdata_4[1].Rx[3], 2, OLED_6X8);
        OLED_ShowString(0, 32, "POS3:", OLED_6X8);
        OLED_ShowSignedNum(36, 32, now_pos[3], 8, OLED_6X8);
        


        OLED_Update();
        OLED_Update_flag = 0;
    }

}


