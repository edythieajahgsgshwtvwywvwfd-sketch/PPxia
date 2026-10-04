#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>

void timer_init(void);
void zdt_over_time_jc(void); // 张大头步进电机运行超时检测。
uint8_t zdt_motor_stop_wait(uint8_t motor_1, uint8_t motor_2, uint8_t motor_3, uint8_t motor_4, 
                           uint8_t motor_5, uint8_t motor_6, uint8_t motor_7, uint8_t motor_8);
void zuobiao_group1(int32_t motor_pos_1, int32_t motor_pos_2, int32_t motor_pos_3, int32_t motor_pos_4);
void zuobiao_group2(int32_t motor_pos_5, int32_t motor_pos_6, int32_t motor_pos_7, int32_t motor_pos_8);
void zuobiao_single(uint8_t motor_id, int32_t motor_pos);
void zuobiao_single_custom(uint8_t motor_id, int32_t motor_pos, int16_t vel, uint8_t acc);
void zuobiao_single_relative_custom(uint8_t motor_id, int32_t delta_pulse, int16_t vel, uint8_t acc);
void zuobiao(int32_t motor_pos_1, int32_t motor_pos_2, int32_t motor_pos_3, int32_t motor_pos_4,
             int32_t motor_pos_5, int32_t motor_pos_6, int32_t motor_pos_7, int32_t motor_pos_8);
void zdt_stop_read(void);
#endif



