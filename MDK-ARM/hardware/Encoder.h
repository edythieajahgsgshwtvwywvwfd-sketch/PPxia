#ifndef BATTLECAR_ENCODER_H
#define BATTLECAR_ENCODER_H



#include <stdint.h>


void Encoder_Init(void);
int Read_Encoder(uint8_t TIMX);
void Get_Speed(void);

extern int16_t Speed_l;//编码器1（左）
extern int16_t Speed_r;//编码器2（右）



#endif //BATTLECAR_ENCODER_H