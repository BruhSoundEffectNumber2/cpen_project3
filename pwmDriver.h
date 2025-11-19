#ifndef pwmDriver_H
#define pwmDriver_H
#include "TM4c123GH6PM.h"
#include "delay.h"
void MOT34_Init(uint16_t period, uint16_t duty);

void MOT34_Forward(void);

void MOT34_Reverse(void);
void MOT34_Speed_Set(uint16_t duty);
#endif