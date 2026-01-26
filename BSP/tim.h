#ifndef __TIM_H__
#define __TIM_H__
#include "stm32f4xx.h"

#define PWM1 TIM4->CCR3  //PB3 Õ¼¿Õ±È = (TIM4->CCR3 / TIM4->ARR) * 100%
#define PWM2 TIM4->CCR4  //PB4



void TIM2_Configuration(void);
void TIM3_Configuration(void);
void TIM5_Configuration(void);
void TIM4_Configuration(void);
void TIM7_Configuration(void);
void TIM9_Configuration(void);
void TIM10_Configuration(void);
//void TIM9_Configuration(void);
#endif
