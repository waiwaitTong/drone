#ifndef DM_J4310_H
#define DM_J4310_H
#include "stm32f4xx.h"

/*Ä¬ÈÏÖµ*/
#define P_MIN -30.0f
#define P_MAX 30.0f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f



extern float position, velocity, torque;


float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void ctrl_motor(CAN_TypeDef *CANx, float _torq);
void ctrl_motor_v(CAN_TypeDef* CANx, float _vel);
void send_enable(CAN_TypeDef* CANx);
void Send_Zero(CAN_TypeDef* CANx);

void CAN_7010_SendCurrent_Single(CAN_TypeDef *CANx,u32 id,s16 current);
void CAN_7010_Open(CAN_TypeDef* CANx,u32 id);

#endif
