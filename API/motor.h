#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f4xx.h"
#include "rm_types_my.h"
#include "global_declare.h"

void Send_Current_To_SupplyPelletMotor(CAN_TypeDef* CANx, SSHORT16 supply_i);
void Send_Current_To_Gimbal(CAN_TypeDef *CANx,SSHORT16 gmpitch_v,SSHORT16 gmyaw_i);
void RC_Protocol(void);
void Change_Control_State_Deal(void);
SINT32 Get_Encoder_Number(CanRxMsg* rx_message);
SINT32 Get_Speed(CanRxMsg* rx_message);
SINT32 Get_Torque(CanRxMsg* rx_message);
void Send_Current_To_Pitch(CAN_TypeDef* CANx, SSHORT16 supply_i);
void Send_Voltage_To_Pitch(CAN_TypeDef* CANx, SSHORT16 voltage);

#endif