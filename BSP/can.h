#ifndef __CAN_H__
#define __CAN_H__
#include "stm32f4xx.h"

typedef struct
{
	u32 Property :8;
	u32 Channel	:8;
	u32 DeviceID :8;
	u32 Priority :4;
	u32 Reserve  :1;
}ST_CAN_ID_EXT;

typedef union
{
	ST_CAN_ID_EXT stExtID;
	u32  uiExtID; 
}UN_CAN_ID_EXT;


void CAN1_Configuration(void);
void CAN2_Configuration(void);
#endif















