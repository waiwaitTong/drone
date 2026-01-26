#ifndef __MAIN_H__
#define __MAIN_H__

#include "global_declare.h"
/*--------------Types--------------*/
#include "rm_types_my.h"
/*--------------BSP--------------*/
#include "BSP_init.h"
#include "delay.h"
#include "stm32f4xx_it.h"
/*--------------API--------------*/
#include "bmi088_driver.h"
#include "dji_protocol.h"
#include "usart_protocol.h"
#include "rm_algorithm.h"
#include "encode_process.h"
#include "RefereeSys.h"
/*--------------TASK--------------*/
#include "system_monitor_task.h"
#include "gimbol_task.h"
#include "imu_update_task.h"
#include "gimbol_control_task.h"
#include "friction_wheel_task.h"
#include "supply_pellet_task.h"
#include "DR16_task.h"
#include "DM_J4310.h"
/*------------freeRTOS_init-----------*/
#include "rm_os.h"
#include "FreeRTOS.h"
////#include "OS.h"
	
#endif


