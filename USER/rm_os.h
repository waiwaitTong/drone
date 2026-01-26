#ifndef __RM_OS_H_
#define __RM_OS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

//声明创建的事件标志组			 
#define DECLARE_RM_OS_FLAG()\
DECLARE_OS_FLAG(IRQ);\
            
#define DECLARE_OS_FLAG(NAME)\
EventGroupHandle_t NAME##Handler

/*定义主任务的堆栈长度*/
#define StartTask_SIZE              128
#define SystemMonitorTask_SIZE      256
#define IMUUpdateTask_SIZE          128
#define SupplyPelletTask_SIZE       128
#define FrictionWheelTask_SIZE      128
#define RefereeSysTask_SIZE         128
#define Wireless_debugging_SIZE     128
#define Send_Data_Task_SIZE         128
#define IMUCalibTask_SIZE           128
#define FWheelSampleTask_SIZE       256
#define Get_bmi_datatask_SIZE       128
#define updatatask_SIZE             128
#define gombol_control_task_SIZE    128
//#define size_Task_SIZE              128

/*定义主任务的优先级*/
#define StartTask_PRIO	           10
#define IMUSampleTask_PRIO         5
#define IRQHandlerTask_PRIO        9
#define SystemMonitorTask_PRIO     8
#define IMUUpdateTask_PRIO         6
#define updatatask_PRIO            7
#define Get_bmi_datatask_PRIO      6
#define gombol_control_task_PRIO   7
#define SupplyPelletTask_PRIO      5
#define FWheelSampleTask_PRIO      5
#define Send_Data_Task_PRIO        7
#define FrictionWheelTask_PRIO     5
#define RefereeSysTask_PRIO		   4
#define Wireless_debugging_PRIO    4
#define IMUCalibTask_PRIO          5
//#define size_Task_PRIO             10
/****************************************************************************************************
宏函数名称：DECLARE_OS_TASK(NAME)
函数功能：申明操作系统任务堆栈以及操作系统的任务
入口参数：NAME，任务名称
****************************************************************************************************/
#define DECLARE_OS_TASK(NAME)\
TaskHandle_t NAME##_Handler;\
void NAME(void *pvParameters)

/****************************************************************************************************
宏函数名称：DECLARE_RM_OS_TASK(NAME)
函数功能：申明创建的任务
入口参数：NAME，任务名称
****************************************************************************************************/
#define DECLARE_RM_OS_TASK();\
DECLARE_OS_TASK(StartTask);\
DECLARE_OS_TASK(SystemMonitorTask);\
DECLARE_OS_TASK(Get_bmi_datatask);\
DECLARE_OS_TASK(updatatask);\
DECLARE_OS_TASK(FWheelSampleTask);\
DECLARE_OS_TASK(SupplyPelletTask);\
DECLARE_OS_TASK(FrictionWheelTask);\
DECLARE_OS_TASK(Send_Data_Task);\
DECLARE_OS_TASK(RefereeSysTask);\
DECLARE_OS_TASK(Wireless_debugging);\
DECLARE_OS_TASK(gombol_control_task);\
DECLARE_OS_TASK(IMUCalibTask);\
DECLARE_OS_TASK(size_Task);

/****************************************************************************************************
宏函数名称：CREATE_OS_TASK(NAME)
函数功能：根据任务名称创建操作系统任务，根据名称分配堆栈以及任务优先级
入口参数：NAME，任务名称
****************************************************************************************************/
#define CREATE_OS_TASK(NAME)\
xTaskCreate((TaskFunction_t )NAME,\
			(const char *   )"NAME",\
			(uint16_t       )NAME##_SIZE,\
			(void *         )NULL,\
			(UBaseType_t    )NAME##_PRIO,\
			(TaskHandle_t * )&NAME##_Handler) 

/****************************************************************************************************/			 
/*********************************************事件标志组*********************************************/		
/****************************************************************************************************/	
			 
/*---------------------------------------------------------------------------------------------------
宏函数名称：DECLARE_OS_FLAG(NAME)
函数功能：定义事件标志组
入口参数：NAME，定时器名称
---------------------------------------------------------------------------------------------------*/
#endif


