#ifndef _FRICTIONWHEEL_TASK_H
#define _FRICTIONWHEEL_TASK_H


#include "rm_types_my.h"
#include "tim.h"
#include "global_declare.h"
#include "math.h"
#include "GlobalUse_Basic_Function.h"
#include "rm_algorithm.h"

#define Flameout    (UINT32)500 
#define Acc_Step    (UINT32)1     
#define friction_SMCtoPWM    (FP32) 0.005
//#define PelletSpeed_mes     ((float)Shoot_Data.bullet_speed) 
#define MAX_DATA_NUM    10
#define WINDOW_DATA_NUM 8

// 滑动窗口结构体定义
typedef struct {
    float DataList[MAX_DATA_NUM];        // 原始数据队列
    float DataList_Copy[MAX_DATA_NUM];   // 排序用副本
    int datanum;                         // 当前数据量
    float pre_datanum;
} ST_SLIDINGWINDOWS;
extern u32 pulse1,pulse2;
extern u32 sumtime1,sumtime2;
extern UCHAR8 FrictionWheel_UP_Ready;
void FrictionWheelControl(void);
void FrictionWheel_RC_Mode(void);
void FrictionWheel_KeyMouse_Mode(void);
void Friction_Direction_Calibration(void);
void intRampSignal(FP32 DesValue1, FP32 DesValue2, UINT32 Step);
void Friction_Direction_Calibration(void);
void Friction_Data_Back(void);
float SlidingWindowFilter(ST_SLIDINGWINDOWS *swf, float newdata);
void Friction_Start(void);
extern FP32 DesSpeed1;
extern FP32 DesSpeed2;
extern ST_SLIDINGWINDOWS bullet_speed_filter;

#endif
