#ifndef  _GIMBAL_CONTROL_TASK_H_
#define  _GIMBAL_CONTROL_TASK_H_
#include "main.h" 
#include "global_declare.h"
#include "imu_update_task.h"
#include "rm_types_my.h"
//#include "GlobalUse_Basic_Function.h"
#include "stdlib.h"
#include "rm_algorithm.h"
#include "math.h"
#define radian                    57.29578f  //弧度转化为度 180/pi
#define MouseSST_Pitch            0.0015f     //PITCH轴鼠标灵敏度
#define MouseSST_Yaw              0.0015f     //YAW轴鼠标灵敏度  
#define RCSST_Pitch               0.0001f    //PITCH轴遥控器灵敏度
#define RCSST_Yaw                 0.0001f    //YAW轴遥控器灵敏
#define SHEPIN_Shooter            0.0002f
#define Pitch_Compensation_Step   0.006f     //手动补偿步进角度
#define Yaw_Compensation_Step     0.006f
#define Pitch_Compensation_Step_Vision   0.005f     //手动补偿步进角度
#define Yaw_Compensation_Step_Vision      0.005f

#define Pitch_Limit_Up            30.0f//14.0！！！！
#define Pitch_Limit_Down          -95.0f  //-28.0！！！
#define Yaw_Limit_Up              55.0f//57.0f
#define Yaw_Limit_Down            -55.0f//-53.0f
#define Roll_Limit_Up              5.0f//57.0f
#define Roll_Limit_Down            -5.0f//-53.0f

void GimbalControl(void);
float Angle_180_To_Inf(float angle_input, ST_ANGLE* st_angle);
float Angle_Inf_To_180(float angle);
void Gimbal_Loop(void);
void Gravity_Compensation(FP32 Angle);
void IMU_Data_Deal(void);
void Operation_Mode_Choose(void);
void Gimbal_RC_Mode(void);
void Gimbal_KeyMouse_Mode(void);
void Gimbal_VisionControl_Mode(void);
void Aim_Mode_Switch(void);
void Gimbal_ImageControl_Mode(void);

void LK_7010_DataProcess(uint8_t * CAN_RX_BUF, Motor_LK_7010 *LK_Motor);
void Abs_Encoder_Process_LK(ST_ENCODER_LK* pEncoder, s32 value);


extern _imu_st imu_data;
extern u8 AIM_mode;
extern SINT32 YawBaseCnt;
extern SINT32 PitchBaseCnt;
extern u32 d_or_s_flag;
extern u8 spinning_flag;
extern u8 change_xiangji_flag;
extern u8 Yaw_s_flag;
extern u8 Pitch_s_flag;
extern u8 Roll_s_flag;
void Aim_Mode_Choose(void);
extern FP32 Des2;
extern FP32 Roll_Current_Des;
extern FP32 Yaw_C;
extern FP32 Roll_k;
extern FP32 Pitch_C;


#endif
