#ifndef _GLOBAL_DECLARE_H_
#define _GLOBAL_DECLARE_H_
#include "stm32f4xx.h"
#include "rm_types_my.h"
#include "rm_rc_types.h"
#include "Rc_RsysData.h"

	
extern SYSTEM_MONITOR system_monitor; //系统监视器
extern ST_TESTFLAG g_stTestFlag;


extern u8 BMI088_Cali_Flag ;
extern u8 Manual_flag;
extern ST_IMU  DNGimbalDataBuf;
extern FP32 BMIPitchSpeed; 
extern FP32 BMIYawSpeed;
extern FP32 Visionrx_freq_data_p;
extern ST_PID Pitch_PosPID;
extern ST_PID Pitch_SpeedPID;
extern ST_PID g_stGM_PitchSpeedPID_Move;
extern ST_PID Yaw_PosPID;
extern ST_PID Yaw_SpeedPID;
extern ST_PID Roll_PosPID;
extern ST_PID Roll_SpeedPID;

extern ST_PID Pitch_PosPID_1;
extern ST_PID Pitch_SpeedPID_1;
extern ST_PID Yaw_PosPID_1;
extern ST_PID Yaw_SpeedPID_1;
extern TD yaw_td_1;
extern TD pitch_td_1;

extern ST_PID Pitch_PosPID_2;
extern ST_PID Pitch_SpeedPID_2;
extern ST_PID Yaw_PosPID_2;
extern ST_PID Yaw_SpeedPID_2;
extern TD yaw_td_2;
extern TD pitch_td_2;

extern ST_PID Pitch_PosPID_3;
extern ST_PID Pitch_SpeedPID_3;
extern ST_PID Yaw_PosPID_3;
extern ST_PID Yaw_SpeedPID_3;
extern TD yaw_td_3;
extern TD pitch_td_3;

extern kalman_filter pitch_kf;
extern kalman_filter yaw_kf;
extern kalman_filter yawfb_kf;
extern kalman_filter pitchfb_kf;
extern kalman_filter rollfb_kf;
extern kalman_filter pitch_speed_kf;
extern kalman_filter yaw_speed_kf;
extern ST_LPF pitch_speed_lpf;

extern ST_LPF pitch_pos_lpf;


extern float yaw_fileter;
extern float pitch_fileter;
extern float roll_fileter;
extern float Pitch_E;
extern float Yaw_E;


extern FP32 GravityCompensation;
extern FP32 PitchPos_Reference;
extern FP32 YawPos_Reference;
extern FP32 RollPos_Reference;
extern  s16 YawCurrent ;
extern s16 PitchCurrent   ;
extern FP32 RollCurrent   ;
extern TD yaw_td;
extern TD pitch_td;
extern TD pitch_speed_td;

extern bool FrictionWheel_ReadyFLAG;
extern bool FrictionWheel_Ready;
extern u16 FrictionWheel_Ready_cnt;

extern FP32 Pitch_Limit_Up_imu;
extern FP32 Pitch_Limit_Down_imu;
extern FP32 Yaw_Limit_Up_imu;
extern FP32 Yaw_Limit_Down_imu;
extern FP32 Roll_Limit_Up_imu;
extern FP32 Roll_Limit_Down_imu;

extern FP32 Pitch_encode;
extern FP32 Yaw_encode;
extern FP32 Roll_encode;

extern FP32 Pitch_current;
extern FP32 Yaw_current;
extern FP32 Roll_current;

extern volatile ST_ENCODER g_stShooterEncoder;
extern volatile ST_ENCODER g_Encoder_1SMC;
extern volatile ST_ENCODER g_Encoder_2SMC;
extern ST_SMC g_stFriction1SMC;
extern ST_SMC g_stFriction2SMC;
extern ST_PID Supply_PosPID;
extern ST_PID Supply_SpeedPID;
extern Supply_Status Supply_Motor_Status;

extern ST_VISION G_ST_Vision;
extern UN_AIM_DATA     unAimData;

extern bool PRESSED_W                  ;
extern bool PRESSED_S                  ;
extern bool PRESSED_A                  ;
extern bool PRESSED_D                  ;
extern bool PRESSED_SHIFT              ;
extern bool PRESSED_CTRL               ;
extern bool PRESSED_Q                  ;
extern bool PRESSED_E                  ;
extern bool PRESSED_R                  ;
extern bool PRESSED_F                  ;
extern bool PRESSED_G                  ;
extern bool PRESSED_Z                  ;
extern bool PRESSED_X                  ;
extern bool PRESSED_C                  ;
extern bool PRESSED_V                  ;
extern bool PRESSED_B                  ;
extern bool PRESSED_ML                 ;
extern bool PRESSED_MR                 ;


extern DBUS_BUFF Mouse_X;
extern DBUS_BUFF Mouse_Y;

/*---------------------------------裁判系统---------------------------------------------------*/
extern ST_RSYS_MONITOR RSYS_Monitor ;
extern ext_shoot_data_t          Shoot_Data         ;  //射击状态
extern ext_game_robot_pos_t      Drone_Pos          ;  //空间位置
extern ext_power_heat_data_t     Shoot_Power        ;  //热量状态
extern ext_game_robot_state_t    Drone_State        ;  //发射机构供电状态 友方阵营
extern UN_Custom_Robo            Custom_Robo_Data   ;  //机器人自定义交互数据  代码里没用到
extern UN_Custom_Client          Custom_Client_Data ;  //客户端显示数据        代码里没用到
extern ext_bullet_remaining_t    Ammo_remain        ;  //剩余子弹数   ID:0208
extern ext_game_robot_HP_t       Robot_HP           ;  //机器人血量数据  0x0003
extern ext_game_status_t         Game_Status        ;  //比赛状态    0x0001;
extern ext_student_interactive_header_data_t		 G_ST_Student_Interactive_Header_Data; //交互数据接收信息
extern robot_interactive_data_t					 G_ST_Robot_Interactive_Data; //1.交互数据
extern ext_client_custom_graphic_delete_t			 G_ST_Client_Custom_Graphic_Delete; //2.客户端删除图形
extern ext_client_custom_graphic_single_t			 G_ST_Client_Custom_Graphic_Single; //3.客户端绘制一个图形
extern ext_client_custom_graphic_double_t			 G_ST_Client_Custom_Graphic_Double; //4.客户端绘制二个图形
extern ext_client_custom_graphic_five_t		     G_ST_Client_Custom_Graphic_Five; //5.客户端绘制五个图形
extern ext_client_custom_graphic_seven_t			 G_ST_Client_Custom_Graphic_Seven; //6.客户端绘制七个图形
extern ext_client_custom_graphic_single_t      G_ST_Client_Custom_Graphic_Single_Rectangle;//客户端绘制矩形
extern ext_client_custom_character_t                G_ST_Client_Custom_Character;
extern u8 SendData_Sentinel[3];

extern ext_client_custom_character_t				 G_ST_Client_Custom_Character; //7.客户端绘制字符
extern ext_missile_data_t        DataFromMissile    ;   //来自导弹的数据
extern ext_sentinel_data_t       DataFromSentinel   ;   //来自哨兵的数据
extern ext_ground_data_t        DataFromSoldier3    ;   //来自步兵3的数据
extern ext_ground_data_t        DataFromSoldier4    ;   //来自步兵4的数据
extern ext_ground_data_t        DataFromSoldier5    ;   //来自步兵5的数据
extern ext_ground_data_t        DataFromHero        ;   //来自英雄的数据

extern Gimbal_Control_Mode Gimbal_Control;
extern UCHAR8   Sending_num;
extern UCHAR8   picture_num ;
extern EM_AIM_MODE   g_emAim_Mode;
extern UCHAR8 sentinel_change_id;

extern Send_Data_Vofa Send_vofa;
//extern remote_control_t     remote_control_image_transmission;
extern remote_control_keyboard_mouset_t remote_control_image_transmission;


/*-----------------------------------步兵号码选择----------------------------------------------*/
extern int infantry3_flag;
extern int infantry4_flag;
extern int infantry5_flag;
extern u8 vision_lock ;
extern int infantry_number;

/*-----------------------------------瓴控电机----------------------------------------------*/
extern Motor_LK_7010 ARM_LK_Motor1;
extern Motor_LK_7010 ARM_LK_Motor2;

#endif
