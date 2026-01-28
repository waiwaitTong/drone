#ifndef __RM_ROBOT_H__
#define __RM_ROBOT_H__

#include "stm32f4xx.h"

#define DEC 10
#define HEX 16

typedef unsigned char *byte_pointer;
typedef unsigned char UCHAR8;	 /* defined for unsigned 8-bits integer variable 	  无符号8位整型变量  */
typedef signed char SCHAR8;		 /* defined for signed 8-bits integer variable	  有符号8位整型变量  */
typedef unsigned short USHORT16; /* defined for unsigned 16-bits integer variable 无符号16位整型变量 */
typedef signed short SSHORT16;	 /* defined for signed 16-bits integer variable   有符号16位整型变量 */
typedef unsigned int UINT32;	 /* defined for unsigned 32-bits integer variable   无符号32位整型变量 */
typedef int SINT32;				 /* defined for signed 32-bits integer variable 	  有符号32位整型变量 */
typedef float FP32;				 /* single precision floating point variable (32bits) 单精度浮点数（32位长度）*/
typedef double DB64;			 /* double precision floating point variable (64bits) 双精度浮点数（64位长度）*/

typedef enum
{
	FALSE = 0,
	TRUE = !FALSE
} bool;

typedef struct
{
	bool GimbalTestFlag;	   // 云台测试标志
	bool SupplyPelletTestFlag; // 拨弹测试标志
	bool FrictionTestFlag;	   // 摩擦轮测试标志
} ST_TESTFLAG;

// 视觉数据处理结构体
//typedef struct
//{
//	struct
//	{
//		u8 head[4]; // 4
//		//	    u8 mode[4];             //4
//		//        float Pitchangle;       //4
//		//        float Yawangle;         //4
//		float data;
//		u8 tail[2]; // 2
//	} Receive;		// total:18

//	struct
//	{
//		u8 head[4];			// 4
//		float Pitchangle;	// 4
//		float Yawangle;		// 4
//		float bullet_speed; // 4
//		u32 mode;			// 4
//		float Pitch_e;			// 4
//		float Yaw_e;		// 4
//		float Rollangle;	// 4
//		//			float data;
//		u8 tail[2]; // 2
//	} Send;			// total:19
//} ST_VISION;

// 接收视觉数据
typedef __packed struct
{
	u8 head[2];			// 2
	UCHAR8 Is_Rcg_FLAG; // 1
	u8 LENTH;			// 1
	FP32 E_Pitch;		// 视觉使用
	FP32 Pitch_filter;
	FP32 Pitch_calculate;
	FP32 Pitch_v;
	FP32 Pitch_a;
	FP32 E_Yaw; // 视觉使用
	FP32 Yaw_filter;
	FP32 Yaw_calculate;
	FP32 Yaw_v;
	FP32 target_num; //识别锁定的号码
	FP32 Distance;
	FP32 lock; // 有没有锁定1
//	FP32 E_Roll;//4 位置可调TODOROLL 
	u8 tail[2];
} ST_ENEMY_Aim;
// 接受视觉数据联合体
typedef union
{
	ST_ENEMY_Aim stEnemyE;
	UCHAR8 ucEData[54];
} UN_AIM_DATA;

typedef struct
{
  struct
  {
        u8 head[2];             //2
    u8 id;                  //1
    u8 num;                 //1
    bool FindTargetOrNot;   //1 辅瞄时用
    bool ShootOrNot;        //1
    bool Buff_Correct_Flag; //1 大符模式用
    bool IfFindFive;        //1
    float Pitch;            //4
    float Yaw;              //4
        u8 Target;                //1
        u8 Vision_Is_Target_Top;//1
        u8 Vision_Top_Shoot;      //1
    u8 register0;      //1
    float TargetDistance;           //4
    u8 tail[2];             //2
  }Receive;                  //total:28

  struct
  {
    u8 head[2];             //2
    u8 id;                  //1
    u8 num;                 //1
    float Pitch;            //4
    float Yaw;              //4
        float Rol;              //4
    u8 Vision_Is_Close;            //1：敌人状态：哨兵，前哨，5号步兵，4号步兵，3号步兵，工程，英雄，基地
        u8 Pitch_error_Num;            //1: Pitch方向误差
        u8 Yaw_error_Num;              //1：Yaw方向误差
        u8 Time_delay_Num;            //1: 延时
        u8 BigBuff_Pitch_error_Num; //1:
        u8 BigBuff_Yaw_error_Num;    //1:
        u8 IS_Target_Top;                //1:
        u8 enemybalanceID;                //1:Enemy_Balance_ID
    float PelletSpeed;          //4
        u8 Vision_shutdown;       //1:小电脑关机标志位，默认0，关机1
        u8 other2;                //1:判断G_ST_IMU.Receive.pitch_angle == G_ST_IMU.Receive.pitch_buff_angle
    u8 tail[2];                 //2
  }Send;                         //total:32
    
} ST_VISION;

// 系统帧率检测器
typedef struct
{
	USHORT16 System_cnt;
	DB64 System_cnt_100ms;
	UINT32 System_time;
	UINT32 System_time_1s;
	USHORT16 CAN1_fps;
	USHORT16 CAN2_fps;
	USHORT16 USART1_fps;
	USHORT16 USART1_fps_100ms;
	USHORT16 USART2rx_fps;
	USHORT16 USART2tx_fps;
	USHORT16 USART3_fps;
	USHORT16 USART4rx_fps;
	USHORT16 USART4tx_fps;
	USHORT16 USART6rx_fps;
	USHORT16 SupplyPelletTask_fps;
	USHORT16 GimbalTask_fps;
	USHORT16 FrictionWheelTask_fps;
	USHORT16 IMUSampleTask_fps;
	USHORT16 IMUUpdateTask_fps;
	USHORT16 FWheelSampleTask_fps;
	USHORT16 LedTask_fps;
	USHORT16 SendCurrentTask_fps;
	USHORT16 Wireless_debugging_fps;
	USHORT16 TestTask_fps;
	USHORT16 IMUCalibrationTask_fps;

	USHORT16 CAN1_cnt;
	USHORT16 CAN2_cnt;
	USHORT16 USART1_cnt;	   // 遥控器
	USHORT16 USART1_cnt_100ms; // 遥控器(100ms)
	USHORT16 USART2rx_cnt;	   // 裁判系统
	USHORT16 USART2tx_cnt;
	USHORT16 USART3_cnt;
	USHORT16 USART4rx_cnt;
	USHORT16 USART4tx_cnt; // 视觉
	USHORT16 USART6rx_cnt;
	USHORT16 SupplyPelletTask_cnt;
	USHORT16 GimbalTask_cnt;
	USHORT16 FrictionWheelTask_cnt;
	USHORT16 IMUSampleTask_cnt;
	USHORT16 IMUUpdateTask_cnt;
	USHORT16 FWheelSampleTask_cnt;
	USHORT16 LedTask_cnt;
	USHORT16 SendCurrentTask_cnt;
	USHORT16 Wireless_debugging_cnt;
	USHORT16 IMUCalibrationTask_cnt;
	USHORT16 TaskWarningBit; // 任务警告标志位
	USHORT16 YawInit_cnt;

	USHORT16 TestTask_cnt;
	u16 Z_Offset_Debugger_fps;
	u16 Z_Offset_Debugger_cnt;

} SYSTEM_MONITOR;

// 数据联合体
typedef __packed struct
{

	__packed struct
	{
		u8 head[2];
		float rol_send;
		float pit_send;
		float yaw_send;
		FP32 Gyro_X_Speed_send;
		FP32 Gyro_Y_Speed_send;
		FP32 Gyro_Z_Speed_send;
		u8 tail;
	} Send; // total:27

	__packed struct
	{
		u8 head[2]; // 2
		float rol_receive;
		float pit_receive;
		float yaw_receive;
		FP32 Gyro_X_Speed_receive;
		FP32 Gyro_Y_Speed_receive;
		FP32 Gyro_Z_Speed_receive;
		u8 tail; // 1
	} Receive;	 // total:27
} ST_IMU;		 // total:54

// pid
typedef struct
{
	FP32 fpDes; // 控制变量目标值
	FP32 fpFB;	// 控制变量反馈值

	FP32 fpKp; // 比例系数Kp
	FP32 fpKi; // 积分系数Ki
	FP32 fpKd; // 微分系数Kd

	FP32 fpUp; // 比例输出
	FP32 fpUi; // 积分输出
	FP32 fpUd; // 微分输出

	FP32 fpE;	 // 本次偏差
	FP32 fpPreE; // 上次偏差
	FP32 fpSumE; // 总偏差
	FP32 fpU;	 // 本次PID运算结果

	FP32 fpUMax;  // PID运算后输出最大值及做遇限削弱时的上限值
	FP32 fpEpMax; // 比例项输出最大值
	FP32 fpEiMax; // 积分项输出最大值
	FP32 fpEdMax; // 微分项输出最大值16
	FP32 fpEMin;  // 积分上限
	FP32 fpKipre;// 上次i值
} ST_PID;

// TD参数结构体
typedef struct
{
	float m_x1;	 // 位置
	float m_x2;	 // 速度
	float m_x;	 // 位移
	float m_r;	 // TD阻尼因子（决定跟踪速度，r越大，跟踪速度越快，微分预测的滤残Ч?会变差）?
	float m_h;	 // TD滤波因子（算法式中的h0，h0越大微分预测的滤波效果越好）
	float m_T;	 // TD积分步长（h为步长，h越小滤波效果越好，与采样周期一致）
	float m_aim; // 目标位置
} ST_TD;

// 跟踪微分器
typedef struct
{
	FP32 x1;
	FP32 x2;
	FP32 x;
	FP32 r;
	FP32 h;
	FP32 aim;
} TD;

typedef struct
{
	SINT32 siRawValue;	  // 本次编码器的原始值
	SINT32 siPreRawValue; // 上一次编码器的原始值
	SINT32 siDiff;		  // 编码器两次原始值的差值
	SINT32 siSumValue;	  // 编码器累加值
	SINT32 siGearRatio;	  // 电机减速器减速比
	SINT32 siNumber;	  // 编码器线数
	FP32 fpSpeed;		  // 电机减速器输出轴转速，单位：r/min
} ST_ENCODER;

// 卡尔曼滤波
typedef struct
{
	FP32 x_last;	// x-2
	FP32 x_mid;		// x-1
	FP32 x_now;		// x //算法得到的数
	FP32 p_last;	// p-2
	FP32 p_mid;		// p-1
	FP32 p_now;		// p
	FP32 raw_value; // 原始数据，需要传入的参数
	FP32 K;
	FP32 R;
	FP32 Q;
} kalman_filter;

// 滑模控制
typedef struct
{
	FP32 fpDes;	 // 控制变量目标值
	FP32 fpFB;	 // 控制变量反馈值
	FP32 fpE;	 // 本次偏差
	FP32 fpU;	 // 本次运算结果
	FP32 fpUMax; // 输出上限

	// SMC参数
	FP32 b;
	FP32 eps;
	FP32 gain;
	FP32 dead;
	TD TD;
} ST_SMC;

// LADRC
typedef struct
{
	FP32 fpDes;	 // 控制变量目标值
	FP32 fpFB;	 // 控制变量反馈值
	FP32 fpE;	 // 本次偏差
	FP32 fpU;	 // 本次运算结果
	FP32 fpUMax; // 输出上限

	// LADRC参数
	FP32 b;
	FP32 w0;
	FP32 wc;
	FP32 wc1;
	FP32 LESO1;
	FP32 LESO2;
	FP32 LESO3;
	FP32 LESO1Max;
	FP32 LESO2Max;
	FP32 LESO3Max;
	FP32 delta_t;
	TD TD_LADRC;
} ST_LADRC;

// 解算陀螺仪角度到-180到180
typedef struct
{
	FP32 angle_180;
	FP32 angle_180_pre;
	FP32 angle_inf;
} ST_ANGLE;

// 陀螺仪
typedef enum
{
	OFF = 0,
	ON = 1,
	TWINKLE = 2
} LED_MODE;
typedef enum
{
	INIT = 0,
	NORMAL = 1,
	CALIBRATION = 2
} IMU_MODE;
typedef enum
{
	LOOP = 0,
	IDENTIFY = 1
} CTRL_MODE;

typedef struct
{
	float preout;
	float out;
	float in;
	float off_freq;
	float samp_tim;
} ST_LPF;

enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

typedef struct
{
	float q0; // q0;
	float q1; // q1;
	float q2; // q2;
	float q3; // q3;

	float gkp;
	float gki;

	float x_vec[VEC_XYZ];
	float y_vec[VEC_XYZ];
	float z_vec[VEC_XYZ];

	float a_acc[VEC_XYZ];
	float gacc_deadzone[VEC_XYZ];
	float gra_acc[VEC_XYZ];

	float rol;
	float pit;
	float yaw;
} _imu_st;
// 卡弹状态
typedef enum
{
	May_stop,
	Not_stop,
	Stop
} Supply_Status;

// 开不开辐瞄
typedef enum
{
	Aim_Mode = 1,
	NO_Aim_Mode = 0
} Gimbal_Control_Mode;

// 辅瞄 瞄准模式
typedef enum
{
	Aim_Zero_Mode = 0,
	Aim_Hero_Mode = 1,
	Aim_Soldier_Mode = 3,
	Aim_Soldier2_Mode = 4,
	Aim_Soldier3_Mode = 5,
	Aim_Sentinel_Mode = 7,
	Aim_Base_Mode = 8,
	Aim_Outpost_Mode = 9
} EM_AIM_MODE;

/*操作模式枚举*/
typedef enum
{
	Secuirty_Mode,		// 安全模式0
	RC_Mode,			// 遥控器模式1
	KeyMouse_Mode,		// 键鼠模式2
	SingleShoot_Mode,	// 单发打弹模式3
	SpeedShoot_Mode,	// 连发打弹模式4
	VisionControl_Mode, // 视觉测试模式5
	VisionSpeed_Mode,	//
	VisionSingle_Mode,
	ImageControl_Mode,
} EM_OPERATION_MODE;

// 无线调试
typedef struct
{
	FP32 Pitch_Angle_Des;
	FP32 Pitch_Angle_Fb;

	FP32 Yaw_Angle_Des;
	FP32 Yaw_Angle_Fb;

	FP32 five;
	FP32 six;
	FP32 seven;
	FP32 eight;
	FP32 nine;
	FP32 ten;
	FP32 eleven;
    FP32 twelve;
    FP32 Thirteen;
    FP32 Fourteen;
    FP32 Fifteen;
	u8 tail[4];
} Send_Data_Vofa;

// 某个滤波器结构体
typedef struct
{
	double raw_value;
	double xbuf[18];
	double ybuf[18];
	double filtered_value;
} Filter_t;

// 卡尔曼滤波
typedef struct
{
	float raw_value;
	float filtered_value[2];
	float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
	float P_data[4];
	float AT_data[4], HT_data[4];
	float A_data[4];
	float H_data[4];
	float Q_data[4];
	float R_data[4];
} kalman_filter_init_t;

// 串口DMA接收数据
typedef struct
{
	USART_TypeDef *USARTx;			  // 串口
	DMA_Stream_TypeDef *DMAy_Streamx; // DMA数据流
	UCHAR8 *pMailbox;				  // 邮箱(有效数据)数组
	__IO UCHAR8 *pDMAbuf;			  // DMA数组
	USHORT16 MbLen;					  // mailbox长度
	USHORT16 DMALen;				  // DMA长度
	USHORT16 rxConter;				  // 本次DMA长度
	USHORT16 rxBufferPtr;			  // 上次的长度  长度也代表位置
	USHORT16 rxSize;				  // 本次接收的长度
} USART_RX_TypeDef;

typedef struct
{
	USART_TypeDef *USARTx;
	DMA_Stream_TypeDef *DMAy_Streamx;
	UCHAR8 *pMailbox;
	__IO UCHAR8 *pDMAbuf;
	USHORT16 MbLen;
	USHORT16 DMALen;
} USART_TX_TypeDef;

// 裁判系统相关帧率
typedef struct
{
	u16 total_fps;
	u8 GameStatus_fps;
	u8 GameResult_fps;
	u8 GameRobotHP_fps;
	u8 DartStatus_fps;
	u8 EventData_fps;
	u8 SupplyProjectileAction_fps;
	u8 RefereeWarning_fps;
	u8 DartRemainingTime_fps;
	u8 GameRobotStatus_fps;
	u8 PowerHeatData_fps;
	u8 GameRobotPos_fps;
	u8 Buff_fps;
	u8 AerialRobotEnergy_fps;
	u8 RobotHurt_fps;
	u8 ShootData_fps;
	u8 BulletRemaining_fps;
	u8 RFIDStatus_fps;
	u8 DartClientCmd_fps;
	u8 RobotInteractive_fps;

	u16 total_cnt;
	u8 GameStatus_cnt;
	u8 GameResult_cnt;
	u8 GameRobotHP_cnt;
	u8 DartStatus_cnt;
	u8 EventData_cnt;
	u8 SupplyProjectileAction_cnt;
	u8 RefereeWarning_cnt;
	u8 DartRemainingTime_cnt;
	u8 GameRobotStatus_cnt;
	u8 PowerHeatData_cnt;
	u8 GameRobotPos_cnt;
	u8 Buff_cnt;
	u8 AerialRobotEnergy_cnt;
	u8 RobotHurt_cnt;
	u8 ShootData_cnt;
	u8 BulletRemaining_cnt;
	u8 RFIDStatus_cnt;
	u8 DartClientCmd_cnt;
	u8 RobotInteractive_cnt;
} ST_RSYS_MONITOR;

extern ST_RSYS_MONITOR RSYS_Monitor;

typedef struct ST_ENCODER_LK                                     
{
	uint32_t RawValue;						    //本次编码的原始值
	uint32_t PreRawValue;				        //上一次编码器的原始值
	int Diff;								//编码器两次原始量差值
	int Number;							    //编码器线数
	int SumValue;					        //编码器累加值					
}ST_ENCODER_LK;

typedef struct
{
    s32 siRawValue;                         //本次编码器的原始值
    s32 siPreRawValue;                      //上一次编码器的原始值
    s32 siDiff;                             //编码器两次原始值的差值
    s32 siSumValue;                         //编码器累加值
    FP32 siGearRatio;                       //电机减速器减速比
    s32 siNumber;                           //编码器线数
    FP32 fpSpeed;                           //电机减速器输出轴转速，单位：r/min
} LK_ST_ENCODER;

typedef struct
{
  ST_ENCODER_LK LK_motor_encoder;           //电机码盘变量
    uint32_t LK_ID;
	int16_t temp;                             //电机反馈的温度
	int16_t current;                            //电机反馈的电流
	int16_t speed;                              //电机反馈的速度
	uint16_t encoder_num;                       //电机反馈的码盘值
    float angle;				           //处理后的角度 （度）
	float anglev;				           //处理后的角速度 （度/秒）

    ST_PID LK_PID;
    float Send_Current;
    float outerTarget;				       //获取外环目标值  角度
    float outerFeedback;			       //获取外环反馈值  实时角度
    float innerFeedback;			       //获取内环反馈值  实时速度
}Motor_LK_7010;

#define USART5_RX_STREAM DMA1_Stream0
#define USART3_RX_STREAM DMA1_Stream1
#define UART4_RX_STREAM DMA1_Stream2
#define USART3_TX_STREAM DMA1_Stream3
#define UART4_TX_STREAM DMA1_Stream4
#define USART2_RX_STREAM DMA1_Stream5
#define USART2_TX_STREAM DMA1_Stream6
#define USART5_TX_STREAM DMA1_Stream7
#define UART8_TX_STREAM DMA1_Stream0
#define UART7_TX_STREAM DMA1_Stream1
#define UART7_RX_STREAM DMA1_Stream3
#define UART8_RX_STREAM DMA1_Stream6
#define USART6_RX_STREAM DMA2_Stream1
#define USART1_RX_STREAM DMA2_Stream2
#define USART6_TX_STREAM DMA2_Stream6
#define USART1_TX_STREAM DMA2_Stream7

#define USART5_RX_CHANNEL DMA_Channel_4
#define USART3_RX_CHANNEL DMA_Channel_4
#define UART4_RX_CHANNEL DMA_Channel_4
#define USART3_TX_CHANNEL DMA_Channel_4
#define UART4_TX_CHANNEL DMA_Channel_4
#define USART2_RX_CHANNEL DMA_Channel_4
#define USART2_TX_CHANNEL DMA_Channel_4
#define USART5_TX_CHANNEL DMA_Channel_4
#define UART8_TX_CHANNEL DMA_Channel_5
#define UART7_TX_CHANNEL DMA_Channel_5
#define UART7_RX_CHANNEL DMA_Channel_5
#define UART8_RX_CHANNEL DMA_Channel_5
#define USART1_TX_CHANNEL DMA_Channel_4
#define USART1_RX_CHANNEL DMA_Channel_4
#define USART6_TX_CHANNEL DMA_Channel_5
#define USART6_RX_CHANNEL DMA_Channel_5

#define USART3_DMA_TxBuf_LEN 27
#define USART3_DMA_RxBuf_LEN 27

/*串口1通信缓冲长度*/
#define USART1_RXDMA_LEN 36
#define USART1_RXMB_LEN 18

/*串口2通信缓冲长度*/
#define USART2_RXDMA_LEN 500
#define USART2_RXMB_LEN 250
#define USART2_DATA_LENTH 50

#define USART2_TXDMA_LEN 8
#define USART2_TXMB_LEN 8

/*串口3通信缓冲长度*/
#define USART3_RXDMA_LEN 9 // 温度传感器每一帧为9个bit
#define USART3_RXMB_LEN 50

#define USART3_TXDMA_LEN 64
#define USART3_TXMB_LEN 50

/*串口4通信缓冲长度*/
#define UART4_RXDMA_LEN 26//108
#define UART4_RXMB_LEN 26 // 22
#define UART4_TX_LEN 32	  // 26

/*串口6通信缓冲长度*/
#define USART6_RXDMA_LEN 500
//#define USART6_RXMB_LEN 250
#define USART6_RXMB_LEN 21
#endif
