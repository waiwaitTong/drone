#include "system_monitor_task.h"
#include "main.h"

u8 Send_Zero_Flag=0;

void System_Monitor(void)
{
	/*计算系统各个任务及通信帧率*/

	/*******************************通讯频率*********************************/
	system_monitor.USART1_fps = system_monitor.USART1_cnt;
	system_monitor.USART1_cnt = 0;
	system_monitor.USART2rx_fps = system_monitor.USART2rx_cnt;
	system_monitor.USART2rx_cnt = 0;
	system_monitor.USART2tx_fps = system_monitor.USART2tx_cnt;
	system_monitor.USART2tx_cnt = 0;
	system_monitor.USART3_fps = system_monitor.USART3_cnt;
	system_monitor.USART3_cnt = 0;
	system_monitor.USART4rx_fps = system_monitor.USART4rx_cnt;
	system_monitor.USART4rx_cnt = 0;
	system_monitor.USART6rx_fps = system_monitor.USART6rx_cnt;
	system_monitor.USART6rx_cnt = 0;
	system_monitor.USART4tx_fps = system_monitor.USART4tx_cnt;
	system_monitor.USART4tx_cnt = 0;
	system_monitor.CAN1_fps = system_monitor.CAN1_cnt;
	system_monitor.CAN1_cnt = 0;
	system_monitor.CAN2_fps = system_monitor.CAN2_cnt;
	system_monitor.CAN2_cnt = 0;
	/*******************************任务频率*********************************/
	system_monitor.FrictionWheelTask_fps = system_monitor.FrictionWheelTask_cnt;
	system_monitor.FrictionWheelTask_cnt = 0;
	system_monitor.FWheelSampleTask_fps = system_monitor.FWheelSampleTask_cnt;
	system_monitor.FWheelSampleTask_cnt = 0;
	system_monitor.SendCurrentTask_fps = system_monitor.SendCurrentTask_cnt;
	system_monitor.SendCurrentTask_cnt = 0;
	system_monitor.IMUSampleTask_fps = system_monitor.IMUSampleTask_cnt;
	system_monitor.IMUSampleTask_cnt = 0;
	system_monitor.IMUUpdateTask_fps = system_monitor.IMUUpdateTask_cnt;
	system_monitor.IMUUpdateTask_cnt = 0;
	system_monitor.SupplyPelletTask_fps = system_monitor.SupplyPelletTask_cnt;
	system_monitor.SupplyPelletTask_cnt = 0;
	system_monitor.IMUCalibrationTask_fps = system_monitor.IMUCalibrationTask_cnt;
	system_monitor.IMUCalibrationTask_cnt = 0;
	system_monitor.GimbalTask_fps = system_monitor.GimbalTask_cnt;
	system_monitor.GimbalTask_cnt = 0;
	system_monitor.Wireless_debugging_fps = system_monitor.Wireless_debugging_cnt;
	system_monitor.Wireless_debugging_cnt = 0;
    
	
	//CAN_7010_Open(CAN1,0x141);

//    send_enable(CAN1);

//    if(Send_Zero_Flag==1)
//    {
//        Send_Zero(CAN1);
//        Send_Zero_Flag=0;
//    
//    }
}

void Safemode(void)
{

	Send_Current_To_Gimbal(CAN1, 0, 0);

	pitch_td.aim = Pitch_PosPID.fpFB;
	yaw_td.aim = Yaw_PosPID.fpFB;
	PitchPos_Reference = Pitch_PosPID.fpFB;
	YawPos_Reference = Yaw_PosPID.fpFB;
	Pitch_PosPID.fpDes = Pitch_PosPID.fpFB;
	Yaw_PosPID.fpDes = Yaw_PosPID.fpFB;
	RollPos_Reference = Roll_PosPID.fpFB;
	Roll_PosPID.fpDes = Roll_PosPID.fpFB;
	
	FrictionWheel_Ready_cnt = 0;
	FrictionWheel_Ready = FALSE;

	PWM1 = 500;
	PWM2 = 500;

	Supply_PosPID.fpDes = Supply_PosPID.fpFB;
	Supply_SpeedPID.fpU = 0;

	Send_Current_To_SupplyPelletMotor(CAN2, 0);

	PitchCurrent = 0;
	YawCurrent = 0;
	RollCurrent = 0;
	FrictionWheel_UP_Ready = 0;
	Yaw_PosPID.fpDes = Yaw_PosPID.fpFB;
	Yaw_SpeedPID.fpDes = Yaw_SpeedPID.fpFB;
	Pitch_PosPID.fpDes = Pitch_PosPID.fpFB;
	Pitch_SpeedPID.fpDes = Pitch_SpeedPID.fpFB;
	Pitch_PosPID.fpSumE = Pitch_PosPID.fpE = Pitch_PosPID.fpPreE = 0;
	Pitch_SpeedPID.fpE = Pitch_SpeedPID.fpPreE = Pitch_SpeedPID.fpSumE = Pitch_SpeedPID.fpU = 0;
	Yaw_PosPID.fpSumE = Yaw_PosPID.fpE = Yaw_PosPID.fpPreE = 0;
	Yaw_SpeedPID.fpSumE = Yaw_SpeedPID.fpE = Yaw_SpeedPID.fpPreE = Yaw_SpeedPID.fpU = 0;
	Yaw_SpeedPID.fpU = 0;
	Pitch_SpeedPID.fpU = 0;
	Roll_PosPID.fpSumE = Roll_PosPID.fpE =Roll_PosPID.fpPreE = 0;
	Roll_SpeedPID.fpSumE = Roll_SpeedPID.fpE = Roll_SpeedPID.fpPreE = Roll_SpeedPID.fpU = 0;
	Roll_SpeedPID.fpU = 0;
    Roll_Current_Des=0;

	Gimbal_Control = NO_Aim_Mode;
}

/*标定*/
float flagZ60 = 0;	 // 60s计数器
int flagZ = 0;		 // z_offset_debugger计数器
int flag_updown = 0; // 判断升降
int flag_delta = 0;	 // 判断斜率
int FINISH = 0;		 // 60s判断
int aaa = 0;

float q0Last = 0;
float q1Last = 0;
float q2Last = 0;
float q3Last = 0;

float yaw_Last = 0;
float delta_yaw = 0;
float delta_yaw_watch = 0;

float lingdianyi = 0.3f;
float fulingdianyi = -0.3f;

int bmireset = 0;
int flag_q = 0;
int timezoff = 0;
int a = 0;
float ZWATCH = 0;

extern FP32 Gyro_Z_Offset;

void Z_Offset_Debugger(void)
{

	if (FINISH)
	{
		if (a == 0)
		{
			a = 1;
		}
	}
	else
	{
		if (flag_updown == 1)
		{
			if (flag_delta == 1)
			{

				Gyro_Z_Offset += 0.00001f;
				if (bmireset > 10)
				{
					yaw_Last = 0;
					bmireset++;
					flag_q = 1;
				}
				ZWATCH = Gyro_Z_Offset * 1000000;
			}
			else if (flag_delta == -1)
			{
				yaw_Last = 0;
				bmireset++;
				flag_q = 1;
				// Send_offset();
			}
		}
		else if (flag_updown == -1)
		{
			if (flag_delta == 1)
			{
				yaw_Last = 0;
				flag_q = 1;
				bmireset++;
				// Send_offset();
			}
			else if (flag_delta == -1)
			{

				Gyro_Z_Offset -= 0.00001f;
				if (bmireset > 10)
				{
					yaw_Last = 0;
					bmireset++;
					flag_q = 1;
				}
				ZWATCH = Gyro_Z_Offset * 1000000;
			}
		}
	}
	system_monitor.Z_Offset_Debugger_cnt++;

	timezoff++;
}
