#include "gimbol_control_task.h"

u8 AIM_mode = 0;
FP32 k = 0;						  // 重力补偿系数3650  8000可以保证0度以下停住7000
FP32 kc = 0;					  // 重力补偿前馈常数
FP32 Angle0 = 9, Angle1 = 0;	  // 记得改一下，之前pitch_zero改了 -0.83
FP32 Des = 5, Des1 = 0, Des2 = 0; // 调试pid时debug里赋值的目标值
FP32 Roll_Current_Des = 0;
FP32 Roll_k=-0.2;
u8 pitch_flag = 0;				 // 分段式pid中的标志
FP32 Pitch_k = 0, Yaw_k = 0; // td参数  
FP32 Roll_c = 0, Roll_vc = 2;	 // roll补偿
/*标志位*/
u8 Yaw_imu_flag = 1, Pitch_imu_flag = 1, Roll_imu_flag = 1; // 反馈值获取方式标志，0为can，1为imu
u8 Yaw_s_flag = 1, Pitch_s_flag = 1, Roll_s_flag = 1;
u8 Yaw_kf_flag =0, Pitch_kf_flag = 0, Roll_kf_flag = 0;

u8 test_flag = 0; // 测试模式标志
int fb_kalman_flag = 1;
extern int fb_kalman_flag;
int cnt_automatic = 0;
extern int cnt_automatic;
int automatic = 0;
int automatic1 = 0;
int automatic2 = 0;

extern int automatic;

int test_current = 1;
int td_flag = 1;
int two_flag = 0; // pid分段控制标志

int Force_flag = 0;
FP32 Force_d = 0;
FP32 Error1 = 0;
FP32 Error2 = 0;
FP32 Error3 = 0;

int DM_Zero = 0;
int DM_Zero_cnt = 0;

FP32 Yaw_C = 0; // 打弹抖动补偿
FP32 Pitch_C = 0; 
FP32 Friction_PreFB = 0;
FP32 Friction_FB = 0;

// 云台控制主函数
void GimbalControl(void)
{

	if (g_stTestFlag.GimbalTestFlag == FALSE)
	{
		if (RC_ON)
		{
			switch (g_emOperation_Mode)
			{
			case Secuirty_Mode:
				Safemode();
				break;
			case RC_Mode:
				Gimbal_Control = NO_Aim_Mode;
				Gimbal_RC_Mode();
				break;
			case KeyMouse_Mode:
				Gimbal_KeyMouse_Mode();
				break;
			case VisionControl_Mode:
				Gimbal_VisionControl_Mode();
				break;
			case VisionSpeed_Mode:
				Gimbal_VisionControl_Mode();
				break;
			case VisionSingle_Mode:
				Gimbal_VisionControl_Mode();
				break;
			case ImageControl_Mode:
				Gimbal_ImageControl_Mode();
				break;
			case SingleShoot_Mode:
				Gimbal_Control = NO_Aim_Mode;
				Gimbal_RC_Mode();
				break;
			case SpeedShoot_Mode:
				Gimbal_Control = NO_Aim_Mode;
				Gimbal_RC_Mode();
				break;

			default:
				break;
			}
			Gimbal_Loop();
			if (g_emOperation_Mode == Secuirty_Mode) // 不添加的话由于先进安全模式再进运算，导致原先赋值为0的数重新赋值导致发送电流不为0
			{
				PitchCurrent = 0;
				YawCurrent = 0;
				RollCurrent = 0;
                Roll_Current_Des=0;
			}

			if (test_current == 1)
			{
				Send_Current_To_Gimbal(CAN1, 0, YawCurrent);
				//
				CAN_7010_SendCurrent_Single(CAN1,0x141,PitchCurrent);
							//	CAN_7010_SendCurrent_Single(CAN1,0x141,0);


//				ctrl_motor(CAN1, RollCurrent);
			}
			if (test_current == 0)
			{
				Send_Current_To_Gimbal(CAN1, 0, 0);
				ctrl_motor(CAN1, 0);
			}
		}
		else
		{
			Pitch_PosPID.fpSumE = 0, Pitch_SpeedPID.fpSumE = 0, Roll_SpeedPID.fpSumE = 0;
			Yaw_PosPID.fpSumE = 0, Yaw_SpeedPID.fpSumE = 0, Roll_SpeedPID.fpSumE = 0; // 重置PID累积误差
			Send_Current_To_Gimbal(CAN1, 0, 0);
			ctrl_motor(CAN1, RollCurrent);
		}
	}

	else if (g_stTestFlag.GimbalTestFlag == TRUE)
	{
		if (RC_ON) // 目标值在debug里给
		{

			if (automatic == 1)
			{
				cnt_automatic++;

				if (cnt_automatic == 1)
					Des += 2;
				if (cnt_automatic == 1000)
					Des += 2;
				if (cnt_automatic == 2000)
					Des += 2;
				if (cnt_automatic == 3000)
					Des += 2;
					if (cnt_automatic ==4000 )
					Des -= 2;
				if (cnt_automatic == 5000)
					Des -= 2;
				if (cnt_automatic == 6000)
					Des -= 2;
				if (cnt_automatic == 7000)
					Des -= 2;
		
				if (cnt_automatic == 8000)
					cnt_automatic = 0;
			}
			if (automatic1 == 1)
			{
				cnt_automatic++;

				if (cnt_automatic == 1)
					Des1 += 3;
				if (cnt_automatic == 1000)
					Des1 += 3;
				if (cnt_automatic == 2000)
					Des1 -= 3;
				if (cnt_automatic == 3000)
					Des1 -= 3;
				if (cnt_automatic == 4000)
					cnt_automatic = 0;
			}
			if (automatic2 == 1)
			{
				cnt_automatic++;

				if (cnt_automatic == 1)
					Des2 += 2;
				if (cnt_automatic == 1000)
					Des2 += 2;
				if (cnt_automatic == 2000)
					Des2 -= 2;
				if (cnt_automatic == 3000)
					Des2 -= 2;
				if (cnt_automatic == 4000)
					cnt_automatic = 0;
			}
			if (Pitch_imu_flag == 0)
			{
				Des = Clip(Des, Pitch_Limit_Down, Pitch_Limit_Up);
			}
			else
			{
				Des = Clip(Des, Pitch_Limit_Down_imu, Pitch_Limit_Up_imu);
			}
			if (Yaw_imu_flag == 0)
			{
				Des1 = Clip(Des1, Yaw_Limit_Down, Yaw_Limit_Up);
			}
			else
			{
				Des1 = Clip(Des1, Yaw_Limit_Down_imu, Yaw_Limit_Up_imu);
			}
			if (Roll_imu_flag == 0)
			{
				Des2 = Clip(Des2, Roll_Limit_Down, Roll_Limit_Up);
			}
			else
			{
				Des2 = Clip(Des2, Roll_Limit_Down_imu, Roll_Limit_Up_imu);
			}

			if (td_flag == 1)
			{
				pitch_td.aim = Des;
				TD_Function(&pitch_td);
				Pitch_PosPID.fpDes = pitch_td.x1;
				yaw_td.aim = Des1;
				TD_Function(&yaw_td);
				Yaw_PosPID.fpDes = yaw_td.x1;

				Roll_PosPID.fpDes = Des2;
			}

			if (td_flag == 0)
			{
				pitch_kf.raw_value = Des;
				yaw_kf.raw_value = Des1;
				//	}

				Kalman_Filter(&yaw_kf);
				yaw_td.aim = yaw_kf.x_now;
				TD_Function(&yaw_td);
				Yaw_PosPID.fpDes = yaw_kf.x_now;
				//  Yaw_PosPID.fpDes = yaw_td.x1;

				Kalman_Filter(&pitch_kf);
				pitch_td.aim = pitch_kf.x_now;
				TD_Function(&pitch_td);
				Pitch_PosPID.fpDes = pitch_kf.x_now;

				Roll_PosPID.fpDes = Des2;
				//  Pitch_PosPID.fpDes = pitch_td.x1;
			}
			Gimbal_Loop();
			if (g_emOperation_Mode == Secuirty_Mode) // 不添加的话由于先进安全模式再进运算，导致原先赋值为0的数重新赋值导致发送电流不为0
			{
				PitchCurrent = 0;
				YawCurrent = 0;
				RollCurrent = 0;
                Roll_Current_Des=0;

			}
			Send_Current_To_Gimbal(CAN1, 0, YawCurrent);
			//
			CAN_7010_SendCurrent_Single(CAN1,0x141,PitchCurrent);
			//	CAN_7010_SendCurrent_Single(CAN1,0x141,0);

			ctrl_motor(CAN1, RollCurrent);
		}
		else
		{
			Send_Current_To_Gimbal(CAN1, 0, 0);
			ctrl_motor(CAN1, RollCurrent);
		}
	}
	system_monitor.GimbalTask_cnt++;

	if (DM_Zero == 1)
	{
		if (abs_fl(imu_data.rol) <= 0.1f)
		{
			DM_Zero_cnt++;
		}
	}
	if (DM_Zero_cnt == 2000)
	{
		Send_Zero(CAN1);
		DM_Zero_cnt = 0;
		DM_Zero = 0;
	}
}

int hyst_flag = 0;
int hyst_flag_1 = 0;
int hyst_flag_2 = 0;

// 云台pid算法控制函数
void Gimbal_Loop(void)
{
//	if (two_flag == 1)
//	{
//		/*yaw*/
//		if ((((Absolute_value(Yaw_PosPID.fpFB - Yaw_PosPID.fpDes) > 3.0f) || (Absolute_value(Yaw_PosPID.fpFB - yaw_td.aim) > 3.0f) || (Absolute_value(Yaw_PosPID.fpFB - yaw_kf.raw_value) > 3.0f)) && hyst_flag == 1))
//		{
//			Yaw_SpeedPID.fpKp = Yaw_SpeedPID_3.fpKp;
//			Yaw_PosPID.fpKp = Yaw_PosPID_3.fpKp;
//			Yaw_PosPID.fpKi = Yaw_PosPID_3.fpKi;
//			yaw_td.r = yaw_td_3.r;

//			hyst_flag = 0;
//		}

//		else if ((((Absolute_value(Yaw_PosPID.fpFB - Yaw_PosPID.fpDes) < 1.0f) || (Absolute_value(Yaw_PosPID.fpFB - yaw_td.aim) < 1.0f) || (Absolute_value(Yaw_PosPID.fpFB - yaw_kf.raw_value) < 1.0f)) && hyst_flag == 0))

//		{

//			Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp;
//			Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp;
//			Yaw_PosPID.fpKi = Yaw_PosPID_1.fpKi;
//			yaw_td.r = yaw_td_1.r;

//			hyst_flag = 1;
//		}

//		/*pitch*/
//		if ((((Absolute_value(Pitch_PosPID.fpFB - Pitch_PosPID.fpDes) > 3.0f) || (Absolute_value(Pitch_PosPID.fpFB - pitch_td.aim) > 3.0f) || (Absolute_value(Pitch_PosPID.fpFB - pitch_kf.raw_value) > 3.0f)) && hyst_flag_1 == 1))
//		{
//			Pitch_SpeedPID.fpKp = Pitch_SpeedPID_3.fpKp;
//			Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp;
//			Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
//			pitch_td.r = pitch_td_3.r;
//			hyst_flag_1 = 0;
//		}

//		else if ((((Absolute_value(Pitch_PosPID.fpFB - Pitch_PosPID.fpDes) < 1.0f) || (Absolute_value(Pitch_PosPID.fpFB - pitch_td.aim) < 1.0f) || (Absolute_value(Pitch_PosPID.fpFB - pitch_kf.raw_value) < 1.0f)) && hyst_flag_1 == 0))

//		{

//			Pitch_SpeedPID.fpKp = Pitch_SpeedPID_1.fpKp;
//			Pitch_PosPID.fpKp = Pitch_PosPID_1.fpKp;
//			Pitch_PosPID.fpKi = Pitch_PosPID_1.fpKi;
//			pitch_td.r = pitch_td_1.r;
//			hyst_flag_1 = 1;
//		}
//	}
//Pitch_encode>21.6


//	if (Pitch_encode > 14 ) // 20-40
//	{
////		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp + (Yaw_SpeedPID_2.fpKp - Yaw_SpeedPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd + (Yaw_SpeedPID_2.fpKd - Yaw_SpeedPID_1.fpKd) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp + (Yaw_PosPID_2.fpKp - Yaw_PosPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
////		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd;
////		yaw_td.r = yaw_td_2.r;
//		
//		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_2.fpKp;
//		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_2.fpKd;
//		Yaw_PosPID.fpKp = Yaw_PosPID_2.fpKp;
//		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
//		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd;
//		yaw_td.r = yaw_td_2.r;
//	}
//		
////		
////		
////	}
////	else if( Pitch_encode > 20 && Yaw_encode > -45 && Yaw_encode<-10)
////	{
////		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_3.fpKp;
////		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_3.fpKd;
////		Yaw_PosPID.fpKp = Yaw_PosPID_3.fpKp;
////		Yaw_PosPID.fpKi = Yaw_PosPID_3.fpKi;
////		Yaw_PosPID.fpKd = Yaw_PosPID_3.fpKd;
////		yaw_td.r = yaw_td_3.r;
//////		
////	}
//	else
//	{
//		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp;
//		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd;
//		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp;
//		Yaw_PosPID.fpKi = Yaw_PosPID_1.fpKi;
//		Yaw_PosPID.fpKd = Yaw_PosPID_1.fpKd;
//		yaw_td.r = yaw_td_1.r;
//	}
	
	
//	
//  ////////////////////////////////////////////////////////////5.15	
//////	
//	if (Yaw_encode > 12.0f&&Yaw_encode<68.0f) // 20-40
//	{
////		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp + (Yaw_SpeedPID_2.fpKp - Yaw_SpeedPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd + (Yaw_SpeedPID_2.fpKd - Yaw_SpeedPID_1.fpKd) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp + (Yaw_PosPID_2.fpKp - Yaw_PosPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
////		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd;
////		yaw_td.r = yaw_td_2.r;
//		
//		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_3.fpKp;
//		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_3.fpKd;
//		Yaw_PosPID.fpKp = Yaw_PosPID_3.fpKp;
//		Yaw_PosPID.fpKi = Yaw_PosPID_3.fpKi;
//		Yaw_PosPID.fpKd = Yaw_PosPID_3.fpKd;
//		yaw_td.r = yaw_td_3.r;
//		
//		
////			if (Pitch_encode > 12 ) // 0-16 ：    30 120     《0：35   130     16-20:30 100    20-28:30 100 10 110  28-:28 100 10 100
////	{
////		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp + (Yaw_SpeedPID_2.fpKp - Yaw_SpeedPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd + (Yaw_SpeedPID_2.fpKd - Yaw_SpeedPID_1.fpKd) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp + (Yaw_PosPID_2.fpKp - Yaw_PosPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
////		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd;
////		yaw_td.r = yaw_td_2.r;
////		
////		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_2.fpKp;
////		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_2.fpKd;
////		Yaw_PosPID.fpKp = Yaw_PosPID_2.fpKp;
////		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
////		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd;
////		yaw_td.r = yaw_td_2.r;
////	}
//////////////////////////////////////////////////7.5
//					if (ARM_LK_Motor2.angle < 2 ) // 20-40
//	{
//	
//		Pitch_SpeedPID.fpKp = Pitch_SpeedPID_1.fpKp;
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_1.fpKd;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_1.fpKp;
//		Pitch_PosPID.fpKi = Pitch_PosPID_1.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_1.fpKd;
//		
//		pitch_td.r = pitch_td_1.r;
//	}
//	
//						if (ARM_LK_Motor2.angle >= 2 && ARM_LK_Motor2.angle < 16) // 20-40
//	{
//	
//		Pitch_SpeedPID.fpKp = Pitch_SpeedPID_1.fpKp;
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_1.fpKd;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_1.fpKp-8;
//		Pitch_PosPID.fpKi = Pitch_PosPID_1.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_1.fpKd-20;
//		
//		pitch_td.r = pitch_td_1.r;
//	}
//	
//							if (ARM_LK_Motor2.angle >= 16 && ARM_LK_Motor2.angle < 15) // 20-40
//	{
//	
//		Pitch_SpeedPID.fpKp = Pitch_SpeedPID_1.fpKp;
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_1.fpKd-10;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_1.fpKp-8;
//		Pitch_PosPID.fpKi = Pitch_PosPID_1.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_1.fpKd-20;
//		
//		pitch_td.r = pitch_td_1.r;
//	}
//	
//							if (ARM_LK_Motor2.angle >= 15 && ARM_LK_Motor2.angle < 60) // 20-40
//	{
//	
//		Pitch_SpeedPID.fpKp = Pitch_SpeedPID_3.fpKp;
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_3.fpKd;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp;
//		Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_3.fpKd;
//		
//		pitch_td.r = pitch_td_2.r;
//		
//		if(Yaw_encode>28&&Yaw_encode<60)
//		{
//			Pitch_SpeedPID.fpKp = Pitch_SpeedPID_3.fpKp-2;
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_3.fpKd-10;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp-2;
//		Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_3.fpKd-10;
//		
//		pitch_td.r = pitch_td_2.r;
//		}
//		if(Yaw_encode>-25&&Yaw_encode<28)
//		{
//			Pitch_SpeedPID.fpKp = Pitch_SpeedPID_3.fpKp-2;
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_3.fpKd-10;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp-2;
//		Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_3.fpKd-10;
//		
//		pitch_td.r = pitch_td_2.r;
//		}
		
//		if(Yaw_encode>-50&&Yaw_encode<=-25)
//		{
////			Pitch_SpeedPID.fpKp = Pitch_SpeedPID_3.fpKp-5;
////		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_3.fpKd-50;
////		
////		Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp-13;
////		Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
////		Pitch_PosPID.fpKd = Pitch_PosPID_3.fpKd-50;
////		
////		pitch_td.r = pitch_td_2.r;
////			
////			Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp-5;
////			Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd-50;
////			Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp-5;
////			Yaw_PosPID.fpKi = Yaw_PosPID_1.fpKi;
////			Yaw_PosPID.fpKd = Yaw_PosPID_1.fpKd-50;
////			yaw_td.r = yaw_td_2.r;
//		Pitch_SpeedPID.fpKp = Pitch_SpeedPID_3.fpKp-4;
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_3.fpKd-20;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp-4;
//		Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_3.fpKd-20;
//		
//		pitch_td.r = pitch_td_2.r;
//			
//			Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp;
//			Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd;
//			Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp;
//			Yaw_PosPID.fpKi = Yaw_PosPID_1.fpKi;
//			Yaw_PosPID.fpKd = Yaw_PosPID_1.fpKd;
//			yaw_td.r = yaw_td_2.r;
//		
//	}
	
//}
	
//	pit>21   yaw>28
	if(ARM_LK_Motor2.angle>-25 && ARM_LK_Motor2.angle<=30 )
	{
		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp ;
		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd;
		
		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp;
		Yaw_PosPID.fpKi = Yaw_PosPID_1.fpKi;
		Yaw_PosPID.fpKd = Yaw_PosPID_1.fpKd;
		
		yaw_td.r = yaw_td_1.r;
		
	}
		
		if (ARM_LK_Motor2.angle>30 && ARM_LK_Motor2.angle<65 ) // 20-40
	{
		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp ;
		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd *cos(ARM_LK_Motor2.angle/57.3f);
		
		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp;
		Yaw_PosPID.fpKi = Yaw_PosPID_1.fpKi;
		Yaw_PosPID.fpKd = Yaw_PosPID_1.fpKd*cos(ARM_LK_Motor2.angle/57.3f)*cos(ARM_LK_Motor2.angle/57.3f);
		
		yaw_td.r = yaw_td_1.r;
		
//		if(Yaw_encode>35&&Yaw_encode<85)
//		{
////		Pitch_SpeedPID.fpKp = (Pitch_SpeedPID_3.fpKp-4);
////		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_3.fpKd-20;
////		
////		Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp-4;
////		Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
////		Pitch_PosPID.fpKd = Pitch_PosPID_3.fpKd-20;
//		Pitch_SpeedPID.fpKp = (Pitch_SpeedPID_3.fpKp-5);
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_3.fpKd-25;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp-6;
//		Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_3.fpKd-25;

//		pitch_td.r = pitch_td_2.r;
//						
//		}
//       
//		Pitch_SpeedPID.fpKp = (Pitch_SpeedPID_3.fpKp);
//		Pitch_SpeedPID.fpKd = Pitch_SpeedPID_3.fpKd;
//		
//		Pitch_PosPID.fpKp = Pitch_PosPID_3.fpKp;
//		Pitch_PosPID.fpKi = Pitch_PosPID_3.fpKi;
//		Pitch_PosPID.fpKd = Pitch_PosPID_3.fpKd;
//		
	}
//	
//	
//						if (Pitch_encode > 14 ) // 20-40
//	{
////		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp + (Yaw_SpeedPID_2.fpKp - Yaw_SpeedPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd + (Yaw_SpeedPID_2.fpKd - Yaw_SpeedPID_1.fpKd) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp + (Yaw_PosPID_2.fpKp - Yaw_PosPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
////		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd;
////		yaw_td.r = yaw_td_2.r;
//		
//		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp-10;
//		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd-200;
//		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp-5;
//		Yaw_PosPID.fpKi = Yaw_PosPID_1.fpKi;
//		Yaw_PosPID.fpKd = Yaw_PosPID_1.fpKd-200;
//		yaw_td.r = yaw_td_1.r;
//	}
//		
//	}
////	else
////	{
////					if (Pitch_encode > 14 ) // 20-40
////	{

////		
////		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_2.fpKp;
////		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_2.fpKd;
////		Yaw_PosPID.fpKp = Yaw_PosPID_2.fpKp;
////		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
////		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd;
////		yaw_td.r = yaw_td_2.r;
////	}
//	else{
//		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp;
//		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd;
//		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp;
//		Yaw_PosPID.fpKi = Yaw_PosPID_1.fpKi;
//		Yaw_PosPID.fpKd = Yaw_PosPID_1.fpKd;
//		yaw_td.r = yaw_td_1.r;
//	}
//	
//	
//				if (Pitch_encode > 24 ) // 20-40
//	{
////		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_1.fpKp + (Yaw_SpeedPID_2.fpKp - Yaw_SpeedPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_1.fpKd + (Yaw_SpeedPID_2.fpKd - Yaw_SpeedPID_1.fpKd) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKp = Yaw_PosPID_1.fpKp + (Yaw_PosPID_2.fpKp - Yaw_PosPID_1.fpKp) / 25 * (Pitch_encode - 15);
////		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
////		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd;
////		yaw_td.r = yaw_td_2.r;
//		
//		Yaw_SpeedPID.fpKp = Yaw_SpeedPID_2.fpKp-20;
//		Yaw_SpeedPID.fpKd = Yaw_SpeedPID_2.fpKd-100;
//		Yaw_PosPID.fpKp = Yaw_PosPID_2.fpKp-10;
//		Yaw_PosPID.fpKi = Yaw_PosPID_2.fpKi;
//		Yaw_PosPID.fpKd = Yaw_PosPID_2.fpKd-100;
//		yaw_td.r = yaw_td_2.r;
//	}

	/**********************************Yaw6020******************************/
	// PID

	CalIWeakenPID(&Yaw_PosPID);
	Yaw_SpeedPID.fpDes = Yaw_PosPID.fpU + Yaw_k * yaw_td.x2;
	CalIWeakenPID(&Yaw_SpeedPID);
	/**********************************Pitch6020****************************/
	Gravity_Compensation(Pitch_PosPID.fpFB);
	
	if(system_monitor.CAN1_fps < 10)
	{
		Pitch_PosPID.fpSumE = 0;
	}

	CalIWeakenPID(&Pitch_PosPID);
	Pitch_SpeedPID.fpDes = Pitch_PosPID.fpU + Pitch_k * pitch_td.x2;
	CalIWeakenPID(&Pitch_SpeedPID);

	/**********************************Roll DM4310******************************/

	CalIWeakenPID(&Roll_PosPID);
	//	Roll_SpeedPID.fpDes = Roll_PosPID.fpU/4000*(V_MAX-V_MIN)+Roll_vc;
	Roll_SpeedPID.fpDes = Roll_PosPID.fpU;

	CalIWeakenPID(&Roll_SpeedPID);

	/******************************准备发送的电流****************************/
	//	if(g_stFriction2SMC.fpFB<7000&&g_stFriction2SMC.fpFB>6800)
	//	Yaw_C=10000;//0.1272
	//	else
	//	Yaw_C=0;
	
	
	/*Friction_FB = ((DesSpeed1-100 - g_stFriction2SMC.fpFB) > 0) ? (DesSpeed1-100 - g_stFriction2SMC.fpFB) : 0;

	if ((Friction_FB - Friction_PreFB) < 0)
	{
		if (FrictionWheel_Ready == TRUE)
		{
			Pitch_C = (Friction_FB) * 20 + (Friction_FB - Friction_PreFB) * 25;
		}
	}
	else
		Pitch_C = 0;

	Friction_PreFB = Friction_FB;*/
	

	YawCurrent = -(SSHORT16)(Yaw_SpeedPID.fpU - Yaw_C);
	
	PitchCurrent = -(SSHORT16)((Pitch_SpeedPID.fpU + GravityCompensation+Pitch_C));
//	YawCurrent = 0;
//	
//	PitchCurrent = 0;
	//	RollCurrent = (Roll_SpeedPID.fpU ) /25000*(T_MAX-T_MIN)+ Roll_c;
	RollCurrent = Roll_SpeedPID.fpU;
	Roll_Current_Des = Roll_SpeedPID.fpU+Roll_k;
}

// 遥控器控制模式
void Gimbal_RC_Mode(void)
{

	//	FrictionWheel_UP_Ready = 0;
	//	FrictionWheel_Ready =FALSE;

	/*-----------------------------------遥控器通道限制--------------------------------*/
	if (abs(DR16_rec.stRC.Ch1 - RC_CH_VALUE_OFFSET) < RC_CH_VALUE_DEAD) // ch1-1024<20
		DR16_rec.stRC.Ch1 = RC_CH_VALUE_OFFSET;
	if (abs(DR16_rec.stRC.Ch0 - RC_CH_VALUE_OFFSET) < RC_CH_VALUE_DEAD)
		DR16_rec.stRC.Ch0 = RC_CH_VALUE_OFFSET;

	PitchPos_Reference -= (FP32)(DR16_rec.stRC.Ch1 - RC_CH_VALUE_OFFSET) * RCSST_Pitch;

	YawPos_Reference += (FP32)(DR16_rec.stRC.Ch0 - RC_CH_VALUE_OFFSET) * RCSST_Yaw;

	RollPos_Reference = 0;

	if (Pitch_imu_flag == 1)
	{
		PitchPos_Reference = Clip(PitchPos_Reference, Pitch_Limit_Down_imu, Pitch_Limit_Up_imu);
	}
	else
	{
		PitchPos_Reference = Clip(PitchPos_Reference, Pitch_Limit_Down, Pitch_Limit_Up);
	}
	if (Yaw_imu_flag == 1)
	{
		YawPos_Reference = Clip(YawPos_Reference, Yaw_Limit_Down_imu, Yaw_Limit_Up_imu);
	}
	else
	{
		YawPos_Reference = Clip(YawPos_Reference, Yaw_Limit_Down, Yaw_Limit_Up);
	}
	if (Roll_imu_flag == 1)
	{
		RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down_imu, Roll_Limit_Up_imu);
	}
	else
	{
		RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down, Roll_Limit_Up);
	}
	if (test_flag == 1)
	{
		yaw_kf.raw_value = YawPos_Reference + Visionrx_freq_data_p;
		pitch_kf.raw_value = PitchPos_Reference + Visionrx_freq_data_p;
	}
	else
	{
		pitch_kf.raw_value = PitchPos_Reference;
		yaw_kf.raw_value = YawPos_Reference;
	}

	Kalman_Filter(&yaw_kf);
	yaw_td.aim = yaw_kf.x_now;
	TD_Function(&yaw_td);
	// Yaw_PosPID.fpDes = yaw_kf.x_now;
	Yaw_PosPID.fpDes = yaw_td.x1;

	Kalman_Filter(&pitch_kf);
	pitch_td.aim = pitch_kf.x_now;
	TD_Function(&pitch_td);
	//  Pitch_PosPID.fpDes = pitch_kf.x_now;
	Pitch_PosPID.fpDes = pitch_td.x1;
	
	Roll_PosPID.fpDes = RollPos_Reference;
}

// 键鼠控制模式
SINT32 PitchBaseCnt = 0;
SINT32 YawBaseCnt = 0;
UINT32 Sentinel_cnt = 0;
u8 sentry_random = 0;
u32 d_or_s_flag = 0;  // 1--动态,0--静态  哨兵
u8 spinning_flag = 0; // 操作手切换小陀螺
u8 change_xiangji_flag = 0;
void Gimbal_KeyMouse_Mode(void)
{
	Balanced_Infantry_Number_Choose();
	Aim_Mode_Choose();
	if (Gimbal_Control == Aim_Mode) //&&unAimData.stEnemyE.Is_Rcg_FLAG == 1
	{
		if (Pitch_imu_flag == 1)
		{
			unAimData.stEnemyE.E_Pitch = Clip(unAimData.stEnemyE.E_Pitch, Pitch_Limit_Down_imu, Pitch_Limit_Up_imu);
		}
		else
		{
			unAimData.stEnemyE.E_Pitch = Clip(unAimData.stEnemyE.E_Pitch, Pitch_Limit_Down, Pitch_Limit_Up);
		}
		if (Yaw_imu_flag == 1)
		{
			unAimData.stEnemyE.E_Yaw = Clip(unAimData.stEnemyE.E_Yaw, Yaw_Limit_Down_imu, Yaw_Limit_Up_imu);
		}
		else
		{
			unAimData.stEnemyE.E_Yaw = Clip(unAimData.stEnemyE.E_Yaw, Yaw_Limit_Down, Yaw_Limit_Up);
		}
		// if (unAimData.stEnemyE.LENTH == 0 && unAimData.stEnemyE.lock == 0 && unAimData.stEnemyE.Pitch_filter == 0 && unAimData.stEnemyE.Yaw_filter == 0)
		// {
		if (Roll_imu_flag == 1)
		{
			RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down_imu, Roll_Limit_Up_imu);
		}
		else
		{
			RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down, Roll_Limit_Up);
		}
		if (unAimData.stEnemyE.lock == 0)
		{
			PitchPos_Reference = Pitch_PosPID.fpFB;
			YawPos_Reference = Yaw_PosPID.fpFB;
			RollPos_Reference = 0;
		}
		else
		{
			PitchPos_Reference = -unAimData.stEnemyE.E_Pitch;
			YawPos_Reference = unAimData.stEnemyE.E_Yaw;
			RollPos_Reference = 0;
		}
		if (FrictionWheel_Ready)
		{
			if (KEY_SingleClick_W())
				PitchBaseCnt++;
			else if (KEY_SingleClick_S())
				PitchBaseCnt--;
			if (KEY_SingleClick_A())
				YawBaseCnt--;
			else if (KEY_SingleClick_D())
				YawBaseCnt++;
		}
		else
			PitchBaseCnt = 0, YawBaseCnt = 0;

		PitchPos_Reference -= PitchBaseCnt * Pitch_Compensation_Step_Vision;
		YawPos_Reference += YawBaseCnt * Yaw_Compensation_Step_Vision; // 其实它才是最终的
	}
	else if (Gimbal_Control == NO_Aim_Mode) //||(Gimbal_Control == Aim_Mode&&unAimData.stEnemyE.Is_Rcg_FLAG == 0
	{
		PitchPos_Reference -= MouseSST_Pitch * Mouse_Y.AVE;
		YawPos_Reference += MouseSST_Yaw * Mouse_X.AVE;
		RollPos_Reference = 0;

		if (PRESSED_W)
			PitchPos_Reference -= Pitch_Compensation_Step;
		else if (PRESSED_S)
			PitchPos_Reference += Pitch_Compensation_Step;
		if (PRESSED_A)
			YawPos_Reference -= Yaw_Compensation_Step;
		else if (PRESSED_D)
			YawPos_Reference += Yaw_Compensation_Step;
	}

	if (Pitch_imu_flag == 1)
	{
		PitchPos_Reference = Clip(PitchPos_Reference, Pitch_Limit_Down_imu, Pitch_Limit_Up_imu);
	}
	else
	{
		PitchPos_Reference = Clip(PitchPos_Reference, Pitch_Limit_Down, Pitch_Limit_Up);
	}
	if (Yaw_imu_flag == 1)
	{
		YawPos_Reference = Clip(YawPos_Reference, Yaw_Limit_Down_imu, Yaw_Limit_Up_imu);
	}
	else
	{
		YawPos_Reference = Clip(YawPos_Reference, Yaw_Limit_Down, Yaw_Limit_Up);
	}
	if (Roll_imu_flag == 1)
	{
		RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down_imu, Roll_Limit_Up_imu);
	}
	else
	{
		RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down, Roll_Limit_Up);
	}

	yaw_kf.raw_value = YawPos_Reference;
	Kalman_Filter(&yaw_kf);
	yaw_td.aim = yaw_kf.x_now;
	TD_Function(&yaw_td);
	//	Yaw_PosPID.fpDes = yaw_kf.x_now;
	Yaw_PosPID.fpDes = yaw_td.x1;

	pitch_kf.raw_value = PitchPos_Reference;
	Kalman_Filter(&pitch_kf);
	pitch_td.aim = pitch_kf.x_now;
	TD_Function(&pitch_td);
	//	Pitch_PosPID.fpDes = pitch_kf.x_now;
	Pitch_PosPID.fpDes = pitch_td.x1;

	Roll_PosPID.fpDes = RollPos_Reference;
}
// 视觉控制模式
FP32 last_rc_ch3 = 0;
FP32 last_rc_ch1 = 0;
FP32 last_rc_dial = 0;


void Gimbal_VisionControl_Mode(void)
{
	Gimbal_Control = Aim_Mode;

	//------------小陀螺切换-------------//	左摇杠向下
	if (DR16_rec.stRC.Ch3 == RC_CH_VALUE_MIN && last_rc_ch3 != RC_CH_VALUE_MIN)
	{
		//			if(!spinning_flag)	spinning_flag=1;
		//		else spinning_flag=0;
		spinning_flag = 1;
	}
	if (DR16_rec.stRC.Dial == RC_CH_VALUE_MIN && last_rc_dial != RC_CH_VALUE_MIN)

	{
		spinning_flag = 0;
	}
	last_rc_ch3 = DR16_rec.stRC.Ch3;
	last_rc_dial = DR16_rec.stRC.Dial;

	if (Pitch_imu_flag == 1)
	{
		unAimData.stEnemyE.E_Pitch = Clip(unAimData.stEnemyE.E_Pitch, Pitch_Limit_Down_imu ,Pitch_Limit_Up_imu);

	}
	else
	{
		unAimData.stEnemyE.E_Pitch = Clip(unAimData.stEnemyE.E_Pitch, Pitch_Limit_Down, Pitch_Limit_Up);
	}
	if (Yaw_imu_flag == 1)
	{
		unAimData.stEnemyE.E_Yaw = Clip(unAimData.stEnemyE.E_Yaw, Yaw_Limit_Down_imu, Yaw_Limit_Up_imu);
	}
	else
	{
		unAimData.stEnemyE.E_Yaw = Clip(unAimData.stEnemyE.E_Yaw, Yaw_Limit_Down, Yaw_Limit_Up);
	}
	if (Roll_imu_flag == 1)
	{
		RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down_imu, Roll_Limit_Up_imu);
	}
	else
	{
		RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down, Roll_Limit_Up);
	}
	/****************/

	if (unAimData.stEnemyE.LENTH == 0 && unAimData.stEnemyE.lock == 0 && unAimData.stEnemyE.Pitch_filter == 0 && unAimData.stEnemyE.Yaw_filter)

	{
		PitchPos_Reference = Pitch_PosPID.fpFB;
		YawPos_Reference = Yaw_PosPID.fpFB;
		RollPos_Reference = 0;
		Pitch_PosPID.fpDes = Pitch_PosPID.fpFB;
		Yaw_PosPID.fpDes = Yaw_PosPID.fpFB;
		Roll_PosPID.fpDes = 0;
	}
	else
	{
		PitchPos_Reference =   unAimData.stEnemyE.E_Pitch;
		YawPos_Reference = unAimData.stEnemyE.E_Yaw;
					//YawPos_Reference += MouseSST_Yaw * remote_control_image_transmission.mouse_x;

		RollPos_Reference = 0;

		//        yaw_kf.raw_value = YawPos_Reference;
		//        Kalman_Filter(&yaw_kf);
		//        yaw_td.aim = yaw_kf.x_now;
		yaw_td.aim = YawPos_Reference;
		TD_Function(&yaw_td);
		//	Yaw_PosPID.fpDes = yaw_kf.x_now;
		Yaw_PosPID.fpDes = yaw_td.x1;

		//        pitch_kf.raw_value = PitchPos_Reference;
		//        Kalman_Filter(&pitch_kf);
		//        pitch_td.aim = pitch_kf.x_now;
		pitch_td.aim = PitchPos_Reference;
		TD_Function(&pitch_td);
		//	Pitch_PosPID.fpDes = pitch_kf.x_now;
		Pitch_PosPID.fpDes = pitch_td.x1;
		Roll_PosPID.fpDes = RollPos_Reference;
	}
}

void Gimbal_ImageControl_Mode(void)
{
	Press_Y_N();
	Aim_Mode_Choose(); // 是否打前哨站or基地or切换相机等
	Aim_Mode_Switch(); // 是否开辅瞄
//	Balanced_Infantry_Number_Choose();
	if (Gimbal_Control == Aim_Mode)
	{
		if (Pitch_imu_flag == 1)
		{
			//////注意检查限幅，可能和视觉规定方向相反，注意符号
		unAimData.stEnemyE.E_Pitch = Clip(unAimData.stEnemyE.E_Pitch,Pitch_Limit_Down_imu, Pitch_Limit_Up_imu);
		}
		else
		{
			unAimData.stEnemyE.E_Pitch = Clip(unAimData.stEnemyE.E_Pitch, Pitch_Limit_Down, Pitch_Limit_Up);
		}
		if (Yaw_imu_flag == 1)
		{
			unAimData.stEnemyE.E_Yaw = Clip(unAimData.stEnemyE.E_Yaw, Yaw_Limit_Down_imu, Yaw_Limit_Up_imu);
		}
		else
		{
			unAimData.stEnemyE.E_Yaw = Clip(unAimData.stEnemyE.E_Yaw, Yaw_Limit_Down, Yaw_Limit_Up);
		}
		if (Roll_imu_flag == 1)
		{
			RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down_imu, Roll_Limit_Up_imu);
		}
		else
		{
			RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down, Roll_Limit_Up);
		}
		if (unAimData.stEnemyE.LENTH == 0 && G_ST_Vision.Receive.FindTargetOrNot == (bool)0 && unAimData.stEnemyE.Pitch_filter == 0 && unAimData.stEnemyE.Yaw_filter)

		{
			PitchPos_Reference = Pitch_PosPID.fpFB;
			YawPos_Reference = Yaw_PosPID.fpFB;
		}
		else if (Absolute_value(Yaw_PosPID.fpFB - unAimData.stEnemyE.E_Yaw) > 180.0f)
		{
			YawPos_Reference = Yaw_PosPID.fpFB;
		}
		else
		{
//			 if((int)(unAimData.stEnemyE.target_num)==6)//前哨
//			 {
			PitchPos_Reference = unAimData.stEnemyE.E_Pitch;
//			 if((int)(unAimData.stEnemyE.target_num)!=6)//
//			 {
			YawPos_Reference =   unAimData.stEnemyE.E_Yaw;
//			 }
//			 if((int)(unAimData.stEnemyE.target_num)==6)//前哨
//			 {
//			YawPos_Reference += MouseSST_Yaw * remote_control_image_transmission.mouse_x;
			 //}

		}
		RollPos_Reference = Roll_PosPID.fpFB;

			if (PRESSED_W)
		Pitch_E +=  Pitch_Compensation_Step_Vision;
			else if (PRESSED_S)
		Pitch_E -=  Pitch_Compensation_Step_Vision;
			if (PRESSED_A)
		Yaw_E -= Yaw_Compensation_Step_Vision; 
			else if (PRESSED_D)
		Yaw_E += Yaw_Compensation_Step_Vision; 
			
			///////////////////按Q取消手动偏置
		if (PRESSED_Q)
		{
			Pitch_E = 0;
			Yaw_E = 0;
		}
					
			
	}
	else if (Gimbal_Control == NO_Aim_Mode)
	{
		PitchPos_Reference += MouseSST_Pitch * remote_control_image_transmission.mouse_y;
		YawPos_Reference -= MouseSST_Yaw * remote_control_image_transmission.mouse_x;
		RollPos_Reference = 0;
		if (PRESSED_W)
			PitchPos_Reference += Pitch_Compensation_Step;
		else if (PRESSED_S)
			PitchPos_Reference -= Pitch_Compensation_Step;
		if (PRESSED_A)
			YawPos_Reference += Yaw_Compensation_Step;
		else if (PRESSED_D)
			YawPos_Reference -= Yaw_Compensation_Step;
		
					///////////////////按Q取消手动偏置
		if (PRESSED_Q)
		{
			Pitch_E = 0;
			Yaw_E = 0;
		}
	}

	if (Pitch_imu_flag == 1)
	{
		PitchPos_Reference = Clip(PitchPos_Reference, Pitch_Limit_Down_imu, Pitch_Limit_Up_imu);
	}
	else
	{
		PitchPos_Reference = Clip(PitchPos_Reference, Pitch_Limit_Down, Pitch_Limit_Up);
	}
	if (Yaw_imu_flag == 1)
	{
		YawPos_Reference = Clip(YawPos_Reference, Yaw_Limit_Down_imu, Yaw_Limit_Up_imu);
	}
	else
	{
		YawPos_Reference = Clip(YawPos_Reference, Yaw_Limit_Down, Yaw_Limit_Up);
	}
	if (Roll_imu_flag == 1)
	{
		RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down_imu, Roll_Limit_Up_imu);
	}
	else
	{
		RollPos_Reference = Clip(RollPos_Reference, Roll_Limit_Down, Roll_Limit_Up);
	}
	


	yaw_kf.raw_value = YawPos_Reference;
	Kalman_Filter(&yaw_kf);
	yaw_td.aim = YawPos_Reference;
	TD_Function(&yaw_td);
	//	Yaw_PosPID.fpDes = yaw_kf.x_now;
	Yaw_PosPID.fpDes = yaw_td.x1;

	pitch_kf.raw_value = PitchPos_Reference;
	Kalman_Filter(&pitch_kf);
	pitch_td.aim = PitchPos_Reference;
	TD_Function(&pitch_td);
	//	Pitch_PosPID.fpDes = pitch_kf.x_now;
	Pitch_PosPID.fpDes = pitch_td.x1;

	//Roll_PosPID.fpDes = RollPos_Reference;
}
// 重力补偿
void Gravity_Compensation(FP32 Angle)
{
	//	if(Pitch_PosPID.fpFB > -10)
	//	{
	//	 k = 8000;
	//	}
	//	else
	//	{
	//	 k = 6000;
	//	}
	Angle = Angle * 3.14f / 180.0f;
		/*
     if (ARM_LK_Motor2.angle > 46 && ARM_LK_Motor2.angle<=60)
	{
//		kc = 190----220;
		kc = 330+(ARM_LK_Motor2.angle-43)*3;

	}
	else if (ARM_LK_Motor2.angle > 40 && ARM_LK_Motor2.angle<=46)
	{
//		kc = 120---190;
		kc = 310+(ARM_LK_Motor2.angle-40)*3.5f;

	}
	else if (ARM_LK_Motor2.angle > 37 && ARM_LK_Motor2.angle<=40)
	{
//		kc = 80;
		kc = 280+(ARM_LK_Motor2.angle-37)*10;
	}
	else if (ARM_LK_Motor2.angle > 32 && ARM_LK_Motor2.angle<=37)
	{
		//kc = 50----60;
		kc = 250+(ARM_LK_Motor2.angle-32)*7.5;
	}

	else if (ARM_LK_Motor2.angle > 29 && ARM_LK_Motor2.angle<=32)
	{
//		kc = 38;
		kc = 200+(ARM_LK_Motor2.angle-29)*10;
	}
	
	
	
	else if (ARM_LK_Motor2.angle > 26 && ARM_LK_Motor2.angle<=29)
	{
//		kc = 28;
		kc = 180+(ARM_LK_Motor2.angle-26)*6;

	}
	else if (ARM_LK_Motor2.angle > 20 && ARM_LK_Motor2.angle<=26)
	{
//		kc = 20;
		kc = 160+(ARM_LK_Motor2.angle-22)*3.5;

	}
	else if (ARM_LK_Motor2.angle > 14 && ARM_LK_Motor2.angle<=20)
	{
//		kc = 15;
		kc = 130+(ARM_LK_Motor2.angle-18)*5;

	}
	else if (ARM_LK_Motor2.angle > -9 && ARM_LK_Motor2.angle<=14)
	{
//		kc = 5---10;
		kc = 100;
	}
	
	*/
	
//	else if (Pitch_encode > -15 && Pitch_encode<=-1)
//	{
//		kc = 500;
//	}
//	else 
		//kc = 0;
//	}

	GravityCompensation = (FP32)k * sin(Angle + Angle0 * 3.14f / 180.0f) + kc;
}

float Angle_180_To_Inf(float angle_input, ST_ANGLE *st_angle)
{
	st_angle->angle_180 = angle_input;

	if (st_angle->angle_180_pre - st_angle->angle_180 > 180)						  // 上比这大180
		st_angle->angle_inf += (st_angle->angle_180 - st_angle->angle_180_pre) + 360; // 这-上+360
	else if (st_angle->angle_180 - st_angle->angle_180_pre > 180)					  // 这次比上次大180
		st_angle->angle_inf += (st_angle->angle_180 - st_angle->angle_180_pre) - 360;
	else
		st_angle->angle_inf += (st_angle->angle_180 - st_angle->angle_180_pre);

	st_angle->angle_180_pre = angle_input;

	return st_angle->angle_inf;
}

float Angle_Inf_To_180(float angle)
{
	while (angle > +180)
		angle -= 360;
	while (angle < -180)
		angle += 360;
	return angle;
}

FP32 Diff = 0;
ST_ANGLE G_ST_IMU_Angle = {0};

void IMU_Data_Deal(void)
{
	IMU_Update_Mahony(&imu_data, 1e-3f);
	// your code
	if (!Pitch_imu_flag)
	{
		//Pitch_PosPID.fpFB = Pitch_encode ; // 直接获取反馈值
		Pitch_PosPID.fpFB = ARM_LK_Motor2.angle ; // 直接获取反馈值
	}
	else  
	{
		Pitch_PosPID.fpFB = -imu_data.pit;
		//Pitch_PosPID.fpFB = -imu_data.rol;
	}

	if (!Yaw_imu_flag)
		Yaw_PosPID.fpFB = Yaw_encode; // 直接获取反馈值
	else
		Yaw_PosPID.fpFB = Angle_180_To_Inf(imu_data.yaw, &G_ST_IMU_Angle);

	if (!Roll_imu_flag)
		Roll_PosPID.fpFB = Roll_encode; // 直接获取反馈值
	else
		Roll_PosPID.fpFB = -imu_data.rol;

	/*速度反馈原来*/
	if (Yaw_s_flag)
		Yaw_SpeedPID.fpFB = (Gyro_Z_Real * radian);

	if (Pitch_s_flag)
		Pitch_SpeedPID.fpFB = -(Gyro_Y_Real * radian);
		//Pitch_SpeedPID.fpFB = -(Gyro_X_Real * radian);
	

	if (Roll_s_flag)
		Roll_SpeedPID.fpFB = (Gyro_X_Real * radian);

	/*卡尔曼滤波*/
	if (Pitch_kf_flag)
	{
		pitch_speed_kf.raw_value = Pitch_SpeedPID.fpFB;
		Kalman_Filter(&pitch_speed_kf);
		Pitch_SpeedPID.fpFB = pitch_speed_kf.x_now;
	}
	if (Yaw_kf_flag)
	{
		yawfb_kf.raw_value = Yaw_SpeedPID.fpFB;
		Kalman_Filter(&yawfb_kf);
		Yaw_SpeedPID.fpFB = yawfb_kf.x_now;
	}
}
