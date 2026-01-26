#include "stm32f4xx_it.h"
#include "rm_types_my.h"
#include "main.h"
#include "stm32f4xx.h"
// #include "global_declare.h"
#include "gimnol.h"
// #include "gimbol_control_task.h"
#include <cstring>
USHORT16 Clear_IT = 0;
/*************************************************************************
中断处理函数名称：SysTick_Handler
中断产生机制：1ms进入一次中断。
函数功能：将SysTick（系统滴答定时器）作为操作系统的时钟
*************************************************************************/
extern void xPortSysTickHandler(void);
void SysTick_Handler(void)
{
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) // 系统已经运行
	{
		xPortSysTickHandler();
	}
	system_monitor.System_cnt++;
	system_monitor.System_time++; // 1ms
	system_monitor.System_cnt_100ms = system_monitor.System_time / 100;
	system_monitor.System_time_1s = system_monitor.System_time / 1000;
	if (!RC_ON)
	{
		Safemode();
	}
	if (system_monitor.System_cnt == 1000)
	{
		system_monitor.System_cnt = 0;
	}
}

/*************************************************************************
中断处理函数名称：USART2_IRQHandler
中断产生机制：USART2接收到一个空字节后触发中断
函数功能：裁判系统通讯
*************************************************************************/
extern USART_RX_TypeDef USART2_Rcr;
s16 length;
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		USART2->SR;
		USART2->DR;							 // 先读SR后读DR清楚中断标志位
		length = USART_Receive(&USART2_Rcr); // 读出裁判系统发送的数据为50位
		//		if(USART_Receive(&USART2_Rcr)==50||USART_Receive(&USART2_Rcr)==80)//不知道为什么该语句不起作用
		//		{
		Rc_RsysProtocol(USART2_Rcr.rxSize);
		system_monitor.USART2rx_cnt++;
		//		}
	}
}

/***********************************************************************************
中断处理函数名称：DMA1_Stream6_IRQHandler
中断产生机制：串口2发送完成中断
函数功能：
************************************************************************************/
void DMA1_Stream6_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6))
	{
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6); // 清除标志位
		DMA_Cmd(DMA1_Stream6, DISABLE);				 // 关闭DMA传输
	}
}
/*************************************************************************
中断处理函数名称：USART1_IRQHandler
中断产生机制：DR16每隔14ms通过DBus发送一帧数据（18字节）,USART1每接收一帧数据
			 进入一次中断
函数功能：遥控器传输数据接收
*************************************************************************/
extern USART_RX_TypeDef USART1_Rcr;
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		Clear_IT = USART1->SR;
		Clear_IT = USART1->DR; // 先读SR后读DR清楚中断标志位

		if (USART_Receive(&USART1_Rcr) == USART1_RXMB_LEN)
		{
			RC_Protocol();
			system_monitor.USART1_cnt++;
			system_monitor.USART1_cnt_100ms++;
		}
	}
}
/***********************************************************************************
中断处理函数名称：UART4_IRQHandler
中断产生机制：视觉通讯
函数功能：
************************************************************************************/
void UART4_IRQHandler(void)
{
	if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET) // 空闲中断
	{
		Clear_IT = UART4->SR;
		Clear_IT = UART4->DR; // 先读SR后读DR清楚中断标志位

		if( DMA_GetCurrDataCounter(UART4_RX_STREAM) == UART4_RXMB_LEN )//if (USART_Receive(&UART4_Rcr) == UART4_RXMB_LEN) // 把数组放在邮箱里，返回本次接收长度
		{
			Vision_Rx_Protocol();//运算最大消耗时间不超过2us

			system_monitor.USART4rx_cnt++;
		}
		       else
        {
            DMA_Cmd(UART4_RX_STREAM, DISABLE);         //设置当前计数值前先禁用DMA
            UART4_RX_STREAM->NDTR = UART4_RXMB_LEN;   //设置当前待发的数据的数量:Number of Data units to be TRansferred
            DMA_Cmd(UART4_RX_STREAM, ENABLE);          //启用串口DMA接收
        }

	}
}

/*************************************************************************
中断处理函数名称：CAN1_TX_IRQHandler
中断产生机制：
函数功能：
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
	}
}

uint16_t p_int, v_int, t_int;
extern ST_ANGLE G_ST_IMU_Angle;
FP32 Pitch_ZERO = -80; //-60
FP32 Yaw_ZERO = 256;  // 180
FP32 Roll_ZERO = 0.0;
/***********************************************************
中断处理函数名称：CAN1_RX0_IRQHandler
中断产生机制：
函数功能：
*************************************************************************/
CanRxMsg CAN1_RxMsg;
void CAN1_RX0_IRQHandler(void)
{

	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RxMsg);
		switch (CAN1_RxMsg.StdId)
		{

		case 0x206: // yaw  0x204+ID
			Yaw_encode = -Angle_Inf_To_180(-Get_Encoder_Number(&CAN1_RxMsg) * 360.0 / 8192.0 + Yaw_ZERO);
			if (Yaw_s_flag == 0)
				Yaw_SpeedPID.fpFB = Get_Speed(&CAN1_RxMsg);
			Yaw_current = (float)Get_Current_Number(&CAN1_RxMsg);
			Yaw_Limit_Up_imu = Angle_180_To_Inf(imu_data.yaw, &G_ST_IMU_Angle) + Yaw_encode + Yaw_Limit_Up; // 限位实时更新
			Yaw_Limit_Down_imu = Angle_180_To_Inf(imu_data.yaw, &G_ST_IMU_Angle) + Yaw_encode + Yaw_Limit_Down;
			break;
			
//		case 0x206: // pitch
//			Pitch_encode = -Angle_Inf_To_180(-Get_Encoder_Number(&CAN1_RxMsg) * 360.0 / 8192.0 - Pitch_ZERO);
//			if (Pitch_s_flag == 0)
//				Pitch_SpeedPID.fpFB = Get_Speed(&CAN1_RxMsg);
//			Pitch_current = (float)Get_Current_Number(&CAN1_RxMsg);
//			Pitch_Limit_Up_imu = -imu_data.pit - Pitch_encode + Pitch_Limit_Up;
//			Pitch_Limit_Down_imu = -imu_data.pit - Pitch_encode + Pitch_Limit_Down;
//			break;
			
		case 0x141:	//LK电机接收  0x140+ID
			LK_7010_DataProcess(CAN1_RxMsg.Data, &ARM_LK_Motor2);
			ARM_LK_Motor2.angle = (ARM_LK_Motor2.LK_motor_encoder.SumValue )* 360.0f / 65535.f - Pitch_ZERO;
			
			ARM_LK_Motor2.anglev = ARM_LK_Motor2.speed/10.f;///32768.0f*180.0f/10.f; 
		
			if (Pitch_s_flag == 0)
			Pitch_SpeedPID.fpFB =  ARM_LK_Motor2.anglev;
			
			ARM_LK_Motor2.LK_ID = 0x141;
			Abs_Encoder_Process_LK(&ARM_LK_Motor2.LK_motor_encoder, ARM_LK_Motor2.encoder_num);
			
			Pitch_Limit_Up_imu = -imu_data.pit + ARM_LK_Motor2.angle + Pitch_Limit_Up;
			Pitch_Limit_Down_imu = -imu_data.pit + ARM_LK_Motor2.angle + Pitch_Limit_Down;
			break;

//		case 0x15:
//			p_int = (CAN1_RxMsg.Data[1] << 8) | CAN1_RxMsg.Data[2];
//			v_int = (CAN1_RxMsg.Data[3] << 4) | (CAN1_RxMsg.Data[4] >> 4);
//			t_int = ((CAN1_RxMsg.Data[4] & 0xF) << 8) | CAN1_RxMsg.Data[5];
//			Roll_encode = uint_to_float(p_int, P_MIN, P_MAX, 16)*180/3.1415926535;
//			if(Roll_s_flag==0)
//			Roll_SpeedPID.fpFB = uint_to_float(v_int, V_MIN, V_MAX, 12);
//			torque = uint_to_float(t_int, T_MIN, T_MAX, 12);
//			break;

		default:
			break;
		}
		system_monitor.CAN1_cnt++;
	}
}
/*************************************************************************
中断处理函数名称：CAN2_TX_IRQHandler
中断产生机制：
函数功能：
*************************************************************************/
void CAN2_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2, CAN_IT_TME) != RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
	}
}
/*************************************************************************
中断处理函数名称：CAN2_RX0_IRQHandler
中断产生机制：
函数功能：
*************************************************************************/

void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg CAN2_RxMsg;

	if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);

		CAN_Receive(CAN2, CAN_FIFO0, &CAN2_RxMsg);
		switch (CAN2_RxMsg.StdId)
		{
		case 0x201: // 修改id
			g_stFriction1SMC.fpFB = Get_Speed(&CAN2_RxMsg);
			break;

		case 0x202:
			g_stFriction2SMC.fpFB = Get_Speed(&CAN2_RxMsg);
			break;
        
		case 0x203: // 拨弹电机的ID为3,2
			Abs_Encoder_Process(&g_stShooterEncoder, Get_Encoder_Number(&CAN2_RxMsg));
			Supply_PosPID.fpFB = g_stShooterEncoder.siSumValue;//19662
			Supply_SpeedPID.fpFB = Get_Speed(&CAN2_RxMsg) / 36.0f;//2006
//        	Supply_SpeedPID.fpFB =Get_Speed(&CAN2_RxMsg)/ 19.20321f;//拨盘的r/min  3508
//        	Supply_SpeedPID.fpFB =Get_Speed(&CAN2_RxMsg)/ 51.0f;//拨盘的r/min

			// 拨盘转一圈 点击平均转295527.8转  共八格，每格步进码数=36941
			break;
		default:
			break;
		}
		system_monitor.CAN2_cnt++;
	}
}
/*************************************************************************
函 数 名：Abs_Encoder_Process(volatile ST_ENCODER* encoder, SINT32 value)
函数功能：RM3510电机绝对式编码器数据处理，得到转速
备    注:
*************************************************************************/
void Abs_Encoder_Process(volatile ST_ENCODER *encoder, SINT32 value)
{
	static FP32 fpVeltCoff;
	encoder->siPreRawValue = encoder->siRawValue;
	encoder->siRawValue = value;
	encoder->siDiff = encoder->siRawValue - encoder->siPreRawValue;
	if (encoder->siDiff < -6000) // 两次编码器的反馈值差别太大,表示绝对式编码器圈数发生了改变 //高速供弹必须大于6k
	{
		encoder->siDiff += (encoder->siNumber + 1);
	}
	else if (encoder->siDiff > 6000) // 两次编码器的反馈值差别太大,表示绝对式编码器圈数发生了改变
	{
		encoder->siDiff -= (encoder->siNumber + 1);
	}

	fpVeltCoff = 60.0 / encoder->siGearRatio / encoder->siNumber / 0.001; // 0.001是指两次采样间隔1ms

	encoder->fpSpeed = fpVeltCoff * encoder->siDiff; // 单位：r/min
	encoder->siSumValue += encoder->siDiff;			 // 记录编码器的总数，位置闭环用
}

/*************************************************************************
函 数 名:Abs_Encoder_Process_LK
函数功能:判断LK码盘正反转
备    注:
*************************************************************************/
void Abs_Encoder_Process_LK(ST_ENCODER_LK* pEncoder, s32 value)
{
	pEncoder->PreRawValue = pEncoder->RawValue;
	pEncoder->RawValue    = value;
	pEncoder->Diff        = pEncoder->RawValue - pEncoder->PreRawValue;
	
	if(pEncoder->Diff < -pEncoder->Number/2)												//正转过0
		pEncoder->Diff   += pEncoder->Number;						
	else if(pEncoder->Diff > pEncoder->Number/2)										    //反转过0
		pEncoder->Diff   -= pEncoder->Number;		
	
	pEncoder->SumValue   += pEncoder->Diff;
}

// 模拟视觉帧率发数并增加噪声
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		static s8 a;
		static u8 Flag = 0;
		if (!Flag)
		{
			a = rand() % 100;
			Flag = 1;
		}
		else
		{
			a = -rand() % 100;
			Flag = 0;
		}
		static bool UpPitchDir = TRUE;
//		static bool UpYawDir = FALSE;
		if (UpPitchDir)
			//			     Visionrx_freq_data_p=Visionrx_freq_data_p+0.01f+a*0.0025;
			Visionrx_freq_data_p = a * 0.005;
		else
			//				 Visionrx_freq_data_p=Visionrx_freq_data_p-0.01f-a*0.0025;
			Visionrx_freq_data_p = -a * 0.005;
		if (Visionrx_freq_data_p > 0.2f)
			UpPitchDir = FALSE;
		if (Visionrx_freq_data_p < -0.2f)
			UpPitchDir = TRUE;
	}
}

/*************************************************************************
中断处理函数名称：USART6_IRQHandler
中断产生机制：USART6接收到一个空字节后触发中断
函数功能：图传链路
*************************************************************************/
extern USART_RX_TypeDef USART6_Rcr;
s16 length_6;
void USART6_IRQHandler(void)
{
	if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		USART6->SR;
		USART6->DR;							   // 先读SR后读DR清楚中断标志位
		length_6 = USART_Receive(&USART6_Rcr); // 读出裁判系统发送的数据为50位
		//		if(USART_Receive(&USART2_Rcr)==50||USART_Receive(&USART2_Rcr)==80)//不知道为什么该语句不起作用
		//		{
		Rc_RsysProtocol_U6(USART6_Rcr.rxSize);

		if (system_monitor.USART1_fps < 10 && system_monitor.USART6rx_fps != 0)
			g_emOperation_Mode = ImageControl_Mode;
		system_monitor.USART6rx_cnt++;
	}
}


/**
 * @brief: 
 * @param {uint8_t *} CAN_RX_BUF
 * @param {Motor8016_Msg} *MG8016_Msg
 * @note: 
 * @author: HITCRT
 */
void LK_7010_DataProcess(uint8_t * CAN_RX_BUF, Motor_LK_7010 *LK_Motor)
{
	LK_Motor->temp=CAN_RX_BUF[1];
	LK_Motor->current=CAN_RX_BUF[2] | (CAN_RX_BUF[3]<<8);
	LK_Motor->speed=CAN_RX_BUF[4] | (CAN_RX_BUF[5]<<8);
	LK_Motor->encoder_num=CAN_RX_BUF[6] | (CAN_RX_BUF[7]<<8);
}
