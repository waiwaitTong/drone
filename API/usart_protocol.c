#include "usart_protocol.h"
#include "bmi088_driver.h"
u8 testFlag, IDFlag, LengthFlag = 0;
u8 dataFlag = 0;

int Vision_flag = 0; // 用vofa测试发送数据是否正常

/*******************************数据帧协议格式*******************************/
/*0x55    0x00    0xXX    0xXX    |0xXX 0xXX 0xXX 0xXX|    0x00    0xAA*/
/*帧头             ID   数据长度  |   1个浮点数据     |            帧尾*/
/****************************************************************************/
// UCHAR8 Visual_Data_RxBuffer[Visual_Data_Size] = {0};

void rx_handle(USART_RX_TypeDef *USARTx_Rcr) // 视觉
{
	if (USARTx_Rcr->pMailbox[0] == 0x55 && USARTx_Rcr->pMailbox[1] == 0x00 && USARTx_Rcr->pMailbox[UART4_RXMB_LEN - 1] == 0xaa)//首两位和最后一位
	{
		memcpy(&unAimData.ucEData, (u8 *)USARTx_Rcr->pMailbox, sizeof(unAimData.ucEData));//58位
	}
	else
	{
		memset(USARTx_Rcr->pMailbox, 0, UART4_RXMB_LEN); // 清零
	}
	//	shooting_or_not_r=unAimData.ucEData[12];
}

typedef enum
{
    //对应掩码Vision_Type_Mask = 0x0f
  Vision_Assist_Aim_Normal        = 0x00, //辅瞄-敌方正常模式
  Vision_Assist_Aim_TOP           = 0x01, //辅瞄-敌方小陀螺模式
  Vision_Interfere_BigBuff        = 0x02, //反大符
  Vision_Interfere_SmallBuff      = 0x05, //反小符
  Vision_SmallBuff                = 0x03, //小幅
  Vision_BigBuff                  = 0x04, //大幅

  //对应掩码Target_Type_Mask = 0x10
  Vision_Target_Red               = 0x00, //目标为红
  Vision_Target_Blue              = 0x10, //目标为蓝

  //对应掩码Vision_Cmd_Mask  = 0x80
  Vision_Disable                  = 0x00, //视觉失能
  Vision_Enable                   = 0x80, //视觉使能
}USART_Protocol_ID;
//串口6发送处理：控制板>>>视觉小电脑
void Vision_Tx_Protocol(void)
{
	uint8_t Vision_State_Now = (Gimbal_Control==Aim_Mode?Vision_Enable:Vision_Disable) | (Drone_State.robot_id == 6?Vision_Target_Blue:Vision_Target_Red) | (Vision_Assist_Aim_Normal);
  G_ST_Vision.Send.head[0]     = 0x55;
  G_ST_Vision.Send.head[1]     = 0x11;
	G_ST_Vision.Send.id          = Vision_State_Now;
  G_ST_Vision.Send.num         = 6;

   G_ST_Vision.Send.Pitch       = Angle_Inf_To_180(Pitch_PosPID.fpFB);
	// G_ST_Vision.Send.Rol         = Angle_Inf_To_180(Rol_Angle);

	G_ST_Vision.Send.Yaw             = Angle_Inf_To_180(Yaw_PosPID.fpFB);
//  G_ST_Vision.Send.Vision_Is_Close = Vision_Is_Close;
//	G_ST_Vision.Send.Pitch_error_Num = 	128+Operater_Coe.Pitch_error_Num;
//	G_ST_Vision.Send.Yaw_error_Num   = 	128+Operater_Coe.Yaw_error_Num;
//	G_ST_Vision.Send.Time_delay_Num  = 	128+Operater_Coe.Time_delay_Num;
//  G_ST_Vision.Send.BigBuff_Pitch_error_Num = 128 + Operater_Coe.BigBuff_Pitch_error_Num;
//	G_ST_Vision.Send.BigBuff_Yaw_error_Num = 128 + Operater_Coe.BigBuff_Yaw_error_Num;
	G_ST_Vision.Send.PelletSpeed = SlidingWindowFilter(&bullet_speed_filter,Shoot_Data.bullet_speed);//(float)PelletSpeed_mes;
//	G_ST_Vision.Send.Vision_shutdown = (Vision_Shutdown_Cnt>=5000)?1:0;
	
  ///UART4
  Append_CRC16_Check_Sum(&G_ST_Vision.Send.head[0], UART4_TX_LEN);
	
	system_monitor.USART4tx_cnt += DMA_GetCurrDataCounter(UART4_TX_STREAM)==0?1:0;
  DMA_ClearITPendingBit(UART4_TX_STREAM, DMA_IT_TCIF4);	//开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

  DMA_Cmd(UART4_TX_STREAM, DISABLE);				              //设置当前计数值前先禁用DMA
  UART4_TX_STREAM->M0AR = (uint32_t)&G_ST_Vision.Send;   //设置当前待发数据基地址:Memory0 tARget
  UART4_TX_STREAM->NDTR = (uint32_t)UART4_TX_LEN;     //设置当前待发的数据的数量:Number of Data units to be TRansferred
  DMA_Cmd(UART4_TX_STREAM, ENABLE);
} 
//小电脑->主控
int temp = 0 ;
extern UCHAR8 UA4RxDMAbuf[];
void Vision_Rx_Protocol(void)
{
	/*2024/7/20 袁神主控烂，遂同时用串口4和串口6，一个坏了换一个*/ 
	/////////串口4
    if( UA4RxDMAbuf[0]==0x55&&                                  //帧头校验
      UA4RxDMAbuf[1]==0x11&&                                  //帧头校验
			Verify_CRC16_Check_Sum(UA4RxDMAbuf, UART4_RXMB_LEN) )  //帧尾检验
  {
		memcpy(&G_ST_Vision.Receive, &UA4RxDMAbuf, UART4_RXMB_LEN);

    //视觉数据可能出现问题，因此将这一帧舍弃
    if( fpclassify(G_ST_Vision.Receive.Pitch)!=FP_NORMAL ||
        fpclassify(G_ST_Vision.Receive.Yaw)!=FP_NORMAL)
    {
			G_ST_Vision.Receive.FindTargetOrNot = FALSE;
      G_ST_Vision.Receive.Pitch = Pitch_PosPID.fpFB;
			G_ST_Vision.Receive.Yaw = Yaw_PosPID.fpFB;
    }
    else
    {   //将目标值转为离当前Yaw反馈值最近的角度
			temp = ( (int) (G_ST_Vision.Receive.Yaw - Yaw_PosPID.fpFB ) ) / 360;
			G_ST_Vision.Receive.Yaw -= temp*360;
			if( G_ST_Vision.Receive.Yaw - Yaw_PosPID.fpFB > 180 )
			{
				G_ST_Vision.Receive.Yaw -= 360;
			}
			else if( G_ST_Vision.Receive.Yaw - Yaw_PosPID.fpFB < -180 )
			{
				G_ST_Vision.Receive.Yaw += 360;
			}

      if( fabs(G_ST_Vision.Receive.Yaw - Yaw_PosPID.fpFB)>90.0f ||
          fabs(G_ST_Vision.Receive.Pitch - Pitch_PosPID.fpFB)>90.0f )	//如果视觉辅瞄的数据和当前位置差的角度很大就默认为辅瞄没跟上
      {
				G_ST_Vision.Receive.FindTargetOrNot = FALSE ;
        G_ST_Vision.Receive.Pitch = Pitch_PosPID.fpFB ;
        G_ST_Vision.Receive.Yaw = Yaw_PosPID.fpFB ;
		  
      }
	}
	unAimData.stEnemyE.E_Pitch = G_ST_Vision.Receive.Pitch;
	unAimData.stEnemyE.E_Yaw = G_ST_Vision.Receive.Yaw;
	
	}
}

/*----------------------------------------------------------------------------------------
函数名：PitchAngleUpdate(void)
功  能：向视觉更新数据
----------------------------------------------------------------------------------------*/
u8 robot_type3 = 0, robot_type4 = 0, robot_type5 = 0;
u8 fistflag_1 = 1, fistflag_2 = 1, fistflag_3 = 1;
//void PitchAngleUpdate(void)
//{
//		G_ST_Vision.Send.head[0] = 0x55;
//		G_ST_Vision.Send.head[1] = 0x00;

//		if (Drone_State.robot_id == 6)
//		{
//			G_ST_Vision.Send.head[2] = 0x09;	// 红方ID 7	(识别蓝色)
//			if (Game_Status.game_progress == 4) // 3V3
//			{
//				robot_type3 = (infantry3_flag) ? 1 : 0;
//				robot_type4 = (infantry4_flag) ? 1 : 0;
//				robot_type5 = (infantry5_flag) ? 1 : 0;
//				fistflag_1 = 0, fistflag_2 = 0, fistflag_3 = 0;
//			}
//			else
//				fistflag_1 = 1, fistflag_2 = 1, fistflag_3 = 1, robot_type3 = 0, robot_type4 = 0, robot_type5 = 0;
//		}
//		else if (Drone_State.robot_id == 106)
//		{
//			G_ST_Vision.Send.head[2] = 0x0A; // 蓝方ID 17	(识别红色)
//			if (Game_Status.game_progress == 4)
//			{
//				robot_type3 = (infantry3_flag) ? 1 : 0;
//				robot_type4 = (infantry4_flag) ? 1 : 0;
//				robot_type5 = (infantry5_flag) ? 1 : 0;
//				//					  if(infantry3_flag)  robot_type3=1;
//				//					  if(infantry4_flag)  robot_type4=1;
//				//					  if(infantry5_flag)  robot_type5=1;
//				fistflag_1 = 0, fistflag_2 = 0, fistflag_3 = 0;
//			}
//			else
//				fistflag_1 = 1, fistflag_2 = 1, fistflag_3 = 1, robot_type3 = 0, robot_type4 = 0, robot_type5 = 0;
//		}
//		G_ST_Vision.Send.head[3] = 0x07;
//		yawfb_kf.raw_value = Yaw_PosPID.fpFB;
//		pitchfb_kf.raw_value = -Pitch_PosPID.fpFB;
//		rollfb_kf.raw_value = imu_data.rol;
//		Kalman_Filter(&pitchfb_kf);
//		Kalman_Filter(&yawfb_kf);
//		Kalman_Filter(&rollfb_kf);
//		yaw_fileter = yawfb_kf.x_now;
//		pitch_fileter = pitchfb_kf.x_now;
//		roll_fileter = rollfb_kf.x_now;
//		G_ST_Vision.Send.Pitchangle = pitch_fileter;
//		G_ST_Vision.Send.Yawangle = yaw_fileter;
////		G_ST_Vision.Send.bullet_speed = 25.0;
//		G_ST_Vision.Send.bullet_speed =SlidingWindowFilter(&bullet_speed_filter,Shoot_Data.bullet_speed);//滑动窗口滤波
//		G_ST_Vision.Send.mode = (spinning_flag << 24) | (danger_flag << 16) | (shooting_or_not_s << 8) | Gimbal_Control; // 是否小陀螺&对面是否残血&是否打弹&辅瞄是否开启
//		G_ST_Vision.Send.Pitch_e =Pitch_E ;
//		G_ST_Vision.Send.Yaw_e = Yaw_E;//角度
//		G_ST_Vision.Send.Rollangle = roll_fileter;
//		G_ST_Vision.Send.tail[0] = 0x00;
//		G_ST_Vision.Send.tail[1] = 0xAA;
//		////	Append_CRC16_Check_Sum(&G_ST_Vision.Send.head[0], 18);  //CRC检验，以后再加上去
//		/************************DAM发送***********************************/
//		while (DMA_GetCurrDataCounter(DMA1_Stream4))
//			;											   // 等之前的发完
//		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4); // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

//		DMA_Cmd(DMA1_Stream4, DISABLE);					  // 设置当前计数值前先禁用DMA
//		DMA1_Stream4->M0AR = (uint32_t)&G_ST_Vision.Send; // 设置当前待发数据基地址:Memory0 tARget
//		DMA1_Stream4->NDTR = (uint32_t)UART4_TX_LEN;	  // 设置当前待发的数据的数量:Number of Data units to be TRansferred
//		DMA_Cmd(DMA1_Stream4, ENABLE);

//}
/*----------------------------------------------------------------------------------------
函数名：USHORT16 USART_Receive(USART_RX_TypeDef* USARTx)
功  能：计算一次空闲中断接受数据长度，对DMA缓冲区数据一定长度数据进行存储
----------------------------------------------------------------------------------------*/
USHORT16 USART_Receive(USART_RX_TypeDef *USARTx)
{
	USARTx->rxConter = USARTx->DMALen - DMA_GetCurrDataCounter(USARTx->DMAy_Streamx); // 本次DMA缓冲区填充到的位置

	USARTx->rxBufferPtr += USARTx->rxSize; // 上次DMA缓冲区填充到的位置

	if (USARTx->rxBufferPtr >= USARTx->DMALen) // 说明DMA缓冲区已经满了一次
	{
		USARTx->rxBufferPtr %= USARTx->DMALen;
	}

	if (USARTx->rxBufferPtr < USARTx->rxConter) // 上次和这次在同一段缓冲区时
	{
		USARTx->rxSize = USARTx->rxConter - USARTx->rxBufferPtr; // 计算本次接收数据的长度
		if (USARTx->rxSize <= USARTx->MbLen)					 // 接收的数据长度不超过期望数据长度，把数据写进邮箱，防止数组越界
		{
			for (u16 i = 0; i < USARTx->rxSize; i++)
				*(USARTx->pMailbox + i) = *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);
		}
	}
	else // 上次和这次在不同段的缓冲区时
	{
		USARTx->rxSize = USARTx->rxConter + USARTx->DMALen - USARTx->rxBufferPtr; // 计算本次接收数据的长度
		if (USARTx->rxSize <= USARTx->MbLen)									  // 接收的数据长度不超过期望数据长度，把数据写进邮箱，防止数组越界
		{
			for (u16 i = 0; i < USARTx->rxSize - USARTx->rxConter; i++)
				*(USARTx->pMailbox + i) = *(USARTx->pDMAbuf + USARTx->rxBufferPtr + i);
			for (u16 i = 0; i < USARTx->rxConter; i++)
				*(USARTx->pMailbox + USARTx->rxSize - USARTx->rxConter + i) = *(USARTx->pDMAbuf + i);
		}
	}
	return USARTx->rxSize; // 返回本次空闲中断一共接收多少字节
}

void USART_Transmit(USART_TX_TypeDef *USARTx)
{
	DMA_Cmd(USARTx->DMAy_Streamx, DISABLE); // 关闭DMA传输
	while (DMA_GetCmdStatus(USARTx->DMAy_Streamx) != DISABLE)
		; // 确保DMA可以被设置
	for (u16 i = 0; i < USARTx->DMALen; i++)
		*(USARTx->pDMAbuf + i) = *(USARTx->pMailbox + i);		 // 设置数据
	DMA2_Stream7->M0AR = (UINT32)(USARTx->pDMAbuf);				 // 设置地址
	DMA_SetCurrDataCounter(USARTx->DMAy_Streamx, USARTx->MbLen); // 数据传输量
	DMA_Cmd(USARTx->DMAy_Streamx, ENABLE);						 // 开启DMA传输
}

/*************************************************************************
函 数 名：void Send_Data_Update(void)
函数功能：发送给伏特加的数据
备    注：
*************************************************************************/
extern u32 Delta_t2;
extern u32 Delta_t1;
void Send_Data_Update(void)
{ /*typedef struct
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
	u8 tail[4];
} Send_Data_Vofa;*/
	//  Send.Pitch_Angle_Des=Pitch_encode;
	Send_vofa.Pitch_Angle_Des = Pitch_PosPID.fpDes;
	Send_vofa.Yaw_Angle_Des = Yaw_PosPID.fpDes;
	Send_vofa.Pitch_Angle_Fb = Pitch_PosPID.fpFB;
	Send_vofa.Yaw_Angle_Fb = Yaw_PosPID.fpFB;
    
	Send_vofa.five  = Supply_PosPID.fpFB;														   /*pitch_td.x1;*/
	Send_vofa.six   = Supply_PosPID.fpDes; /*pitch_kf.x_now;*/								   /*unAimData.stEnemyE.Yaw_calculate*/
    Send_vofa.seven = yaw_td.aim;   /*Visionrx_freq_data_p;*/							   /*unAimData.stEnemyE.Yaw_filter*/
	
    Send_vofa.eight = Shoot_Data.bullet_speed;       /*unAimData.stEnemyE.Yaw_v;*/ /*pitch_kf.raw_value;*/ /*system_monitor.USART4rx_fps;*/
	Send_vofa.nine  = unAimData.stEnemyE.E_Pitch;
	Send_vofa.ten   = unAimData.stEnemyE.E_Yaw;
    Send_vofa.eleven= Pitch_SpeedPID.fpDes;
    Send_vofa.twelve= Pitch_SpeedPID.fpFB;
    Send_vofa.Thirteen=Yaw_SpeedPID.fpDes;
    Send_vofa.Fourteen=Yaw_SpeedPID.fpFB;
    Send_vofa.Fifteen=(float)Bullet_num_actaul;
	//	Send.x=Shoot_Data.bullet_speed;/*unAimData.stEnemyE.Pitch_v;*//*Pitch_PosPID.fpDes;*//*yaw_kf.raw_value;*/

	Send_vofa.tail[0] = 0x00;
	Send_vofa.tail[1] = 0x00;
	Send_vofa.tail[2] = 0x80;
	Send_vofa.tail[3] = 0x7f;

	while (DMA_GetCurrDataCounter(DMA1_Stream3))
		;											   // 等之前的发完DMA_GetCurrDataCounter()获取剩余数据量
	DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3); // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

	DMA_Cmd(DMA1_Stream3, DISABLE);					 // 设置当前计数值前先禁用DMA
	DMA1_Stream3->M0AR = (uint32_t)&Send_vofa;			 // 设置当前待发数据基地址:Memory0 tARget
	DMA1_Stream3->NDTR = (uint32_t)USART3_TXDMA_LEN; // 设置当前待发的数据的数量:Number of Data units to be TRansferred
	DMA_Cmd(DMA1_Stream3, ENABLE);
    

}
    

