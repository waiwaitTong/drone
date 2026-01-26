#include "gimnol.h"
#include "imu_update_task.h"


void DMADATAcontrol(void)
{
	if(DNGimbalRx_DMABUF[0]==0x34 && DNGimbalRx_DMABUF[1]==0x33 && DNGimbalRx_DMABUF[USART3_DMA_RxBuf_LEN-1]==0x31 )
	{
		memcpy(&DNGimbalDataBuf.Receive,(u8*)DNGimbalRx_DMABUF,sizeof(DNGimbalDataBuf.Receive));
		
	}	
	else
	{
		memset(DNGimbalRx_DMABUF,0,USART3_DMA_RxBuf_LEN);
	}
}

void DMADATASEND(void)
{
//	DNGimbalDataBuf.Send.rol_send=imu_data.rol;
//	DNGimbalDataBuf.Send.pit_send=imu_data.pit;
//	DNGimbalDataBuf.Send.yaw_send=imu_data.yaw;
//	DNGimbalDataBuf.Send.Gyro_X_Speed_send=Gyro_X_Speed;
//	DNGimbalDataBuf.Send.Gyro_Y_Speed_send=BMIPitchSpeed;
//	DNGimbalDataBuf.Send.Gyro_Z_Speed_send=BMIYawSpeed;
//	
////  DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF3);
////  DMA_Cmd(DMA1_Stream4,DISABLE);
////  DMA1_Stream4->M0AR = (u32)&DNGimbalDataBuf.Send;
////	DMA1_Stream4->NDTR = (u32)USART3_DMA_TxBuf_LEN;
////	DMA_Cmd(DMA1_Stream1,ENABLE);
//	DMA_Cmd(DMA1_Stream4,DISABLE);
//	while (DMA_GetCmdStatus(DMA1_Stream4)!=DISABLE){};
//	DMA_ClearITPendingBit(DMA1_Stream4,DMA_IT_TCIF4);
//	DMA_SetCurrDataCounter(DMA1_Stream4,USART3_DMA_TxBuf_LEN);
//	DMA_Cmd(DMA1_Stream4,ENABLE);
//	system_monitor.usart3_tx_cnt++;
	
}

