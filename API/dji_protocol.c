#include "dji_protocol.h"

/*************************************************************************
函 数 名：Send_Current_To_Gimbal
函数功能：向2006拨弹电机发送电流
备    注：电流值范围：-10000~10000
*************************************************************************/
void Send_Current_To_SupplyPelletMotor(CAN_TypeDef* CANx, SSHORT16 supply_i)
{
	CanTxMsg tx_message;    
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = (UCHAR8)(supply_i >> 8);
    tx_message.Data[5] = (UCHAR8)supply_i;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

/*************************************************************************
函 数 名：Send_Current_To_Gimbal
函数功能：向云台电机6020发送电压电流值
备    注：电压值范围：-30000~30000(6020) -16384~16384(3508)
*************************************************************************/
void Send_Current_To_Gimbal(CAN_TypeDef *CANx,SSHORT16 gmpitch_v,SSHORT16 gmyaw_i)
{
	CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (UCHAR8)(gmpitch_v>>8);  
    tx_message.Data[1] = (UCHAR8)gmpitch_v; 
    tx_message.Data[2] = (UCHAR8)(gmyaw_i>>8);
    tx_message.Data[3] = (UCHAR8)(gmyaw_i);
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

/*************************************************************************
函 数 名：Send_Current_To_Friction
函数功能：向云台电机3508发送电流值
备    注： -16384~16384(3508)
*************************************************************************/
void Send_Current_To_Friction(CAN_TypeDef *CANx,SSHORT16 Friction1_v,SSHORT16 Friction2_v)
{
	CanTxMsg tx_message;    
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (UCHAR8)(Friction1_v>>8);  
    tx_message.Data[1] = (UCHAR8)Friction1_v; 
    tx_message.Data[2] = (UCHAR8)(Friction2_v>>8);
    tx_message.Data[3] = (UCHAR8)(Friction2_v);
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}
/*************************************************************************
函 数 名：GetEncoderNumber
函数功能：接收6025、6623或RM3510电调板返回的机械角度值（绝对式编码器值）
备    注：机械角度值范围：0~8191（0x1FFF）
*************************************************************************/
SINT32 Get_Encoder_Number(CanRxMsg* rx_message)
{
    SINT32 encoder_temp;
	encoder_temp = rx_message->Data[0]<<8 | rx_message->Data[1];
	return encoder_temp;
}

/*************************************************************************
函 数 名：Get_Current_Number
函数功能：接收6026电机实际转矩电流值
备    注：机械角度值范围：0~8191（0x1FFF）
*************************************************************************/
SINT32 Get_Current_Number(CanRxMsg* rx_message)
{
    SINT32 current_temp;
	current_temp = (rx_message->Data[4]<<8 | rx_message->Data[5]);
	return current_temp;
}

/************************************************************************
函 数 名：Get_Speed
函数功能：接收RM3510或RM3508电调板返回的转速，单位：r/min
备    注：
*************************************************************************/
SINT32 Get_Speed(CanRxMsg* rx_message)
{
    SINT32 speed_temp;
	SINT32 base_value=0xFFFF;
	if(rx_message->Data[2] & 0x01<<7)
	{	
		speed_temp = (base_value<<16 | rx_message->Data[2]<<8 | rx_message->Data[3]);
	}
	else
	{
	    speed_temp = (rx_message->Data[2]<<8 | rx_message->Data[3]);//rpm
	}
	return speed_temp;
}

