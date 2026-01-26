#include "motor.h"

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
    tx_message.Data[3] = (UCHAR8)gmyaw_i;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

/*************************************************************************
函 数 名：Send_Current_To_Pitch
函数功能：向2006拨弹电机发送电流
备    注：电流值范围：-10000~10000
*************************************************************************/
void Send_Current_To_Pitch(CAN_TypeDef* CANx, SSHORT16 supply_i)
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
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = (UCHAR8)(supply_i >> 8);
    tx_message.Data[7] = (UCHAR8)supply_i;
    CAN_Transmit(CANx,&tx_message);
	//system_monitor.GimbalTask_cnt++; //发送电流后视为成功
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
函 数 名：GetTorque
函数功能：接收GM3510的转矩
备    注：
*************************************************************************/
SINT32 Get_Torque(CanRxMsg* rx_message)
{
    SINT32 encoder_temp;
	encoder_temp = rx_message->Data[2]<<8 | rx_message->Data[3];
	return encoder_temp;
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
/*************************************************************************
函 数 名：Change_State_Deal()
函数功能：控制方式切换处理
备    注：
*************************************************************************/
void Change_Control_State_Deal(void)
{

}

/*************************************************************************
函 数 名：Send_Voltage_To_Pitch
函数功能：向6020电机发送电压
备    注：电流值范围：-30000~30000
*************************************************************************/
void Send_Voltage_To_Pitch(CAN_TypeDef* CANx, SSHORT16 voltage)
{
	CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (UCHAR8)(voltage >> 8);
    tx_message.Data[1] = (UCHAR8)voltage;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}
