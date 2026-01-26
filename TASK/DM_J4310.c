#include "main.h"

float position, velocity, torque;

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits
    ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

CanTxMsg TxMessage;
uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
FP32 DM_Kp=0;
FP32 DM_Kd=0;


// MIT 控制模式发送函数
// 反馈ID 0x15，发送ID 0x03
void ctrl_motor(CAN_TypeDef *CANx, float _torq)
{
    pos_tmp = float_to_uint(Des2, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(Roll_SpeedPID.fpDes, V_MIN, V_MAX, 12);
//    vel_tmp = float_to_uint(0, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(DM_Kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(DM_Kd, KD_MIN, KD_MAX, 12);
//    tor_tmp = float_to_uint(Roll_current, T_MIN, T_MAX, 12);
//      Des2=Roll_current;
     tor_tmp = float_to_uint(Roll_Current_Des, T_MIN, T_MAX, 12);

    TxMessage.StdId = 0x03;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = (pos_tmp >> 8);
    TxMessage.Data[1] = pos_tmp;
    TxMessage.Data[2] = (vel_tmp >> 4);
    TxMessage.Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    TxMessage.Data[4] = kp_tmp;
    TxMessage.Data[5] = (kd_tmp >> 4);
    TxMessage.Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    TxMessage.Data[7] = tor_tmp;

    CAN_Transmit(CANx, &TxMessage);
    
}



void ctrl_motor_v(CAN_TypeDef* CANx, float _vel)
{
    CanTxMsg TxMessage;
    uint8_t vbuf[4];

    // Copy float value into byte array
    memcpy(vbuf, &Roll_SpeedPID.fpDes, sizeof(float));

    // Configure the CAN message
    TxMessage.StdId = 0x203;
    TxMessage.ExtId = 0x00;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 4;
    TxMessage.Data[0] = vbuf[0];
    TxMessage.Data[1] = vbuf[1];
    TxMessage.Data[2] = vbuf[2];
    TxMessage.Data[3] = vbuf[3];


    CAN_Transmit(CANx, &TxMessage);
    
}

void send_enable(CAN_TypeDef* CANx)
{
    CanTxMsg TxMessage;

    // 配置控制帧的 ID
    TxMessage.StdId = 0x03;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08;  // 数据长度为8字节

    // 填充数据段
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFC;

    // 发送 CAN 消息
    CAN_Transmit(CANx, &TxMessage);
}

void send_disable(CAN_TypeDef* CANx)
{
    CanTxMsg TxMessage;

    // 配置控制帧的 ID
    TxMessage.StdId = 0x03;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08;  // 数据长度为8字节

    // 填充数据段
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFD;

    // 发送 CAN 消息
    CAN_Transmit(CANx, &TxMessage);
}

void Send_Zero(CAN_TypeDef* CANx)
{
    CanTxMsg TxMessage;

    // 配置控制帧的 ID
    TxMessage.StdId = 0x03;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = 0x08;  // 数据长度为8字节

    // 填充数据段
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFE;

    // 发送 CAN 消息
    CAN_Transmit(CANx, &TxMessage);
}


/**
 * @brief: 翎控电机使能函数
 * @param {CAN_HandleTypeDef} *hcan
 * @param {u32} id
 * @note: 
 * @author: HITCRT
 */
void CAN_7010_Open(CAN_TypeDef* CANx,u32 id)
{
	static CanTxMsg TxMessage;
	TxMessage.StdId=id;
	TxMessage.IDE=CAN_Id_Standard;
	TxMessage.DLC=0x08;
	TxMessage.RTR=CAN_RTR_DATA;
	
	TxMessage.Data[0]=0x88;
	TxMessage.Data[1]=0x00;
	TxMessage.Data[2]=0x00;
	TxMessage.Data[3]=0x00;
	TxMessage.Data[4]=0x00;
	TxMessage.Data[5]=0x00;
	TxMessage.Data[6]=0x00;
	TxMessage.Data[7]=0x00;
	CAN_Transmit(CANx,&TxMessage);
}

/**
 * @brief: 翎控电机单个电机电流控制函数
 * @param {CAN_HandleTypeDef} *hcan
 * @param {u32} id
 * @param {s16} current
 * @note: 数值范围-2048~ 2048 对应 ，MG 电机实际转矩电流范围-33A~33A，
 * @author: HITCRT
 */

void CAN_7010_SendCurrent_Single(CAN_TypeDef *CANx,u32 id,s16 current)
{
	static CanTxMsg TxMessage;
	TxMessage.StdId=id;
	TxMessage.IDE=CAN_Id_Standard;
	TxMessage.DLC=0x08;
	TxMessage.RTR=CAN_RTR_DATA;
	
	TxMessage.Data[0]=0xA1;
	TxMessage.Data[1]=0x00;
	TxMessage.Data[2]=0x00;
	TxMessage.Data[3]=0x00;
	TxMessage.Data[4]=current;
	TxMessage.Data[5]=current>>8;
	TxMessage.Data[6]=0x00;
	TxMessage.Data[7]=0x00;
	CAN_Transmit(CANx,&TxMessage);
}
