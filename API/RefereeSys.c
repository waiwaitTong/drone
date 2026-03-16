#include "RefereeSys.h"
#include "delay.h"
#include "os.h"
/*-----------------------------------------------------------------------------
						RoboMaster2021裁判系统通讯协议
frame_header(5-byte) | cmd_id(2-byte) | data(n-byte) | frame_tail(2-byte,CRC16)
						  Frame_Header (5-byte)
SOF(1-byte)	| data_length(2-byte) | seq(1-byte) | CRC8(1-byte)
   0xA5            数据长度           包序号      帧头CRC8校验
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
	域 			        偏移位置        大小（字节）              详细描述
	SOF 		           0 		          1 		      数据帧起始字节，固定值为 0xA5
	DataLength         1 		          2 		      数据帧内 Data 长度
	Seq		             3 		          1           包序号
	CRC8 	             4          		1           帧头 CRC 校验
  CmdID(2-Byte)      5              2           命令码
  Data(n-Byte)	     7              n			      命令码对应数据包
  FrameTail(CRC16)  7+n             2  		  	  帧尾校验
----------------------------------------------------------------------------*/
extern UCHAR8 UA2RxMailbox[USART2_RXMB_LEN];
UCHAR8 USART2_Rx_Buf[USART2_RXMB_LEN];

extern UCHAR8 UA6RxMailbox[USART6_RXMB_LEN];
UCHAR8 USART6_Rx_Buf[USART6_RXMB_LEN];

USHORT16 ucCmdID;	   // 命令码ID
USHORT16 usDataLength; // 数据帧长度
int DifferID = 0;	   // 与我的ID差值
UCHAR8 Seq;			   // 包序号UCHAR8
UCHAR8 CRC8;		   // 帧头CRC8校验
UCHAR8 ucHeaderCRC8;
UINT32 Verify_Ok = 0;

UCHAR8 usData;
UCHAR8 Buf_num;

/*函数功能：浮点数转整型数*/
int show_float(float x)
{
	int i, sum = 0;
	byte_pointer start;
	start = (byte_pointer)&x;
	for (i = 0; i < 4; i++)
	{
		sum += start[i] << 8 * i;
	}
	return sum;
}

/*--------------------------------------------
功能：CRC校验
--------------------------------------------*/
// crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
	{
		0x00,
		0x5e,
		0xbc,
		0xe2,
		0x61,
		0x3f,
		0xdd,
		0x83,
		0xc2,
		0x9c,
		0x7e,
		0x20,
		0xa3,
		0xfd,
		0x1f,
		0x41,
		0x9d,
		0xc3,
		0x21,
		0x7f,
		0xfc,
		0xa2,
		0x40,
		0x1e,
		0x5f,
		0x01,
		0xe3,
		0xbd,
		0x3e,
		0x60,
		0x82,
		0xdc,
		0x23,
		0x7d,
		0x9f,
		0xc1,
		0x42,
		0x1c,
		0xfe,
		0xa0,
		0xe1,
		0xbf,
		0x5d,
		0x03,
		0x80,
		0xde,
		0x3c,
		0x62,
		0xbe,
		0xe0,
		0x02,
		0x5c,
		0xdf,
		0x81,
		0x63,
		0x3d,
		0x7c,
		0x22,
		0xc0,
		0x9e,
		0x1d,
		0x43,
		0xa1,
		0xff,
		0x46,
		0x18,
		0xfa,
		0xa4,
		0x27,
		0x79,
		0x9b,
		0xc5,
		0x84,
		0xda,
		0x38,
		0x66,
		0xe5,
		0xbb,
		0x59,
		0x07,
		0xdb,
		0x85,
		0x67,
		0x39,
		0xba,
		0xe4,
		0x06,
		0x58,
		0x19,
		0x47,
		0xa5,
		0xfb,
		0x78,
		0x26,
		0xc4,
		0x9a,
		0x65,
		0x3b,
		0xd9,
		0x87,
		0x04,
		0x5a,
		0xb8,
		0xe6,
		0xa7,
		0xf9,
		0x1b,
		0x45,
		0xc6,
		0x98,
		0x7a,
		0x24,
		0xf8,
		0xa6,
		0x44,
		0x1a,
		0x99,
		0xc7,
		0x25,
		0x7b,
		0x3a,
		0x64,
		0x86,
		0xd8,
		0x5b,
		0x05,
		0xe7,
		0xb9,
		0x8c,
		0xd2,
		0x30,
		0x6e,
		0xed,
		0xb3,
		0x51,
		0x0f,
		0x4e,
		0x10,
		0xf2,
		0xac,
		0x2f,
		0x71,
		0x93,
		0xcd,
		0x11,
		0x4f,
		0xad,
		0xf3,
		0x70,
		0x2e,
		0xcc,
		0x92,
		0xd3,
		0x8d,
		0x6f,
		0x31,
		0xb2,
		0xec,
		0x0e,
		0x50,
		0xaf,
		0xf1,
		0x13,
		0x4d,
		0xce,
		0x90,
		0x72,
		0x2c,
		0x6d,
		0x33,
		0xd1,
		0x8f,
		0x0c,
		0x52,
		0xb0,
		0xee,
		0x32,
		0x6c,
		0x8e,
		0xd0,
		0x53,
		0x0d,
		0xef,
		0xb1,
		0xf0,
		0xae,
		0x4c,
		0x12,
		0x91,
		0xcf,
		0x2d,
		0x73,
		0xca,
		0x94,
		0x76,
		0x28,
		0xab,
		0xf5,
		0x17,
		0x49,
		0x08,
		0x56,
		0xb4,
		0xea,
		0x69,
		0x37,
		0xd5,
		0x8b,
		0x57,
		0x09,
		0xeb,
		0xb5,
		0x36,
		0x68,
		0x8a,
		0xd4,
		0x95,
		0xcb,
		0x29,
		0x77,
		0xf4,
		0xaa,
		0x48,
		0x16,
		0xe9,
		0xb7,
		0x55,
		0x0b,
		0x88,
		0xd6,
		0x34,
		0x6a,
		0x2b,
		0x75,
		0x97,
		0xc9,
		0x4a,
		0x14,
		0xf6,
		0xa8,
		0x74,
		0x2a,
		0xc8,
		0x96,
		0x15,
		0x4b,
		0xa9,
		0xf7,
		0xb6,
		0xe8,
		0x0a,
		0x54,
		0xd7,
		0x89,
		0x6b,
		0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
	unsigned char ucIndex;
	while (dwLength--)
	{
		ucIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}
	return (ucCRC8);
}

/*------------------------------------------------------------------
Descriptions: CRC8 Verify function
Input: Data to Verify,Stream length = Data + checksum
Output: True or False (CRC Verify Result)
------------------------------------------------------------------*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucExpected = 0;
	if ((pchMessage == 0) || (dwLength <= 2))
		return 0;
	ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
	return (ucExpected == pchMessage[dwLength - 1]);
}

/*------------------------------------------------------------------
Descriptions: append CRC8 to the end of data
Input: Data to CRC and append,Stream length = Data + checksum
Output: True or False (CRC Verify Result)
------------------------------------------------------------------*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2))
		return;
	ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
	pchMessage[dwLength - 1] = ucCRC;
}

uint16_t CRC16_INIT = 0xffff;
const uint16_t wCRC16_Table[256] =
	{
		0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
		0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
		0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
		0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
		0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
		0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
		0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
		0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
		0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
		0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
		0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
		0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
		0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
		0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
		0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
		0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
		0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
		0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
		0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
		0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
		0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
		0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
		0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
		0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
		0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
		0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
		0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
		0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
		0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
		0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
		0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
		0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/*---------------------------------------------------------------
Descriptions: CRC16 checksum function
Input: Data to check,Stream length, initialized checksum
Output: CRC checksum
----------------------------------------------------------------*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t chData;
	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}
	while (dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC16_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}

/*-----------------------------------------------------------------
Descriptions: CRC16 Verify function
Input: Data to Verify,Stream length = Data + checksum
Output: True or False (CRC Verify Result)
------------------------------------------------------------------*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return FALSE;
	}
	wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*--------------------------------------------------------------------
Descriptions: append CRC16 to the end of data
Input: Data to CRC and append,Stream length = Data + checksum
Output: True or False (CRC Verify Result)
--------------------------------------------------------------------*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return;
	}
	wCRC = Get_CRC16_Check_Sum((UCHAR8 *)pchMessage, dwLength - 2, CRC16_INIT);
	pchMessage[dwLength - 2] = (UCHAR8)(wCRC & 0x00ff);
	pchMessage[dwLength - 1] = (UCHAR8)((wCRC >> 8) & 0x00ff);
}

/*---------------------------------------------------
函数名称：Rc_RsysProtocol
功    能：裁判系统接收数据校验
---------------------------------------------------*/
void Rc_RsysProtocol(USHORT16 rxSize)
{
	static UCHAR8 DMA1_Rx_Status = USART2_RX_FREE;
	int i;
	for (i = 0; i < rxSize; i++)
	{
		usData = UA2RxMailbox[i];
		switch (DMA1_Rx_Status)
		{
		case USART2_RX_FREE:
			if (usData == 0xA5)
			{
				Buf_num = 0;
				DMA1_Rx_Status = USART2_RX_Length;
				USART2_Rx_Buf[Buf_num++] = 0xA5; // 自由状态下接收到0xA5认为接收开始，先赋值，再算自加
			}
			else
				DMA1_Rx_Status = USART2_RX_FREE;
			break;
		case USART2_RX_Length:
			if (Buf_num <= 4)
				USART2_Rx_Buf[Buf_num++] = usData; // 接收帧头全部数据
			else if (Buf_num > 4)
			{
				usDataLength = USART2_Rx_Buf[1] | USART2_Rx_Buf[2] << 8; // 接收到的帧的数据量（data长度）
				Seq = USART2_Rx_Buf[3];
				CRC8 = USART2_Rx_Buf[4]; // 帧头CRC校验
				USART2_Rx_Buf[Buf_num++] = usData;
				if (Verify_CRC8_Check_Sum(USART2_Rx_Buf, 5) == 1)
				{
					DMA1_Rx_Status = USART2_RX_CRC16; // 帧头校验成功开启数据接收
				}
				else
					DMA1_Rx_Status = USART2_RX_FREE;
			}
			break;
		case USART2_RX_CRC16:
			if (Buf_num < (9 + usDataLength))
			{
				USART2_Rx_Buf[Buf_num++] = usData;
			}
			if (Buf_num >= (9 + usDataLength))
			{
				Buf_num = 0;
				DMA1_Rx_Status = USART2_RX_FREE;
				if (Verify_CRC16_Check_Sum(USART2_Rx_Buf, usDataLength + 9))
				{
					ucCmdID = USART2_Rx_Buf[5] | (USART2_Rx_Buf[6] << 8);
					MonitorDataDeal(ucCmdID);
				}
			}
			break;
		default:
			break;
		}
	}
}

/*****************************************2024修改***********************************************
1. 修订命令码 0x0101、0x0201、0x0203、0x0209、0x020A、0x0303
2. 修订已知的数据段长度不一致或结构体定义错误
3. 完善部分描述
*/
/*****************************************2023修改但2024未修改***********************************************
1. 修改了命令字 0x0001、0x0101、0x0105、0x0203、0x0205、
0x020A 的发送频率
2. 修订命令码 0x0102、0x0104、0x0105、0x0301、0x0307
3. 新增命令码 0x020D、0x020E、0x0308
4. 完善部分描述
*/

/*---------------------------------------------
函 数 名：MonitorDataDeal
函数功能：处理接收到的裁判系统数据
备    注：
---------------------------------------------*/
void MonitorDataDeal(USHORT16 usCmdID)
{
	switch (usCmdID)
	{
	case Game_Status_ID:
		memcpy(&Game_Status, &USART2_Rx_Buf[7], usDataLength);
		RSYS_Monitor.GameStatus_cnt++;
		break;
	case Robot_State_ID:
		memcpy(&Drone_State, &USART2_Rx_Buf[7], usDataLength);
		RSYS_Monitor.GameRobotStatus_cnt++;
		break;
	case Power_Data_ID:
		memcpy(&Shoot_Current, &USART2_Rx_Buf[7], usDataLength);
		RSYS_Monitor.PowerHeatData_cnt++;
		break;
	case Robot_Pos_ID:
		memcpy(&Drone_Pos, &USART2_Rx_Buf[7], usDataLength);
		RSYS_Monitor.GameRobotPos_cnt++;
		break;
	case Drone_Energy_ID:
		memcpy(&Drone_Energy, &USART2_Rx_Buf[7], usDataLength);
		RSYS_Monitor.AerialRobotEnergy_cnt++;
		break;
	case Shoot_Data_ID:
		memcpy(&Shoot_Data, &USART2_Rx_Buf[7], usDataLength);
		RSYS_Monitor.ShootData_cnt++;
		break;
	case Robot_HP_ID:
		memcpy(&Robot_HP, &USART2_Rx_Buf[7], usDataLength);
		RSYS_Monitor.GameRobotHP_cnt++;
		break;
	case Ammo_Remain_ID:
		memcpy(&Ammo_remain, &USART2_Rx_Buf[7], usDataLength);
		RSYS_Monitor.BulletRemaining_cnt++;
		break;
	case Robo_InteractiveCmdID:
		DifferID = USART2_Rx_Buf[9] - Sender_ID; // 接收者ID-己方ID
		switch (DifferID)
		{
		case 2:												 // 飞镖
			memcpy(&DataFromMissile, &USART2_Rx_Buf[13], 3); // 为什么是13
			RSYS_Monitor.RobotInteractive_cnt++;
			break;
		case 1: // 哨兵
			memcpy(&DataFromSentinel, &USART2_Rx_Buf[13], 2);
			RSYS_Monitor.RobotInteractive_cnt++;
			break;
		case -3: // 步兵
			memcpy(&DataFromSoldier3, &USART2_Rx_Buf[13], 2);
			RSYS_Monitor.RobotInteractive_cnt++;
			break;
		case -2: // 步兵
			memcpy(&DataFromSoldier4, &USART2_Rx_Buf[13], 2);
			RSYS_Monitor.RobotInteractive_cnt++;
			break;
		case -5: // 英雄
			memcpy(&DataFromHero, &USART2_Rx_Buf[13], 2);
			RSYS_Monitor.RobotInteractive_cnt++;
			break;
		case -1: // 步兵
			memcpy(&DataFromSoldier5, &USART2_Rx_Buf[13], 2);
			RSYS_Monitor.RobotInteractive_cnt++;
			break;
		default:
			break;
		}
		break;
	case ROBOT_COMMAND_CMD_ID:
	{
		memcpy(&remote_control_image_transmission, &USART2_Rx_Buf[7], sizeof(remote_control_image_transmission));
		Press_Y_N();
	}
	break;
	default:
		break;
	}
}

/******************************************************发送到客户端********************************************************/

UCHAR8 Custom_DataBuf[128] = {0};
UCHAR8 Client_DataBuf[128] = {0};
u8 floatData1Name[4] = "FP1";
u8 floatData2Name[4] = "FP2";
u8 floatData3Name[4] = "FP3";
u8 floatData4Name[4] = "FP4";
u8 floatData5Name[4] = "FP5";
u8 floatData6Name[4] = "FP6";
u8 floatData7Name[4] = "FP7";
u8 floatData8Name[4] = "FP8";
u8 floatData9Name[4] = "FP9";
u8 floatData10Name[4] = "FPA";
u8 floatData11Name[4] = "FPB";
u8 floatData12Name[4] = "FPC";
u8 *floatDataName[] = {floatData1Name, floatData2Name, floatData3Name, floatData4Name, floatData5Name, floatData6Name, floatData7Name, floatData8Name, floatData9Name, floatData10Name,floatData11Name};
/*------------------------------------------------------------------------
函 数 名：void Picture_Operation(graphic_data_struct_t* pGraphicData, u8* graphicName, u32 operateTpye, u32 graphicTpye,
								u32 layer, u32 color, u32 startAngle, u32 endAngle, u32 width, u32 startX, u32 startY,
								u32 radius, u32 endX, u32 endY)
函数功能：对图像进行操作
------------------------------------------------------------------------*/
void pictureOperation(graphic_data_struct_t *pGraphicData, u8 *graphicName, u32 operateTpye, u32 graphicTpye,
					  u32 layer, u32 color, u32 startAngle, u32 endAngle, u32 width, u32 startX, u32 startY,
					  u32 radius, u32 endX, u32 endY)
{

	memcpy(pGraphicData->graphic_name, graphicName, 3); // 图形名 例： AimStar_Y,  1, 0, 0, 0, 0, 0, 2, 1080, 490, 0, 1080, 700
	pGraphicData->operate_tpye = operateTpye;			// 图形操作
	pGraphicData->graphic_tpye = graphicTpye;			// 图形类型
	pGraphicData->layer = layer;						// 图层数，0-9
	pGraphicData->color = color;						// 颜色
	pGraphicData->start_angle = startAngle;				// 起始角度，单位：°，范围[0,360]
	pGraphicData->end_angle = endAngle;					// 终止角度，单位：°，范围[0,360]
	pGraphicData->width = width;						// 线宽
	pGraphicData->start_x = startX;						// 起点 x 坐标
	pGraphicData->start_y = startY;						// 起点 y 坐标
	pGraphicData->radius = radius;						// 字体大小或者半径
	pGraphicData->end_x = endX;							// 终点 x 坐标
	pGraphicData->end_y = endY;							// 终点 y 坐标
														// 共13个
}

u8 AimStar_Y[4] = "YYY";
u8 AimStar_X1[4] = "XX1";
u8 AimStar_X2[4] = "XX2";
u8 AimStar_X3[4] = "XX3";
u8 AimStar_X4[4] = "XX4";
u8 AimStar_X5[4] = "XX5";
u8 AimStar_X6[4] = "XX6";
u8 AimStar_X7[4] = "XX7";
u8 AimStar_X8[4] = "XX8";
u8 AimStar_X9[4] = "XX9";
u8 AimStar_X10[4] = "XXA";
u8 AimStar_X11[4] = "XXB";
u8 AimStar_X12[4] = "XXC";

u8 Float1Name[4] = "FI1";
u8 Float2Name[4] = "FI2";
u8 Float3Name[4] = "FI3";
u8 Float4Name[4] = "FI4";
u8 Float5Name[4] = "FI5";
u8 Float6Name[4] = "FI6";
u8 Float7Name[4] = "FI7";

u8 float1Name[4] = "FE1";
u8 float2Name[4] = "FE2";
/*------------------------------------------------------------------------
函 数 名：void sendLobToClient(void)
函数功能：发送准星图层
------------------------------------------------------------------------*/
void sendLobToClient(void)
{

	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[0], AimStar_Y, 1, 0, 0, 0, 0, 0, 2, 960, 490, 0, 960, 700);
	
	int centerX = 960;

	// 循环处理每条线段
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[1], AimStar_X1, 1, 0, 0, 0, 0, 0, 2, centerX - 60, 575, 0, centerX + 60, 575);   // 线段1
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[2], AimStar_X2, 1, 0, 0, 1, 0, 0, 2, centerX - 70, 545, 0, centerX + 70, 545);   // 线段2
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[3], AimStar_X3, 1, 0, 0, 2, 0, 0, 2, centerX - 80, 515, 0, centerX + 80, 515);   // 线段3
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[4], AimStar_X4, 1, 0, 0, 4, 0, 0, 2, centerX - 90, 485, 0, centerX + 90, 485);   // 线段4
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[5], AimStar_X5, 1, 0, 0, 5, 0, 0, 2, centerX - 100, 455, 0, centerX + 100, 455); // 线段5
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[6], AimStar_X6, 1, 0, 0, 7, 0, 0, 2, centerX - 110, 425, 0, centerX + 110, 425); // 线段6

	sendPictureToClient(&G_ST_Client_Custom_Graphic_Seven, 7);
}

void sendRectangleToClient(void)
{
	pictureOperation(&G_ST_Client_Custom_Graphic_Single.grapic_data_struct, AimStar_X12, 1, 1, 0, 1, 0, 0, 2, 850, 500, 0, 1070, 650);
	sendPictureToClient(&G_ST_Client_Custom_Graphic_Single, 1);
}

/*------------------------------------------------------------------------
函 数 名：void sendDartToClient(SINT32 data1)
函数功能：发送飞镖自定义整型数据
------------------------------------------------------------------------*/
void sendDartToClient(SINT32 data1)
{
	pictureOperation(&G_ST_Client_Custom_Graphic_Single.grapic_data_struct, float1Name, 2, 6, 8, 8, 15, 0, 2, 540, 900, data1, data1 >> 10, data1 >> 21);
	sendPictureToClient(&G_ST_Client_Custom_Graphic_Single, 1);
}
/*------------------------------------------------------------------------
函 数 名：sendDataToClient(SINT32 data1, SINT32 data2, SINT32 data3, SINT32 data4, SINT32 data5）
函数功能：发送七个自定义整型数据
------------------------------------------------------------------------*/
void sendDataToClient(SINT32 data1, SINT32 data2, SINT32 data3, SINT32 data4, SINT32 data5)
{
	//	if (unAimData.stEnemyE.Is_Rcg_FLAG)
	//    {
	//		pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[0], Float1Name, 2, 6, 2, 4, 15, 0, 2, 1540, 900, data1, data1 >> 10, data1 >> 21);
	//	}
	//     else
	//     {
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[0], Float1Name, 2, 6, 4, 4, 15, 0, 2, 1540, 900, data1, data1 >> 10, data1 >> 21);
	//     }
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[1], Float2Name, 2, 6, 4, 8, 15, 0, 2, 1540, 900 - 22 * 1, data2, data2 >> 10, data2 >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[2], Float3Name, 2, 6, 4, 8, 15, 0, 2, 1540, 900 - 22 * 2, data3, data3 >> 10, data3 >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[3], Float4Name, 2, 6, 4, 8, 15, 0, 2, 1540, 900 - 22 * 3, data4, data4 >> 10, data4 >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[4], Float5Name, 2, 6, 4, 3, 15, 0, 2, 1540, 900 - 22 * 4, data5, data5 >> 10, data5 >> 21);
	sendPictureToClient(&G_ST_Client_Custom_Graphic_Five, 5);
}

/*------------------------------------------------------------------------
函 数 名：void sendDataToClient(float data1, float data2, float data3, u32 data4, float data5)
函数功能：发送五个自定义数据
------------------------------------------------------------------------*/
void sendFloatToClient(float data1, float data2, float data3, float data4, float data5)
{

	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[0], floatDataName[0], 2, 5, 4, 8, 15, 3, 2, 1540, 720, (int)(data1 * 1000), (int)(data1 * 1000) >> 10, (int)(data1 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[1], floatDataName[1], 2, 5, 4, 8, 15, 3, 2, 960, 750, (int)(data2 * 1000), (int)(data2 * 1000) >> 10, (int)(data2 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[2], floatDataName[2], 2, 5, 4, 8, 15, 3, 2, 1200, 372 + 28 * 1, (int)(data3 * 1000), (int)(data3 * 1000) >> 10, (int)(data3 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[3], floatDataName[3], 2, 5, 4, 8, 15, 3, 2, 960, 750 + 22, (int)(data4 * 1000), (int)(data4 * 1000) >> 10, (int)(data4 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Five.grapic_data_struct[4], floatDataName[4], 2, 5, 4, 8, 15, 3, 2, 1200, 372 + 50, (int)(data5 * 1000), (int)(data5 * 1000) >> 10, (int)(data5 * 1000) >> 21);

	sendPictureToClient(&G_ST_Client_Custom_Graphic_Five, 5);
}

/*------------------------------------------------------------------------
函 数 名：void sendDataToClient(float data1, float data2, float data3, u32 data4, float data5)
函数功能：发七个自定义数据
------------------------------------------------------------------------*/
void sendSEVENFloatToClient(float data1, float data2, float data3, float data4, float data5, float data6, float data7)
{

	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[0], floatDataName[0], 2, 5, 4, 8, 15, 3, 2, 1540, 720, (int)(data1 * 1000), (int)(data1 * 1000) >> 10, (int)(data1 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[1], floatDataName[1], 2, 5, 4, 8, 15, 3, 2, 960, 750, (int)(data2 * 1000), (int)(data2 * 1000) >> 10, (int)(data2 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[2], floatDataName[2], 2, 5, 4, 8, 15, 3, 2, 1200, 372 + 28 * 1, (int)(data3 * 1000), (int)(data3 * 1000) >> 10, (int)(data3 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[3], floatDataName[3], 2, 5, 4, 8, 15, 3, 2, 960, 750 + 22, (int)(data4 * 1000), (int)(data4 * 1000) >> 10, (int)(data4 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[4], floatDataName[4], 2, 5, 4, 8, 15, 3, 2, 1200, 372 + 50, (int)(data5 * 1000), (int)(data5 * 1000) >> 10, (int)(data5 * 1000) >> 21);
	
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[5], floatDataName[5], 2, 5, 4, 0, 15, 0, 2, 1020,  680,  (int)(data6 * 1000), (int)(data6 * 1000) >> 10, (int)(data6 * 1000) >> 21);
	pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[6], floatDataName[6], 2, 5, 4, 2, 15, 0, 2, 1080, 490, (int)(data7 * 1000), (int)(data7 * 1000) >> 10, (int)(data7 * 1000) >> 21);

	sendPictureToClient(&G_ST_Client_Custom_Graphic_Seven, 7);
}

/*------------------------------------------------------------------------
函 数 名：void deletePictureOnClient(u8 operateTpye, u8 layer)
函数功能：删除客户端图形
------------------------------------------------------------------------*/
void deletePictureOnClient(u8 operateTpye, u8 layer)
{
	G_ST_Student_Interactive_Header_Data.data_cmd_id = 0x0100; // 数据的内容ID

	G_ST_Student_Interactive_Header_Data.sender_ID = Sender_ID; // 发送者的ID

	G_ST_Student_Interactive_Header_Data.receiver_ID = Receiver_Client_ID;

	G_ST_Client_Custom_Graphic_Delete.operate_tpye = operateTpye;
	G_ST_Client_Custom_Graphic_Delete.layer = layer;

	Custom_DataBuf[0] = 0xA5;											  // 帧头
	Custom_DataBuf[1] = 8;												  // 数据长度低8位
	Custom_DataBuf[2] = 0;												  // 数据长度高8位
	Custom_DataBuf[3] = 0x00;											  // Seq
	Append_CRC8_Check_Sum(Custom_DataBuf, 5);							  // CRC8
	Custom_DataBuf[5] = 0x01;											  // cmd_id低八位
	Custom_DataBuf[6] = 0x03;											  // cmd_id高八位
	memcpy(&Custom_DataBuf[7], &G_ST_Student_Interactive_Header_Data, 6); // Header
	memcpy(&Custom_DataBuf[13], &G_ST_Client_Custom_Graphic_Delete, 2);	  // 数据
	Append_CRC16_Check_Sum(Custom_DataBuf, 8 + 9);						  // CRC16

	/*--------------------------开启DMA发送---------------------------*/

	DMA_Cmd(DMA1_Stream6, DISABLE); // 关闭DMA传输
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE)
		;										   // 确保DMA可以被设置
	DMA1_Stream6->M0AR = (UINT32)(Client_DataBuf); // 设置地址
	DMA1_Stream6->NDTR = 8 + 9;					   // 数据传输量
	DMA_Cmd(DMA1_Stream6, ENABLE);				   // 开启DMA传输
												   // 启用串口DMA发送
}

/*------------------------------------------------------------------------
函 数 名：void sendCharacterToClient(void* character)
函数功能：向客户端发送自定义字符
------------------------------------------------------------------------*/
void sendCharacterToClient(void *character)
{
	u16 dataLength = 6 + 15 + 30;
	G_ST_Student_Interactive_Header_Data.data_cmd_id = 0x0110;
	G_ST_Student_Interactive_Header_Data.sender_ID = Sender_ID; // 发送方ID
	G_ST_Student_Interactive_Header_Data.receiver_ID = Receiver_Client_ID;
	memcpy(&Client_DataBuf[7], &G_ST_Student_Interactive_Header_Data, 6); // Header

	Client_DataBuf[0] = 0xA5;				   // 帧头
	Client_DataBuf[1] = (u8)dataLength;		   // 数据长度低8位
	Client_DataBuf[2] = (u8)(dataLength >> 8); // 数据长度高8位
	Client_DataBuf[3] = 0x00;				   // Seq
	Append_CRC8_Check_Sum(Client_DataBuf, 5);  // CRC8
	Client_DataBuf[5] = 0x01;				   // cmd_id低八位
	Client_DataBuf[6] = 0x03;				   // cmd_id高八位

	memcpy(&Client_DataBuf[7], &G_ST_Student_Interactive_Header_Data, 6); // Header
	memcpy(&Client_DataBuf[13], character, 45);							  // 数据
	Append_CRC16_Check_Sum(Client_DataBuf, dataLength + 9);				  // CRC16

	/*--------------------------开启DMA发送---------------------------*/

	DMA_Cmd(DMA1_Stream6, DISABLE); // 关闭DMA传输
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE)
		;										   // 确保DMA可以被设置
	DMA1_Stream6->M0AR = (UINT32)(Client_DataBuf); // 设置地址
	DMA1_Stream6->NDTR = dataLength + 9;		   // 数据传输量
	DMA_Cmd(DMA1_Stream6, ENABLE);				   // 开启DMA传输
												   // 启用串口DMA发送
}
/*------------------------------------------------------------------------
函 数 名：void sendPictureToClient(void* picture, u16 pictureNum)
函数功能：向客户端发送自定义图像
------------------------------------------------------------------------*/
void sendPictureToClient(void *picture, u16 pictureNum)
{
	u16 dataLength = 0;
	dataLength = 6 + pictureNum * 15;
	switch (pictureNum) // 命令ID
	{
	case 1:
		G_ST_Student_Interactive_Header_Data.data_cmd_id = 0x0101;
		break;
	case 2:
		G_ST_Student_Interactive_Header_Data.data_cmd_id = 0x0102;
		break;
	case 5:
		G_ST_Student_Interactive_Header_Data.data_cmd_id = 0x0103;
		break;
	case 7:
		G_ST_Student_Interactive_Header_Data.data_cmd_id = 0x0104;
		break;
	default:
		break;
	}

	G_ST_Student_Interactive_Header_Data.sender_ID = Sender_ID; // 发送方ID
	G_ST_Student_Interactive_Header_Data.receiver_ID = Receiver_Client_ID;

	Client_DataBuf[0] = 0xA5;											  // 帧头
	Client_DataBuf[1] = (u8)dataLength;									  // 数据长度低8位
	Client_DataBuf[2] = (u8)(dataLength >> 8);							  // 数据长度高8位
	Client_DataBuf[3] = 0x00;											  // Seq
	Append_CRC8_Check_Sum(Client_DataBuf, 5);							  // CRC8
	Client_DataBuf[5] = 0x01;											  // cmd_id低八位
	Client_DataBuf[6] = 0x03;											  // cmd_id高八位
	memcpy(&Client_DataBuf[7], &G_ST_Student_Interactive_Header_Data, 6); // Header
	memcpy(&Client_DataBuf[13], picture, pictureNum * 15);				  // 数据
	Append_CRC16_Check_Sum(Client_DataBuf, dataLength + 9);				  // CRC16

	/*--------------------------开启DMA发送---------------------------*/

	DMA_Cmd(DMA1_Stream6, DISABLE); // 关闭DMA传输
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE)
		;										   // 确保DMA可以被设置
	DMA1_Stream6->M0AR = (UINT32)(Client_DataBuf); // 设置地址
	DMA1_Stream6->NDTR = dataLength + 9;		   // 数据传输量
	DMA_Cmd(DMA1_Stream6, ENABLE);				   // 开启DMA传输
												   // 启用串口DMA发送
}

bool testf = FALSE;
float atest = 0;
float btest = 0;
UCHAR8 testflag = 3;
UINT32 newpicture_cnt = 0;
UN_GameRobotHP Robo_HP;
float hp_print=0;
////stGameRobotHPInfo
void Send_Custom_Data(void)
{
	static bool IfFirstDraw = TRUE; // 是否为第一次画
//	static u8 Clinet_Link = 0;
//	if (RSYS_Monitor.GameStatus_fps != 0) // 裁判系统是否连接到服务器
//		Clinet_Link = 1;
	
	if((int)(unAimData.stEnemyE.target_num)==0)
                {
                    hp_print = 0;
                }

	
	if (system_monitor.USART2rx_fps > 0)
	{
		if (IfFirstDraw)
		{
			static u8 cnt1 = 0;
			if (cnt1 == 0)
			{
				// 图层0
				sendLobToClient(); // 画准星
				cnt1++;
			}
			else if (cnt1 == 1)
			{
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[0], floatDataName[9], 1, 6, 6, 2, 15, 3, 2, 1200, 750-22 ,infantry_number, infantry_number >> 10, infantry_number >> 21); // TODO																	
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], floatDataName[10], 1, 6, 7, 2, 15, 0, 2, 960, 750-22, spinning_flag, spinning_flag >> 10, spinning_flag >> 21);
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Double, 2);
				cnt1++;
			}
//			
			else if (cnt1 == 2)
			{
				// 图层3 字符
				pictureOperation(&G_ST_Client_Custom_Character.grapic_data_struct, AimStar_X8, 1, 7, 3, 2, 15, 30, 2, 1400, 900 - 114, 0, 0, 0);
				G_ST_Client_Custom_Character.data[0] = 'V';
				G_ST_Client_Custom_Character.data[1] = 'i';
				G_ST_Client_Custom_Character.data[2] = 's';
				G_ST_Client_Custom_Character.data[3] = 'i';
				G_ST_Client_Custom_Character.data[4] = 'o';
				G_ST_Client_Custom_Character.data[5] = 'n';
				G_ST_Client_Custom_Character.data[7] = '\n';
				
				G_ST_Client_Custom_Character.data[8] = 's';
				G_ST_Client_Custom_Character.data[9] = 'p';
				G_ST_Client_Custom_Character.data[10] = 'i';
				G_ST_Client_Custom_Character.data[11] = 'n';				
				G_ST_Client_Custom_Character.data[12] = '\n';
				
				G_ST_Client_Custom_Character.data[13] = 'L';
				G_ST_Client_Custom_Character.data[14] = 'o';
				G_ST_Client_Custom_Character.data[15] = 'c';
				G_ST_Client_Custom_Character.data[16] = 'k';
				G_ST_Client_Custom_Character.data[17] = '\n';
				
				G_ST_Client_Custom_Character.data[18] = 'D';
				G_ST_Client_Custom_Character.data[19] = 'i';
				G_ST_Client_Custom_Character.data[20] = 's';
				G_ST_Client_Custom_Character.data[21] = 't';
				G_ST_Client_Custom_Character.data[22] = 'a';
				G_ST_Client_Custom_Character.data[23] = 'n';
				G_ST_Client_Custom_Character.data[24] = 'c';
				G_ST_Client_Custom_Character.data[25] = 'e';
				G_ST_Client_Custom_Character.data[26] = '\n';
				
				G_ST_Client_Custom_Character.data[27] = ' ';
				G_ST_Client_Custom_Character.data[28] = ' ';
				G_ST_Client_Custom_Character.data[29] = ' ';

				

				sendCharacterToClient(&G_ST_Client_Custom_Character);
				cnt1++;
			}
			else if (cnt1 == 3)
			{
				// 图层4浮点
				pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[0], floatDataName[0], 1, 5, 4, 2, 15, 3, 2, 1540, 720, (int)(unAimData.stEnemyE.Distance * 1000), (int)(unAimData.stEnemyE.Distance * 1000) >> 10, (int)(unAimData.stEnemyE.Distance * 1000) >> 21);
				pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[1], floatDataName[1], 1, 5, 4, 2, 15, 3, 2, 960, 750, (int)(g_stFriction1SMC.fpFB * 1000), (int)(g_stFriction1SMC.fpFB * 1000) >> 10, (int)(g_stFriction1SMC.fpFB * 1000) >> 21);
				pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[2], floatDataName[2], 1, 5, 4, 2, 15, 3, 2, 1200, 400, (int)(Pitch_PosPID.fpFB * 1000), (int)(Pitch_PosPID.fpFB * 1000) >> 10, (int)(Pitch_PosPID.fpFB * 1000) >> 21);
				pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[3], floatDataName[3], 1, 5, 4, 8, 15, 0, 2, 960, 750 + 22, (int)(YawBaseCnt * Yaw_Compensation_Step_Vision * 1000), (int)(YawBaseCnt * Yaw_Compensation_Step_Vision * 1000) >> 10, (int)(YawBaseCnt * Yaw_Compensation_Step_Vision * 1000) >> 21);
				pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[4], floatDataName[4], 1, 5, 4, 8, 15, 0, 2, 1200, 372 + 50, (int)(PitchBaseCnt * Pitch_Compensation_Step_Vision * 1000), (int)(PitchBaseCnt * Pitch_Compensation_Step_Vision * 1000) >> 10, (int)(PitchBaseCnt * Pitch_Compensation_Step_Vision * 1000) >> 21);
								
				
				
				pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[5], floatDataName[5], 1, 5, 4, 0, 15, 0, 2, 1020,  680, (int)(hp_print*1000), (int)(hp_print*1000) >> 10, (int)(hp_print*1000) >> 21);
				pictureOperation(&G_ST_Client_Custom_Graphic_Seven.grapic_data_struct[6], floatDataName[6], 1, 5, 4, 2, 15, 0, 2, 1080, 490, (int)(Yaw_E*1000), (int)(Yaw_E*1000) >> 10, (int)(Yaw_E*1000) >> 21);
				
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Seven, 7);
				cnt1++;
			}
			else if (cnt1 == 4)
			{
				// 图层5
//////////////////////////////////////
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[0], floatDataName[5], 1, 5, 5, 2, 15, 3, 2, 900, 680, (int)(unAimData.stEnemyE.target_num * 1000), (int)(unAimData.stEnemyE.target_num * 1000) >> 10, (int)(unAimData.stEnemyE.target_num * 1000) >> 21); // TODO
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], floatDataName[6], 1, 6, 5, 8, 15, 0, 2, 1540, 900 - 28 * 4, Gimbal_Control, Gimbal_Control >> 10, Gimbal_Control >> 21);																								   // TODO
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Double, 2);
				cnt1++;
			}
			else if (cnt1 == 5)
			{
				// 图层6
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[0], floatDataName[7], 1, 5, 6, 2, 15, 3, 2, 1200, 750 , (int)(g_stFriction1SMC.fpFB*1000), (int)(g_stFriction1SMC.fpFB*1000) >> 10, (int)(g_stFriction1SMC.fpFB*1000) >> 21); // TODO
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], floatDataName[8], 1, 6, 6, 2, 15, 0, 2, 1200, 750+22,    FrictionWheel_Ready, FrictionWheel_Ready >> 10, FrictionWheel_Ready >> 21);																								   // TODO
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Double, 2);
				cnt1++;
			}
			else if (cnt1 == 6)
			{
				// 图层7

				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[0], Float6Name, 1, 6, 7, 8, 15, 0, 2, 1540, 745, vision_lock, vision_lock >> 10, vision_lock >> 21);
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], Float7Name, 1, 6, 7, 8, 15, 0, 2, 1540, 766, spinning_flag, spinning_flag >> 10, spinning_flag >> 21);
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Double, 2);
				cnt1++;
			}
			else if (cnt1 == 7)
			{
				pictureOperation(&G_ST_Client_Custom_Character.grapic_data_struct, AimStar_X10, 1, 7, 8, 2, 15, 30, 2, 845, 680, 0, 0, 0);
				G_ST_Client_Custom_Character.data[0] = 'A';
				G_ST_Client_Custom_Character.data[1] = 'i';
				G_ST_Client_Custom_Character.data[2] = 'm';
				G_ST_Client_Custom_Character.data[3] = ' ';
				G_ST_Client_Custom_Character.data[4] = ' ';
				G_ST_Client_Custom_Character.data[5] = ' ';
				G_ST_Client_Custom_Character.data[6] = ' ';
				G_ST_Client_Custom_Character.data[7] = ' ';
				G_ST_Client_Custom_Character.data[8] = ' ';
				G_ST_Client_Custom_Character.data[9] = ' ';
				G_ST_Client_Custom_Character.data[10] = ' ';
				G_ST_Client_Custom_Character.data[11] = ' ';
				G_ST_Client_Custom_Character.data[12] = ' ';
				G_ST_Client_Custom_Character.data[13] = ' ';
				G_ST_Client_Custom_Character.data[14] = ' ';
				G_ST_Client_Custom_Character.data[15] = ' ';
				G_ST_Client_Custom_Character.data[16] = ' ';
				G_ST_Client_Custom_Character.data[17] = ' ';
				G_ST_Client_Custom_Character.data[18] = ' ';
				G_ST_Client_Custom_Character.data[19] = ' ';
				G_ST_Client_Custom_Character.data[20] = ' ';
				G_ST_Client_Custom_Character.data[21] = ' ';
				G_ST_Client_Custom_Character.data[22] = ' ';
				G_ST_Client_Custom_Character.data[23] = ' ';
				G_ST_Client_Custom_Character.data[24] = ' ';
				G_ST_Client_Custom_Character.data[25] = ' ';
				G_ST_Client_Custom_Character.data[26] = ' ';
				G_ST_Client_Custom_Character.data[27] = ' ';
				G_ST_Client_Custom_Character.data[28] = ' ';
				G_ST_Client_Custom_Character.data[29] = ' ';

				sendCharacterToClient(&G_ST_Client_Custom_Character);
				cnt1++;
			}
			else if (cnt1 == 8)
			{
				sendRectangleToClient();
				cnt1++;
				cnt1 = 0;
			}
		}
		else
		{ // 不是第一次画修改即可
			static u8 cnt = 0;
//			if (cnt == 0)
//			{
//				// 修改图层2的数字
//				sendDataToClient(Gimbal_Control, FrictionWheel_Ready, Sending_num, d_or_s_flag, shooting_or_not_s);
//				cnt++;
//			}
			if (cnt == 0)
			{
				// 修改图层4的数字
				//sendFloatToClient(unAimData.stEnemyE.Distance, Yaw_PosPID.fpFB, Pitch_PosPID.fpFB, YawBaseCnt * Yaw_Compensation_Step_Vision, PitchBaseCnt * Pitch_Compensation_Step_Vision);
				sendSEVENFloatToClient(unAimData.stEnemyE.Distance, Yaw_PosPID.fpFB, Pitch_PosPID.fpFB, YawBaseCnt * Yaw_Compensation_Step_Vision, PitchBaseCnt * Pitch_Compensation_Step_Vision,hp_print,Yaw_E);
				cnt++;
			}
			else if (cnt == 1)
			{
				// 图层5																				  操作 类型 层数 颜色 起始角 终止角 线宽  起x     起y                  字体大小                                           终x                                                       终y     
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[0], floatDataName[5], 2, 5,  5,   8,   15,     3,     2,    900,  680, (int)(unAimData.stEnemyE.target_num * 1000), (int)(unAimData.stEnemyE.target_num * 1000) >> 10, (int)(unAimData.stEnemyE.target_num * 1000) >> 21);
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], floatDataName[6], 2, 6,  5,   8,   15,     0,     2,    1540,   900 - 28 * 4,  Gimbal_Control, Gimbal_Control >> 10, Gimbal_Control >> 21); // 前哨站
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Double, 2);
				cnt++;
			}
			else if (cnt == 2)
			{
				// 修改图层7的数字
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[0], Float6Name, 2, 6, 7, 8, 15, 0, 2, 1540, 745, vision_lock, vision_lock >> 10, vision_lock >> 21);
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], Float7Name, 2, 6, 7, 8, 15, 0, 2, 1540, 766, spinning_flag, spinning_flag >> 10, spinning_flag >> 21);
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Double, 2);
				cnt++;
			}
			else if (cnt == 3)
			{
				// 图层5
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[0], floatDataName[7], 2, 5, 6, 8, 15, 3, 2, 1200, 750 , (int)(g_stFriction1SMC.fpFB*1000), (int)(g_stFriction1SMC.fpFB*1000) >> 10, (int)(g_stFriction1SMC.fpFB*1000) >> 21); // TODO
				pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], floatDataName[8], 2, 6, 6, 8, 15, 0, 2, 1200, 750+22,    FrictionWheel_Ready, FrictionWheel_Ready >> 10, FrictionWheel_Ready >> 21);										
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Double, 2);
				cnt++;
			}
            else if (cnt == 4)
            {
                pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[0], floatDataName[9], 2, 6, 6, 2, 15, 3, 2, 1200, 750-22 ,infantry_number, infantry_number >> 10, infantry_number >> 21); // TODO
                if(spinning_flag == 1)
                {
			           pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], floatDataName[10], 2, 6, 7, 4, 15, 0, 2, 960, 750-22, spinning_flag, spinning_flag >> 10, spinning_flag >> 21);
                }
                else 
                {
                 pictureOperation(&G_ST_Client_Custom_Graphic_Double.grapic_data_struct[1], floatDataName[10], 2, 6, 7, 2, 15, 0, 2, 960, 750-22, spinning_flag, spinning_flag >> 10, spinning_flag >> 21);
                }
				sendPictureToClient(&G_ST_Client_Custom_Graphic_Double, 2);
                cnt ++;
            }
            else if (cnt == 5)
			{
                if(unAimData.stEnemyE.lock == 1)
                {
				pictureOperation(&G_ST_Client_Custom_Graphic_Single.grapic_data_struct, AimStar_X12, 2, 1, 0, 4, 0, 0, 2, 850, 500, 0, 1070, 650);
                }
	            else  if(unAimData.stEnemyE.lock == 0)

                {
                pictureOperation(&G_ST_Client_Custom_Graphic_Single.grapic_data_struct, AimStar_X12, 2, 1, 0, 1, 0, 0, 2, 850, 500, 0, 1070, 650); 
                }                    
                sendPictureToClient(&G_ST_Client_Custom_Graphic_Single, 1);
				cnt = 0;
			}
		}
	}
	else
	{
		deletePictureOnClient(2, 0); // 删除所有图形
		IfFirstDraw = TRUE;			 // 准备重新画图
	}
	if (newpicture_cnt > 200)
	{
		IfFirstDraw = TRUE;
		newpicture_cnt = 0;
	}
	else
		newpicture_cnt++;
	IfFirstDraw = testf;
}

/*****************************************************机器人之间的交互****************************************************************/

/*------------------------------------------
函 数 名：Send_Sentinel_Data(void)
函数功能：机器人间通信 发送给哨兵，重启疯了的哨兵，改变哨兵下云台的运动方式
-------------------------------------------*/
void Send_Sentinel_Data(void)
{
	/*-----------------------------------帧头----------------------------------------*/
	Custom_DataBuf[0] = 0xA5;
	Custom_DataBuf[1] = (UCHAR8)9; // 通讯出错可以尝试排查这里
	Custom_DataBuf[2] = (UCHAR8)9 >> 8;
	Custom_DataBuf[3] = 0x00; // 包序号,不知道是用来干嘛的
	Append_CRC8_Check_Sum(Custom_DataBuf, 5);

	/*--------------------------------cmd_id-----------------------------------------*/
	Custom_DataBuf[5] = 0x01;
	Custom_DataBuf[6] = 0x03;

	/*--------------------------------自定义数据-------------------------------------*/
	G_ST_Student_Interactive_Header_Data.data_cmd_id = Robo_Data_Cmd_ID;
	G_ST_Student_Interactive_Header_Data.sender_ID = Sender_ID;
	G_ST_Student_Interactive_Header_Data.receiver_ID = Receiver_Sentinel_ID;
	//  	SendData_Sentinel[0] = sentinel_reset;
	//  	SendData_Sentinel[1] = sentinel_Gimbal;
	SendData_Sentinel[2] = sentinel_change_id;
	memcpy(&Custom_DataBuf[7], &G_ST_Student_Interactive_Header_Data, 6);
	memcpy(&Custom_DataBuf[13], SendData_Sentinel, 3);
	Append_CRC16_Check_Sum(Custom_DataBuf, 18);

	/*---------------------------------开启发送--------------------------------------*/

	DMA_Cmd(DMA1_Stream6, DISABLE); // 关闭DMA传输
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE)
		;										   // 确保DMA可以被设置
	DMA1_Stream6->M0AR = (UINT32)(Custom_DataBuf); // 设置地址
	DMA1_Stream6->NDTR = 18;					   // 数据传输量
	DMA_Cmd(DMA1_Stream6, ENABLE);				   // 开启DMA传输
}

/*------------------------------------------
函 数 名：Send_Radar_Data(void)
函数功能：机器人间通信 发送给雷达，切换界面
-------------------------------------------*/
void Send_Radar_Data(void)
{
	/*-----------------------------------帧头----------------------------------------*/
	Custom_DataBuf[0] = 0xA5;
	Custom_DataBuf[1] = (UCHAR8)Robo_Data_Length; // 通讯出错可以尝试排查这里
	Custom_DataBuf[2] = (UCHAR8)Robo_Data_Length >> 8;
	Custom_DataBuf[3] = 0x00; // 包序号,不知道是用来干嘛的
	Append_CRC8_Check_Sum(Custom_DataBuf, 5);

	/*--------------------------------cmd_id-----------------------------------------*/
	Custom_DataBuf[5] = 0x01;
	Custom_DataBuf[6] = 0x03;

	/*--------------------------------自定义数据-------------------------------------*/
	G_ST_Student_Interactive_Header_Data.data_cmd_id = Robo_Data_Cmd_ID;
	G_ST_Student_Interactive_Header_Data.sender_ID = Sender_ID;
	G_ST_Student_Interactive_Header_Data.receiver_ID = Receiver_Radar_ID;
	G_ST_Robot_Interactive_Data.m_SendData = picture_num;
	memcpy(&Custom_DataBuf[7], &G_ST_Student_Interactive_Header_Data, 6);
	memcpy(&Custom_DataBuf[13], &G_ST_Robot_Interactive_Data, 1);
	Append_CRC16_Check_Sum(Custom_DataBuf, 16);

	/*---------------------------------开启发送--------------------------------------*/
	DMA_Cmd(DMA1_Stream6, DISABLE); // 关闭DMA传输
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE)
		;										   // 确保DMA可以被设置
	DMA1_Stream6->M0AR = (UINT32)(Custom_DataBuf); // 设置地址
	DMA1_Stream6->NDTR = 16;					   // 数据传输量
	DMA_Cmd(DMA1_Stream6, ENABLE);				   // 开启DMA传输
}
/*------------------------------------------
函 数 名：Send_Missile_Data(void)
函数功能：机器人间通信 发送给导弹，发送的颗数
-------------------------------------------*/
void Send_Missile_Data(void)
{
	/*-----------------------------------帧头----------------------------------------*/
	Custom_DataBuf[0] = 0xA5;
	Custom_DataBuf[1] = (UCHAR8)Robo_Data_Length; // 通讯出错可以尝试排查这里
	Custom_DataBuf[2] = (UCHAR8)Robo_Data_Length >> 8;
	Custom_DataBuf[3] = 0x00; // 包序号,不知道是用来干嘛的
	Append_CRC8_Check_Sum(Custom_DataBuf, 5);

	/*--------------------------------cmd_id-----------------------------------------*/
	Custom_DataBuf[5] = 0x01;
	Custom_DataBuf[6] = 0x03;

	/*--------------------------------自定义数据-------------------------------------*/
	G_ST_Student_Interactive_Header_Data.data_cmd_id = Robo_Data_Cmd_ID;
	G_ST_Student_Interactive_Header_Data.sender_ID = Sender_ID;
	G_ST_Student_Interactive_Header_Data.receiver_ID = Receiver_Missile_ID;
	G_ST_Robot_Interactive_Data.m_SendData = Sending_num; // 我方要发射导弹的数量
	memcpy(&Custom_DataBuf[7], &G_ST_Student_Interactive_Header_Data, 6);
	memcpy(&Custom_DataBuf[13], &G_ST_Robot_Interactive_Data, 1);
	Append_CRC16_Check_Sum(Custom_DataBuf, 16);

	/*---------------------------------开启发送--------------------------------------*/
	DMA_Cmd(DMA1_Stream6, DISABLE); // 关闭DMA传输
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE)
		;										   // 确保DMA可以被设置
	DMA1_Stream6->M0AR = (UINT32)(Custom_DataBuf); // 设置地址
	DMA1_Stream6->NDTR = 16;					   // 数据传输量
	DMA_Cmd(DMA1_Stream6, ENABLE);				   // 开启DMA传输
}

void Press_Y_N(void)
{
	if (remote_control_image_transmission.left_button_down & 0x01)
		PRESSED_ML = TRUE;
	else
		PRESSED_ML = FALSE; // left为1则TRUL
	if (remote_control_image_transmission.right_button_down & 0x01)
		PRESSED_MR = TRUE;
	else
		PRESSED_MR = FALSE;
	/*键盘按键控制底盘及其他任务*/
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_W)
		PRESSED_W = TRUE;
	else
		PRESSED_W = FALSE; // 按下及改变TRUE
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_S)
		PRESSED_S = TRUE;
	else
		PRESSED_S = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_A)
		PRESSED_A = TRUE;
	else
		PRESSED_A = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_D)
		PRESSED_D = TRUE;
	else
		PRESSED_D = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_Q)
		PRESSED_Q = TRUE;
	else
		PRESSED_Q = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_E)
		PRESSED_E = TRUE;
	else
		PRESSED_E = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_R)
		PRESSED_R = TRUE;
	else
		PRESSED_R = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_F)
		PRESSED_F = TRUE;
	else
		PRESSED_F = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_G)
		PRESSED_G = TRUE;
	else
		PRESSED_G = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_Z)
		PRESSED_Z = TRUE;
	else
		PRESSED_Z = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_X)
		PRESSED_X = TRUE;
	else
		PRESSED_X = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_C)
		PRESSED_C = TRUE;
	else
		PRESSED_C = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_V)
		PRESSED_V = TRUE;
	else
		PRESSED_V = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_B)
		PRESSED_B = TRUE;
	else
		PRESSED_B = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_SHIFT)
		PRESSED_SHIFT = TRUE;
	else
		PRESSED_SHIFT = FALSE;
	if (remote_control_image_transmission.keyboard_value & KEY_PRESSED_OFFSET_CTRL)
		PRESSED_CTRL = TRUE;
	else
		PRESSED_CTRL = FALSE;
}

/*---------------------------------------------------
函数名称：Rc_RsysProtocol
功    能：裁判系统接收数据校验
---------------------------------------------------*/
void Rc_RsysProtocol_U6(USHORT16 rxSize)
{
//	static UCHAR8 DMA2_Rx_Status = USART6_RX_FREE;
//	int i;
	
	if(UA6RxMailbox[0] == 0xA9 && UA6RxMailbox[1] == 0x53 && Verify_CRC16_Check_Sum(UA6RxMailbox, 21))
	{
		
	MonitorDataDeal_U6(ucCmdID);
				
	remote_control_image_transmission.ch_0 = ((UA6RxMailbox[2] | (UA6RxMailbox[3] << 8)) & 0x07ff);        //Channe0――水平通道
	
    remote_control_image_transmission.ch_1 = ((UA6RxMailbox[3] >> 3) | (UA6RxMailbox[4] << 5)) & 0x07ff; //Channe1――垂直通道
    /*左摇杆*/
    remote_control_image_transmission.ch_2 = ((UA6RxMailbox[4] >> 6) | (UA6RxMailbox[5] << 2) | (UA6RxMailbox[6] << 10)) & 0x07ff; //Channe2――水平通道
    remote_control_image_transmission.ch_3 = ((UA6RxMailbox[6] >> 1) | (UA6RxMailbox[7] << 7)) & 0x07ff;                              //Channe3――垂直通道
    /*拨轮*/
	remote_control_image_transmission.wheel = (UA6RxMailbox[8] >> 1 | UA6RxMailbox[9] << 7) & 0x07ff; //Channe4――垂直通道

    /*左3位开关*/
	remote_control_image_transmission.mode_sw = ((UA6RxMailbox[7] >> 4) & 0x0003); //(上――1；中――3；下――2)

    /*返航键*/
    remote_control_image_transmission.go_home = ((UA6RxMailbox[7] >> 6) & 0x0001); //(按下――1；未按下――0)

    /*自定义按键*/
	remote_control_image_transmission.fn = ((UA6RxMailbox[7] >> 7) & 0x0001); //(按下――1；未按下――0)

    /*拍照切换按键*/
	remote_control_image_transmission.button = ((UA6RxMailbox[8]) & 0x0001); //(按下――1；未按下――0)

    /*拍照/录像按键 */
	remote_control_image_transmission.shutter = ((UA6RxMailbox[9]) >> 4 & 0x0001); //(按下――1；未按下――0)

    /**************************************************鼠标数据解码***************************************************/
	remote_control_image_transmission.mouse_x = UA6RxMailbox[10] | (UA6RxMailbox[11] << 8);   // X坐标

	remote_control_image_transmission.mouse_y = UA6RxMailbox[12] | (UA6RxMailbox[13] << 8); //Y坐标

	remote_control_image_transmission.mouse_z = UA6RxMailbox[14] | (UA6RxMailbox[15] << 8); //Z坐标

	remote_control_image_transmission.left_button_down = (UA6RxMailbox[16] & 0x03);                  //左键状态（1：按下；0：没按下）

	   remote_control_image_transmission.right_button_down = (UA6RxMailbox[16] >> 2 & 0x03);            //右键状态（1：按下；0：没按下）

	   remote_control_image_transmission.middle_button_down = (UA6RxMailbox[16] >> 4 & 0x03);              //中键状态（1：按下；0：没按下）

    /**************************************************键盘数据解码***************************************************/
	   remote_control_image_transmission.keyboard_value = UA6RxMailbox[17] | (UA6RxMailbox[18] << 8); //键盘值
		
	}
	
//	for (i = 0; i < rxSize; i++)
//	{
//		usData = UA6RxMailbox[i];
//		switch (DMA2_Rx_Status)
//		{
//		case USART6_RX_FREE:
//			if (usData == 0xA5)
//			{
//				Buf_num = 0;
//				DMA2_Rx_Status = USART6_RX_Length;
//				USART6_Rx_Buf[Buf_num++] = 0xA5; // 自由状态下接收到0xA5认为接收开始，先赋值，再算自加
//			}
//			else
//				DMA2_Rx_Status = USART6_RX_FREE;
//			break;
//		case USART6_RX_Length:
//			if (Buf_num <= 4)
//				USART6_Rx_Buf[Buf_num++] = usData; // 接收帧头全部数据
//			else if (Buf_num > 4)
//			{
//				usDataLength = USART6_Rx_Buf[1] | USART6_Rx_Buf[2] << 8; // 接收到的帧的数据量（data长度）
//				Seq = USART6_Rx_Buf[3];
//				CRC8 = USART6_Rx_Buf[4]; // 帧头CRC校验
//				USART6_Rx_Buf[Buf_num++] = usData;
//				if (Verify_CRC8_Check_Sum(USART6_Rx_Buf, 5) == 1)
//				{
//					DMA2_Rx_Status = USART6_RX_CRC16; // 帧头校验成功开启数据接收
//				}
//				else
//					DMA2_Rx_Status = USART6_RX_FREE;
//			}
//			break;
//		case USART6_RX_CRC16:
//			if (Buf_num < (9 + usDataLength))
//			{
//				USART6_Rx_Buf[Buf_num++] = usData;
//			}
//			if (Buf_num >= (9 + usDataLength))
//			{
//				Buf_num = 0;
//				DMA2_Rx_Status = USART6_RX_FREE;
//				if (Verify_CRC16_Check_Sum(USART6_Rx_Buf, usDataLength + 9))
//				{
//					ucCmdID = USART6_Rx_Buf[5] | (USART6_Rx_Buf[6] << 8);
//					MonitorDataDeal_U6(ucCmdID);
//				}
//			}
//			break;
//		default:
//			break;
//		}
//	}

}

/*---------------------------------------------
函 数 名：MonitorDataDeal
函数功能：处理接收到的裁判系统数据
备    注：
---------------------------------------------*/
void MonitorDataDeal_U6(USHORT16 usCmdID)
{
	switch (usCmdID)
	{

	case ROBOT_COMMAND_CMD_ID:

	{
		//memcpy(&remote_control_image_transmission, &USART6_Rx_Buf[7], sizeof(remote_control_image_transmission));
				
		//memcpy(&remote_control_new, &USART6_Rx_Buf[7], sizeof(remote_control_new));

		/////////////////////////****************************///////////////////////////////
		
		
	remote_control_image_transmission.ch_0 = ((UA6RxMailbox[2] | (UA6RxMailbox[3] << 8)) & 0x07ff);        //Channe0――水平通道
	
    remote_control_image_transmission.ch_1 = ((UA6RxMailbox[3] >> 3) | (UA6RxMailbox[4] << 5)) & 0x07ff; //Channe1――垂直通道
    /*左摇杆*/
    remote_control_image_transmission.ch_2 = ((UA6RxMailbox[4] >> 6) | (UA6RxMailbox[5] << 2) | (UA6RxMailbox[6] << 10)) & 0x07ff; //Channe2――水平通道
    remote_control_image_transmission.ch_3 = ((UA6RxMailbox[6] >> 1) | (UA6RxMailbox[7] << 7)) & 0x07ff;                              //Channe3――垂直通道
    /*拨轮*/
	remote_control_image_transmission.wheel = (UA6RxMailbox[8] >> 1 | UA6RxMailbox[9] << 7) & 0x07ff; //Channe4――垂直通道

    /*左3位开关*/
	remote_control_image_transmission.mode_sw = ((UA6RxMailbox[7] >> 4) & 0x0003); //(上――1；中――3；下――2)

    /*返航键*/
    remote_control_image_transmission.go_home = ((UA6RxMailbox[7] >> 6) & 0x0001); //(按下――1；未按下――0)

    /*自定义按键*/
	remote_control_image_transmission.fn = ((UA6RxMailbox[7] >> 7) & 0x0001); //(按下――1；未按下――0)

    /*拍照切换按键*/
	remote_control_image_transmission.button = ((UA6RxMailbox[8]) & 0x0001); //(按下――1；未按下――0)

    /*拍照/录像按键 */
	remote_control_image_transmission.shutter = ((UA6RxMailbox[9]) >> 4 & 0x0001); //(按下――1；未按下――0)

    /**************************************************鼠标数据解码***************************************************/
	remote_control_image_transmission.mouse_x = UA6RxMailbox[10] | (UA6RxMailbox[11] << 8);   // X坐标

	remote_control_image_transmission.mouse_y = UA6RxMailbox[12] | (UA6RxMailbox[13] << 8); //Y坐标

	remote_control_image_transmission.mouse_z = UA6RxMailbox[14] | (UA6RxMailbox[15] << 8); //Z坐标

	remote_control_image_transmission.left_button_down = (UA6RxMailbox[16] & 0x03);                  //左键状态（1：按下；0：没按下）

	   remote_control_image_transmission.right_button_down = (UA6RxMailbox[16] >> 2 & 0x03);            //右键状态（1：按下；0：没按下）

	   remote_control_image_transmission.middle_button_down = (UA6RxMailbox[16] >> 4 & 0x03);              //中键状态（1：按下；0：没按下）

    /**************************************************键盘数据解码***************************************************/
	   remote_control_image_transmission.keyboard_value = UA6RxMailbox[17] | (UA6RxMailbox[18] << 8); //键盘值


	}
	break;

	default:
		break;
	}
}
