#ifndef  __USART_PROTOCOL_H__
#define  __USART_PROTOCOL_H__
#include "rm_types_my.h"
#include "global_declare.h"
//#include "RefereeSys.h"
#include "GlobalUse_Basic_Function.h"

#define Color_Type   ((Drone_State.robot_id==6)? 1:0)  //红方返回1，蓝方返回0；
#define COMM6_RX_FREE_1         0
#define COMM6_RX_FREE_2         1
#define COMM6_RX_FLAG			2
#define COMM6_RX_START          3
#define COMM6_RX_DATA           4
#define COMM6_RX_END            5

#define MaxLength               16
#define Visual_Data_Size		16					
#define EMERGENCEY_SHOTDOWN     0x01 //重启
#define FOCUS_ID				0x02 //辅助瞄准ID	
#define FOCUS_DATA_LENGTH	    4

USHORT16 USART_Receive(USART_RX_TypeDef* USARTx);
void USART_Transmit(USART_TX_TypeDef* USARTx);
void rx_handle(USART_RX_TypeDef* USARTx_Rcr);
void Vision_Tx_Protocol(void);
void Vision_Rx_Protocol(void);
void usart_tx_handle(USART_TypeDef* USARTx, UCHAR8* pTxBuf, UCHAR8 length);
void USART_data_deal(void);
void PitchAngleUpdate(void);
void Assign_Bit(u8* ADR,u8 num, u8 IF_1);
void Send_Data_Update(void);

extern ST_VISION G_ST_Vision;
extern bool TOP_Flag,UD_FLAG;
extern void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
extern uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
extern u8 UA4RxMailbox[UART4_RXMB_LEN];
extern u8 AIM_mode;
extern u8 shooting_or_not_s;//操作手是否打弹
extern u8 danger_flag;//操作手是否锁基地，0 不锁，1 锁
extern u8 sentry_spinning;//哨兵是否小陀螺
extern Send_Data_Vofa Send;

#endif
