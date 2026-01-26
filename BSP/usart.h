#ifndef __USART_H__
#define __USART_H__
#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "rm_types_my.h"


#define   DMA1_Stream5_BufLength  1024
#define   Deal_BufLength          128
#define   UA3RxDMAbuf_LEN  128  //ต๗สิ
#define   UA3TxDMAbuf_LEN  128  //ต๗สิ
void USART1_Configuration(void);
void USART2_Configuration(void);
void USART3_Configuration(void);
void UART4_Configuration(void);
void USART6_Configuration(void);


extern USART_RX_TypeDef USART6_Rcr;
extern USART_RX_TypeDef USART3_Rcr;
extern USART_RX_TypeDef USART2_Rcr;
extern USART_TX_TypeDef USART2_Tcr;
extern USART_RX_TypeDef UART4_Rcr;

extern UCHAR8 UA6RxMailbox[USART6_RXMB_LEN] ;


#endif
