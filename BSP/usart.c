#include "usart.h"
#include "sys.h"
#include "global_declare.h"
#if SYSTEM_SUPPORT_OS
// #include "FreeRTOS.h"
#endif
// 加入以下代码,支持printf函数,而不需要选择use MicroLIB
#pragma import(__use_no_semihosting)

void _ttywrch(int ch)
{
	ch = ch;
}

// 标准库需要的支持函数
struct __FILE
{
	int handle;
};

FILE __stdout;

// 定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}

// 重定义fputc函数
int fputc(int ch, FILE *f)
{
	while ((UART4->SR & 0X40) == 0)
		; // 循环发送,直到发送完毕
	UART4->DR = (u8)ch;
	return ch;
}

/*
axm = kx*ax + ax0;

ax = (asm - ax0)/kx

ax^2+ay^2+az^2 = 1;
*/

__IO UCHAR8 UA2RxDMAbuf[USART2_RXDMA_LEN] = {0};
UCHAR8 UA2RxMailbox[USART2_RXMB_LEN] = {0};
USART_RX_TypeDef USART2_Rcr = {USART2, USART2_RX_STREAM, UA2RxMailbox, UA2RxDMAbuf, USART2_RXMB_LEN, USART2_RXDMA_LEN, 0, 0, 0};

extern UCHAR8 Custom_DataBuf[128];
/*************************************************************************
函 数 名：USART2_Configuration
函数功能：串口2底层配置
备    注：裁判系统
*************************************************************************/
void USART2_Configuration(void)
{
	USART_InitTypeDef usart2;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE); // 使能PA端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);						// 使能USART2时钟

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_AF;		 // 复用模式
	gpio.GPIO_OType = GPIO_OType_PP;	 // 推挽输出
	gpio.GPIO_Speed = GPIO_Speed_100MHz; // IO口速度为100MHz
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio); // 根据设定参数初始化GPIOA

	/*USART2接收空闲中断*/
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 6; // 抢占优先级
	nvic.NVIC_IRQChannelSubPriority = 0;		// 子优先级
	nvic.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
	NVIC_Init(&nvic);							// 根据指定的参数初始化VIC寄存器

	/*DMA发送完成中断*/
	nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 9; // 抢占优先级
	nvic.NVIC_IRQChannelSubPriority = 0;		// 子优先级
	nvic.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
	NVIC_Init(&nvic);							// 根据指定的参数初始化VIC寄存器

	usart2.USART_BaudRate = 115200;									   // 波特率
	usart2.USART_WordLength = USART_WordLength_8b;					   // 字长为8位数据格式
	usart2.USART_StopBits = USART_StopBits_1;						   // 一个停止位
	usart2.USART_Parity = USART_Parity_No;							   // 无奇偶校验位
	usart2.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;				   // 收发模式
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_Init(USART2, &usart2);									   // 初始化串口

	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); // 使能串口空闲中断
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);

	USART_Cmd(USART2, ENABLE); // 使能串口
	// Rx
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;						  // 外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR); // 内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA2RxDMAbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // DMA传输为单向
	DMA_InitStructure.DMA_BufferSize = USART2_RXDMA_LEN;	// 设置DMA在传输区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	// Tx
	DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; // 外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Custom_DataBuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // DMA传输为单向
	DMA_InitStructure.DMA_BufferSize = NULL;				// 设置DMA在传输区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

/*************************************************************************
函 数 名：USART3_Configuration
函数功能：串口3底层配置
备    注：USART3用于无线调试
*************************************************************************/
__IO UCHAR8 UA3RxDMAbuf[USART3_RXDMA_LEN] = {0};
UCHAR8 UA3RxMailbox[USART3_RXMB_LEN] = {0};
USART_RX_TypeDef USART3_Rcr = {USART3, USART3_RX_STREAM, UA3RxMailbox, UA3RxDMAbuf, USART3_RXMB_LEN, USART3_RXDMA_LEN, 0, 0, 0};
void USART3_Configuration(void)
{
	USART_InitTypeDef usart3;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE); // 使能PB端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);						// 使能UART3时钟

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

	gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_AF;		// 复用模式
	gpio.GPIO_OType = GPIO_OType_PP;	// 推挽输出
	gpio.GPIO_Speed = GPIO_Speed_50MHz; // IO口速度为50MHz
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;	// 上拉
	GPIO_Init(GPIOB, &gpio);			// 根据设定参数初始化GPIOC

	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 7; // 抢占优先级
	nvic.NVIC_IRQChannelSubPriority = 0;		// 子优先级
	nvic.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
	NVIC_Init(&nvic);							// 根据指定的参数初始化VIC寄存器

	usart3.USART_BaudRate = 460800;									   // 波特率
	usart3.USART_WordLength = USART_WordLength_8b;					   // 字长为8位数据格式
	usart3.USART_StopBits = USART_StopBits_1;						   // 一个停止位
	usart3.USART_Parity = USART_Parity_No;							   // 无奇偶校验位
	usart3.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				   // 仅接收
	usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_Init(USART3, &usart3);									   // 初始化串口

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); // 开启空闲中断

	USART_Cmd(USART3, ENABLE); // 使能串口

	// Rx
	DMA_DeInit(DMA1_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;						  // 外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR); // 内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA3RxDMAbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // DMA传输为单向
	DMA_InitStructure.DMA_BufferSize = USART3_RXDMA_LEN;	// 设置DMA在传输区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);

	// Tx

	DMA_InitTypeDef dma;
	DMA_DeInit(DMA1_Stream3);
	while (DMA_GetCmdStatus(DMA1_Stream3) == ENABLE)
		; // 等待DMA可配置

	dma.DMA_Channel = DMA_Channel_4; // 外设地址
	dma.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
	dma.DMA_Memory0BaseAddr = NULL;
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral; // DMA传输为单向
	dma.DMA_BufferSize = NULL;				  // 设置DMA在传输区的长度
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3, &dma);

	// DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream3, DISABLE);
}

/*************************************************************************
函 数 名：UART4_Configuration
函数功能：串口4底层配置
备    注：视觉通讯
*************************************************************************/
__IO UCHAR8 UA4RxDMAbuf[UART4_RXDMA_LEN] = {0}; // volatile unsigned char
UCHAR8 UA4RxMailbox[UART4_RXMB_LEN] = {0};
USART_RX_TypeDef UART4_Rcr = {UART4, UART4_RX_STREAM, UA4RxMailbox, UA4RxDMAbuf, UART4_RXMB_LEN, UART4_RXDMA_LEN, 0, 0, 0};
void UART4_Configuration(void)
{
	USART_InitTypeDef uart4;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE); // 使能PC端口时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);						// 使能UART4时钟

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);

	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	gpio.GPIO_Mode = GPIO_Mode_AF;		// 复用模式
	gpio.GPIO_OType = GPIO_OType_PP;	// 推挽输出
	gpio.GPIO_Speed = GPIO_Speed_50MHz; // IO口速度为50MHz
	gpio.GPIO_PuPd = GPIO_PuPd_UP;		// 上拉
	GPIO_Init(GPIOA, &gpio);			// 根据设定参数初始化GPIOC

	nvic.NVIC_IRQChannel = UART4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 7; // 抢占优先级
	nvic.NVIC_IRQChannelSubPriority = 0;		// 子优先级
	nvic.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
	NVIC_Init(&nvic);							// 根据指定的参数初始化VIC寄存器

	uart4.USART_BaudRate = 460800;									  // 波特率
	uart4.USART_WordLength = USART_WordLength_8b;					  // 字长为8位数据格式
	uart4.USART_StopBits = USART_StopBits_1;						  // 一个停止位
	uart4.USART_Parity = USART_Parity_No;							  // 无奇偶校验位
	uart4.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				  // 仅接收
	uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_Init(UART4, &uart4);										  // 初始化串口

	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE); // 开启空闲中断

	USART_Cmd(UART4, ENABLE); // 使能串口

	// RX
	DMA_DeInit(DMA1_Stream2);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;							// 通道
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART4->DR);	// 外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA4RxDMAbuf;			// 将串口4接收到的数据ucRxData_DMA1_Stream2[]里，内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					// 设置数据传输方向
	DMA_InitStructure.DMA_BufferSize = UART4_RXDMA_LEN;						// 设置DMA一次传输数据量的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		// 设置外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// 设置内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 设置外设的数据长度为字节（8bits）
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			// 设置内存的数据长度为字节（8bits）
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							// DMA_Mode_Normal;////设置DMA模式为循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					// DMA_Priority_Medium;//设置DMA通道的优先级为最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2, &DMA_InitStructure);

	// DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream2, ENABLE);

	/////////////////////////TX
	DMA_InitTypeDef dma;
	DMA_DeInit(DMA1_Stream4);
	while (DMA_GetCmdStatus(DMA1_Stream4) == ENABLE)
		; // 等待DMA可配置

	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t) & (UART4->DR);
	dma.DMA_Memory0BaseAddr = NULL;			  // 暂无
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral; // 内存到外设
	dma.DMA_BufferSize = NULL;				  // 暂无
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal; // 正常发送
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream4, &dma);
	DMA_Cmd(DMA1_Stream4, DISABLE);
}
char USART3_DMA_Buf[255] = {0};
u8 USART3_DMA_Len = 0;
void USART3_DMA_printf(const char *fmt, ...) // https://www.runoob.com/cprogramming/c-function-vsprintf.html
{
	va_list ap;		   // 定义类型为va_list（可变参数列表）的指针，一般为字符
	va_start(ap, fmt); // 获取可变参数列表的第一个参数的地址
	// va_arg宏，获取可变参数的当前参数，返回指定类型并将指针指向下一参数（t参数描述了当前参数的类型）;
	// va_arg(ap,t) ( *(t *)((ap += _INTSIZEOF(t)) - _INTSIZEOF(t)) )
	USART3_DMA_Len = (u8)vsprintf(USART3_DMA_Buf, fmt, ap); // 将格式化数据打到sz字符串上，返回格式化长度
	// va_end(ap);                                              //清空可变参数列表

	//	while(DMA_GetCurrDataCounter(DMA1_Stream3));		//等之前的发完
	DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3); // 开启DMA_Mode_Normal,即便没有使用完成中断也要软件清除，否则只发一次

	DMA_Cmd(DMA1_Stream3, DISABLE);				   // 设置当前计数值前先禁用DMA
	DMA1_Stream3->M0AR = (uint32_t)USART3_DMA_Buf; // 设置当前待发数据基地址:Memory0 tARget
	DMA1_Stream3->NDTR = (uint32_t)USART3_DMA_Len; // 设置当前待发的数据的数量:Number of Data units to be TRansferred
	DMA_Cmd(DMA1_Stream3, ENABLE);				   // 启用串口DMA发
}

/*************************************************************************
函 数 名：UART6_Configuration
函数功能：串口6底层配置
备    注：备用
*************************************************************************/
__IO UCHAR8 UA6RxDMAbuf[USART6_RXDMA_LEN] = {0};
UCHAR8 UA6RxMailbox[USART6_RXMB_LEN] = {0};

USART_RX_TypeDef USART6_Rcr = {USART6, USART6_RX_STREAM, UA6RxMailbox, UA6RxDMAbuf, USART6_RXMB_LEN, USART6_RXDMA_LEN, 0, 0, 0};

void USART6_Configuration(void)
{
	USART_InitTypeDef usart6;
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE); // 使能PB端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);						// 使能UART3时钟

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);

	gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AF;		// 复用模式
	gpio.GPIO_OType = GPIO_OType_PP;	// 推挽输出
	gpio.GPIO_Speed = GPIO_Speed_50MHz; // IO口速度为50MHz
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;	// 上拉
	GPIO_Init(GPIOC, &gpio);			// 根据设定参数初始化GPIOC

	nvic.NVIC_IRQChannel = USART6_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 8; // 抢占优先级
	nvic.NVIC_IRQChannelSubPriority = 0;		// 子优先级
	nvic.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
	NVIC_Init(&nvic);							// 根据指定的参数初始化VIC寄存器

//	/*DMA发送完成中断*/
//	nvic.NVIC_IRQChannel = DMA2_Stream1_IRQn;
//	nvic.NVIC_IRQChannelPreemptionPriority = 9; // 抢占优先级
//	nvic.NVIC_IRQChannelSubPriority = 0;		// 子优先级
//	nvic.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
//	NVIC_Init(&nvic);							// 根据指定的参数初始化VIC寄存器

	usart6.USART_BaudRate = 921600;									   // 波特率
	usart6.USART_WordLength = USART_WordLength_8b;					   // 字长为8位数据格式
	usart6.USART_StopBits = USART_StopBits_1;						   // 一个停止位
	usart6.USART_Parity = USART_Parity_No;							   // 无奇偶校验位
	usart6.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				   // 仅接收
	usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_Init(USART6, &usart6);									   // 初始化串口

	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE); // 开启空闲中断

	USART_Cmd(USART6, ENABLE); // 使能串口

	// Rx
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;						  // 外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR); // 内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UA6RxDMAbuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // DMA传输为单向
	DMA_InitStructure.DMA_BufferSize = USART6_RXDMA_LEN;	// 设置DMA在传输区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	// Tx
	DMA_DeInit(DMA2_Stream6);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR); // 外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Custom_DataBuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // DMA传输为单向
	DMA_InitStructure.DMA_BufferSize = NULL;				// 设置DMA在传输区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);

//	DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);//5.10修改
	DMA_Cmd(DMA2_Stream1, ENABLE);
}
