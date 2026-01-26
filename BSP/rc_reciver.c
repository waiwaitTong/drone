#include "rc_reciver.h"

/*************************************************************************
函 数 名：USART1_Configuration(void)
函数功能：遥控器接收机DR16底层配置
备    注：PA10(USART1_RX)
*************************************************************************/
__IO UCHAR8 UA1RxDMAbuf[USART1_RXDMA_LEN] = {0};
     UCHAR8 UA1RxMailbox[USART1_RXMB_LEN] = {0};
USART_RX_TypeDef USART1_Rcr = {USART1,USART1_RX_STREAM,UA1RxMailbox,UA1RxDMAbuf,USART1_RXMB_LEN,USART1_RXDMA_LEN,0,0,0};
void USART1_Configuration(void)
{
    USART_InitTypeDef USART1_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB	| RCC_AHB1Periph_DMA2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7 ,GPIO_AF_USART1);
	
	GPIO_InitStructure.GPIO_Pin      =     GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode     =     GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType    =     GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed    =     GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd     =     GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
      
    USART_DeInit(USART1);
	USART1_InitStructure.USART_BaudRate            =    100000;//SBUS 100K baudrate
	USART1_InitStructure.USART_WordLength          =    USART_WordLength_8b;
	USART1_InitStructure.USART_StopBits            =    USART_StopBits_1;
	USART1_InitStructure.USART_Parity              =    USART_Parity_Even;
	USART1_InitStructure.USART_Mode                =    USART_Mode_Rx;
    USART1_InitStructure.USART_HardwareFlowControl =    USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART1_InitStructure);
    
	USART_Cmd(USART1,ENABLE);//使能串口
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//开启串口DMA接收功能
    
    NVIC_InitStructure.NVIC_IRQChannel                     = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 6;   //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority          = 0;      //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    DMA_DeInit(DMA2_Stream2);
    DMA_InitStructure.DMA_Channel                 =    DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr      =    (uint32_t)&(USART1->DR);//设置DMA传输外设基地址
    DMA_InitStructure.DMA_Memory0BaseAddr         =    (uint32_t)UA1RxDMAbuf;//设置DMA传输内存基地址
    DMA_InitStructure.DMA_DIR                     =    DMA_DIR_PeripheralToMemory;//设置数据传输方向
    DMA_InitStructure.DMA_BufferSize              =    USART1_RXDMA_LEN;//设置DMA一次传输数据量的大小,DR16每隔7ms通过DBus发送一帧数据（18字节）
    DMA_InitStructure.DMA_PeripheralInc           =    DMA_PeripheralInc_Disable;//设置外设地址不变
    DMA_InitStructure.DMA_MemoryInc               =    DMA_MemoryInc_Enable;	//设置内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize      =    DMA_PeripheralDataSize_Byte;//设置外设的数据长度为字节（8bits）
    DMA_InitStructure.DMA_MemoryDataSize          =    DMA_MemoryDataSize_Byte;//设置内存的数据长度为字节（8bits）
    DMA_InitStructure.DMA_Mode                    =    DMA_Mode_Circular;//设置DMA模式为循环模式
    DMA_InitStructure.DMA_Priority                =    DMA_Priority_VeryHigh;//设置DMA通道的优先级为最高优先级
    DMA_InitStructure.DMA_FIFOMode                =    DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold           =    DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst             =    DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst         =    DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2,&DMA_InitStructure);

    DMA_Cmd(DMA2_Stream2,ENABLE);//使能DMA
}
