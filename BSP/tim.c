#include "tim.h"

/*************************************************************************
函 数 名：TIM2_Configuration
函数功能：ETR
备    注：TIM2_ETR
          TIM3_ETR
*************************************************************************/
void TIM2_Configuration(void)  
{
	GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    //使能TIM2时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //使能GPIOA时钟
    
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;              //PA0   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //PA0 输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA,GPIO_Pin_15);                       //PA0 下拉
    
    //初始化定时器2 TIM2 
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;              //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_Prescaler =0;                 //预分频值 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);         
    TIM_ITRxExternalClockConfig(TIM2,TIM_TS_ETRF);          //配置外部触发，否则不会计数
    TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_SetCounter(TIM2, 0);        
    
    TIM_Cmd(TIM2,ENABLE );                                  //使能TIM2
}

void TIM3_Configuration(void)  
{
	GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    //使能TIM2时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);   //使能GPIOA时钟
    
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_TIM3);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;              //PA0   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //PA0 输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOD,GPIO_Pin_2);                       //PA0 下拉
    
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);         
    TIM_ITRxExternalClockConfig(TIM3,TIM_TS_ETRF);          //配置外部触发，否则不会计数
    TIM_ETRClockMode2Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
    TIM_SetCounter(TIM3, 0);        
    
    TIM_Cmd(TIM3,ENABLE );                                  //使能TIM2
}

/****************************************************************************************************
函数名称: TIM4_Configuration()
函数功能: PWM输出
输入参数: 无
返回参数: 无
备   注:  
****************************************************************************************************/
void TIM4_Configuration(void)
{
	GPIO_InitTypeDef         	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         	TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//168MHz

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

    //168M/(168*2)/10000=50Hz PWM频率 = 输入时钟频率 / (预分频器值 * (周期值 + 1))
    TIM_TimeBaseStructure.TIM_Prescaler     = 168-1;//时钟预分频数pcs
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period        = 10000-1;//自动重装载值arr
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;          //????,cnt<ccr???
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = 500;   //???
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;      //?????
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_CtrlPWMOutputs(TIM4, ENABLE);   //TIM1 PWM???????
    TIM_Cmd(TIM4, ENABLE);
}

/****************************************************************************************************
函数名称: TIM5_Configuration()
函数功能: 操作系统定时器时间参考
输入参数: 无
返回参数: 无
备   注:  
****************************************************************************************************/
//TIM5 提供时间参考
#define TIM5_Prescaler  84-1
#define TIM5_Period     4294967296-1        //4294967296us=71min，时间够用了，不做多余处理
//定时器初始化，定时器5是32位通用定时器，APB1=84MHz，1us进一次中断
void TIM5_Configuration(void)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    
	TIM_TimeBaseStructure.TIM_Prescaler         = TIM5_Prescaler;
	TIM_TimeBaseStructure.TIM_Period            = TIM5_Period;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    
    TIM_Cmd(TIM5, ENABLE);
}



/****************************************************************************************************
函数名称: TIM9_Configuration()
函数功能: 陀螺仪数据更新
输入参数: 无
返回参数: 无
备   注:  
****************************************************************************************************/
void TIM9_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 1000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 42-1; 				
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  	           
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;	//共用一个中断函数
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM9, TIM_IT_Update); 
	TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM9, ENABLE);
}

#define TIM10_Prescaler  42-1
#define TIM10_Period     1000-1   
void TIM10_Configuration(void)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
    
	TIM_TimeBaseStructure.TIM_Prescaler         = TIM10_Prescaler;
	TIM_TimeBaseStructure.TIM_Period            = TIM10_Period;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;	//共用一个中断函数
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
    TIM_ClearITPendingBit(TIM10, TIM_IT_Update); 
	TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM10, ENABLE);
}

#define TIM7_Prescaler  42-1
#define TIM7_Period     1000-1   
void TIM7_Configuration(void)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    
	TIM_TimeBaseStructure.TIM_Prescaler         = TIM7_Prescaler;
	TIM_TimeBaseStructure.TIM_Period            = TIM7_Period;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;//不分频
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;	//共用一个中断函数
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update); 
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, ENABLE);
}



