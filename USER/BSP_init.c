#include "BSP_init.h"

void BSP_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	SysTick_Configuration();

	USART1_Configuration(); // 遥控器
	USART2_Configuration(); // 裁判系统
	USART3_Configuration(); // 无线调试
	UART4_Configuration();	// 视觉通信
	USART6_Configuration(); //备用
	delay_ms(50);

	TIM2_Configuration(); // TIM2_ETR
	TIM3_Configuration(); // TIM3_ETR
	TIM4_Configuration(); // PWM 50Hz
	TIM5_Configuration(); // 时间评估 1ms
	TIM7_Configuration(); // 信号模拟
	//	TIM9_Configuration();
	//	TIM10_Configuration();//提供时间参考
	//	delay_ms(50);

	CAN1_Configuration();
	CAN2_Configuration();

	SPI_Configuration();
	bmi088_init();
	
	CAN_7010_Open(CAN1,0x141);
}
