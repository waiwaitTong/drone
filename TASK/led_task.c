#include "led_task.h"

void Led_Flick(void)
{
		GPIOB->ODR^=GPIO_Pin_13;
		GPIOB->ODR^=GPIO_Pin_14;
}
