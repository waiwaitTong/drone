#ifndef __GPIO_H__
#define __GPIO_H__
#include "main.h"
#include "global_declare.h"

#define LED_YELLOW_OFF()  		LED_D1_OFF()
#define LED_YELLOW_ON()		    LED_D1_ON()
#define LED_YELLOW_POLLING()  	LED_D1_POLLING()

#define LED_GREEN_OFF()  		LED_D2_OFF()
#define LED_GREEN_ON()		    LED_D2_ON()
#define LED_GREEN_POLLING()  	LED_D2_POLLING()

#define LED_BLUE_OFF()  		LED_D3_OFF()
#define LED_BLUE_ON()		    LED_D3_ON()
#define LED_BLUE_POLLING()  	LED_D3_POLLING()

#define LED_D1_OFF()  		GPIOC->BSRRL = GPIO_Pin_13
#define LED_D1_ON()		    GPIOC->BSRRH = GPIO_Pin_13
#define LED_D1_POLLING()   	GPIOC->ODR ^= GPIO_Pin_13

#define LED_D2_OFF()  		GPIOC->BSRRL = GPIO_Pin_14
#define LED_D2_ON()		    GPIOC->BSRRH = GPIO_Pin_14
#define LED_D2_POLLING()   	GPIOC->ODR ^= GPIO_Pin_14

#define LED_D3_OFF()  		GPIOC->BSRRL = GPIO_Pin_15
#define LED_D3_ON()		    GPIOC->BSRRH = GPIO_Pin_15
#define LED_D3_POLLING()   	GPIOC->ODR ^= GPIO_Pin_15



void GPIO_Configuration(void);
void Drv8302_Configuration(void);

#endif
