
#include "stm32f10x_conf.h"
#include "ili_lcd.h"

void bsp_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	/* 2 bit for pre-emption priority, 2 bits for subpriority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


	//PD13~15输出驱动led，配置成开漏输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
//	lcd_init();
}

void LED_Toggle(unsigned char led)
{
	unsigned long pins;
	pins = GPIO_ReadOutputData(GPIOD);
	switch(led)
	{
	case 0:
		LED_Toggle(1);
		LED_Toggle(2);
		LED_Toggle(3);
		break;
	case 1:
		if( pins & GPIO_Pin_13)
			GPIOD->BRR = GPIO_BRR_BR13 ;
		else
			GPIOD->BSRR = GPIO_BSRR_BS13 ;
		break;
	case 2:
		if( pins & GPIO_Pin_14)
			GPIOD->BRR = GPIO_BRR_BR14 ;
		else
			GPIOD->BSRR = GPIO_BSRR_BS14 ;
		break;
	case 3:
		if( pins & GPIO_Pin_15)
			GPIOD->BRR = GPIO_BRR_BR15 ;
		else
			GPIOD->BSRR = GPIO_BSRR_BS15 ;
		break;
	default:
		break;
	}
}