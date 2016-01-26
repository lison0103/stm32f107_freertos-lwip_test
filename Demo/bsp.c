
#include "stm32f10x_conf.h"
#include "ili_lcd.h"

void bsp_init(void)
{
     GPIO_InitTypeDef  GPIO_InitStructure;
            
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);	 //使能PB,PD端口时钟
            
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //LED1-->PB.15 端口配置
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
     GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.15
     GPIO_SetBits(GPIOB,GPIO_Pin_15);						 //PB.15 输出高

     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	    		 //LED0-->PD.11 端口配置, 推挽输出
     GPIO_Init(GPIOD, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
     GPIO_SetBits(GPIOD,GPIO_Pin_11); 						 //PD.11 输出高 
	
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