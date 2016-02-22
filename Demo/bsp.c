
#include "stm32f10x_conf.h"


void WFI_SET(void)
{
	__ASM("WFI");		  
}
//�ر������ж�
void INTX_DISABLE(void)
{
	__ASM("CPSID I");    		  
}
//���������ж�
void INTX_ENABLE(void)
{
	__ASM("CPSIE I");		  
}
//����ջ����ַ
//addr:ջ����ַ
void MSR_MSP(u32 addr) 
{
    __ASM("MSR MSP, r0"); 			//set Main Stack value
    __ASM("BX r14");
}

void bsp_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
        
        NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x10000);
        
        INTX_ENABLE();

	/* 2 bit for pre-emption priority, 2 bits for subpriority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);	 //ʹ��PB,PD�˿�ʱ��
                
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //LED1-->PB.15 �˿�����
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
         GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.15
         GPIO_SetBits(GPIOB,GPIO_Pin_15);						 //PB.15 �����

         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	    		 //LED0-->PD.11 �˿�����, �������
         GPIO_Init(GPIOD, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
         GPIO_SetBits(GPIOD,GPIO_Pin_11); 
	
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