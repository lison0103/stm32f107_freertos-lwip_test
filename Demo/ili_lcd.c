#include "stm32f10x_conf.h"
#include "ili_lcd.h"

/** 
  * @brief  LCD Registers  
  */ 
#define LCD_REG_0             0x00
#define LCD_REG_1             0x01
#define LCD_REG_2             0x02
#define LCD_REG_3             0x03
#define LCD_REG_4             0x04
#define LCD_REG_5             0x05
#define LCD_REG_6             0x06
#define LCD_REG_7             0x07
#define LCD_REG_8             0x08
#define LCD_REG_9             0x09
#define LCD_REG_10            0x0A
#define LCD_REG_12            0x0C
#define LCD_REG_13            0x0D
#define LCD_REG_14            0x0E
#define LCD_REG_15            0x0F
#define LCD_REG_16            0x10
#define LCD_REG_17            0x11
#define LCD_REG_18            0x12
#define LCD_REG_19            0x13
#define LCD_REG_20            0x14
#define LCD_REG_21            0x15
#define LCD_REG_22            0x16
#define LCD_REG_23            0x17
#define LCD_REG_24            0x18
#define LCD_REG_25            0x19
#define LCD_REG_26            0x1A
#define LCD_REG_27            0x1B
#define LCD_REG_28            0x1C
#define LCD_REG_29            0x1D
#define LCD_REG_30            0x1E
#define LCD_REG_31            0x1F
#define LCD_REG_32            0x20
#define LCD_REG_33            0x21
#define LCD_REG_34            0x22
#define LCD_REG_36            0x24
#define LCD_REG_37            0x25
#define LCD_REG_40            0x28
#define LCD_REG_41            0x29
#define LCD_REG_43            0x2B
#define LCD_REG_45            0x2D
#define LCD_REG_48            0x30
#define LCD_REG_49            0x31
#define LCD_REG_50            0x32
#define LCD_REG_51            0x33
#define LCD_REG_52            0x34
#define LCD_REG_53            0x35
#define LCD_REG_54            0x36
#define LCD_REG_55            0x37
#define LCD_REG_56            0x38
#define LCD_REG_57            0x39
#define LCD_REG_58            0x3A
#define LCD_REG_59            0x3B
#define LCD_REG_60            0x3C
#define LCD_REG_61            0x3D
#define LCD_REG_62            0x3E
#define LCD_REG_63            0x3F
#define LCD_REG_64            0x40
#define LCD_REG_65            0x41
#define LCD_REG_66            0x42
#define LCD_REG_67            0x43
#define LCD_REG_68            0x44
#define LCD_REG_69            0x45
#define LCD_REG_70            0x46
#define LCD_REG_71            0x47
#define LCD_REG_72            0x48
#define LCD_REG_73            0x49
#define LCD_REG_74            0x4A
#define LCD_REG_75            0x4B
#define LCD_REG_76            0x4C
#define LCD_REG_77            0x4D
#define LCD_REG_78            0x4E
#define LCD_REG_79            0x4F
#define LCD_REG_80            0x50
#define LCD_REG_81            0x51
#define LCD_REG_82            0x52
#define LCD_REG_83            0x53
#define LCD_REG_96            0x60
#define LCD_REG_97            0x61
#define LCD_REG_106           0x6A
#define LCD_REG_118           0x76
#define LCD_REG_128           0x80
#define LCD_REG_129           0x81
#define LCD_REG_130           0x82
#define LCD_REG_131           0x83
#define LCD_REG_132           0x84
#define LCD_REG_133           0x85
#define LCD_REG_134           0x86
#define LCD_REG_135           0x87
#define LCD_REG_136           0x88
#define LCD_REG_137           0x89
#define LCD_REG_139           0x8B
#define LCD_REG_140           0x8C
#define LCD_REG_141           0x8D
#define LCD_REG_143           0x8F
#define LCD_REG_144           0x90
#define LCD_REG_145           0x91
#define LCD_REG_146           0x92
#define LCD_REG_147           0x93
#define LCD_REG_148           0x94
#define LCD_REG_149           0x95
#define LCD_REG_150           0x96
#define LCD_REG_151           0x97
#define LCD_REG_152           0x98
#define LCD_REG_153           0x99
#define LCD_REG_154           0x9A
#define LCD_REG_157           0x9D
#define LCD_REG_192           0xC0
#define LCD_REG_193           0xC1
#define LCD_REG_229           0xE5

//******************************************************************************
/** 
  * @brief  LCD Control pins  
  */ 
//液晶片选
#define LCD_NCS_PIN            GPIO_Pin_2                  
#define LCD_NCS_GPIO_PORT      GPIOB                       
#define LCD_NCS_GPIO_CLK       RCC_APB2Periph_GPIOB  
//触摸屏片选
#define LCD_TCS_PIN		       GPIO_Pin_4
#define LCD_TCS_GPIO_PORT      GPIOC
#define LCD_TCS_GPIO_CLK       RCC_APB2Periph_GPIOC
//触摸屏中断
#define LCD_TIRQ_PIN			GPIO_Pin_7
#define LCD_TIRQ_GPIO_PORT		GPIOE
#define LCD_TIRQ_GPIO_CLK		RCC_APB2Periph_GPIOE
/** 
  * @brief  LCD SPI Interface pins 
  */ 
//SPI3
#define LCD_SPI_SCK_PIN        GPIO_Pin_10                 
#define LCD_SPI_MISO_PIN       GPIO_Pin_11                 
#define LCD_SPI_MOSI_PIN       GPIO_Pin_12                 
#define LCD_SPI_GPIO_PORT      GPIOC                       
#define LCD_SPI_GPIO_CLK       RCC_APB2Periph_GPIOC  
#define LCD_SPI			       SPI3
#define LCD_SPI_CLK		       RCC_APB1Periph_SPI3

/*液晶屏和触摸屏的SPI时序不一样，
	处理方式是默认是液晶屏的SPI时序，需要操作触摸屏是更改SPI时序，
	操作完毕后,恢复到液晶屏SPI的时序。
*/

#define START_BYTE      0x70
#define SET_INDEX       0x00
#define READ_STATUS     0x01
#define LCD_WRITE_REG   0x02
#define LCD_READ_REG    0x03


#define ABS(X)  ((X) > 0 ? (X) : -(X))  

//******************************************************************************
static unsigned short deviceid = 0 ; //设置一个静态变量用来保存LCD的ID
//字体
static sFONT *LCD_Currentfonts;
//字节颜色和背景颜色
static __IO uint16_t TextColor = 0x0000, BackColor = 0xFFFF;
//******************************************************************************
static void _delay_(int cnt)
{
    volatile unsigned int dl;
    while(cnt--)
    {
        for(dl=0; dl<500; dl++);
    }
}
static void lcd_port_init(void)
{
	SPI_InitTypeDef		SPI_InitStructure;
  	GPIO_InitTypeDef	GPIO_InitStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	
	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(LCD_NCS_GPIO_CLK, ENABLE);
	
	/* Configure NCS in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = LCD_NCS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LCD_NCS_GPIO_PORT, &GPIO_InitStructure);
	
	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(LCD_TCS_GPIO_CLK, ENABLE);
	
	/* Configure NCS in Output Push-Pull mode */
	GPIO_InitStructure.GPIO_Pin = LCD_TCS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(LCD_TCS_GPIO_PORT, &GPIO_InitStructure);
	
	LCD_TCS_GPIO_PORT->BSRR = LCD_TCS_PIN ;
	LCD_NCS_GPIO_PORT->BSRR = LCD_NCS_PIN ;
	
	//配置中断脚
	RCC_APB2PeriphClockCmd(LCD_TIRQ_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = LCD_TIRQ_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(LCD_TIRQ_GPIO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(LCD_TIRQ_GPIO_PORT,LCD_TIRQ_PIN);

	//配置int脚
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource7);

	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(LCD_SPI_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);
	
	/* Enable SPI clock  */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
	/* Configure SPI pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = LCD_SPI_SCK_PIN | LCD_SPI_MISO_PIN | LCD_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(LCD_SPI_GPIO_PORT, &GPIO_InitStructure);
	
	SPI_I2S_DeInit(LCD_SPI);
	
	/* SPI Config */
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(LCD_SPI, &SPI_InitStructure);
	
	/* SPI enable */
	SPI_Cmd(LCD_SPI, ENABLE);
	
	 //配置中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
/**
  * @brief  Sets or reset LCD control lines.
  * @param  GPIOx: where x can be B or D to select the GPIO peripheral.
  * @param  CtrlPins: the Control line. This parameter can be:
  *     @arg LCD_NCS_PIN: Chip Select pin
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
static void LCD_CtrlLinesWrite(GPIO_TypeDef* GPIOx, uint16_t CtrlPins, BitAction BitVal)
{
	/* Set or Reset the control line */
	GPIO_WriteBit(GPIOx, CtrlPins, BitVal);
}
/**
  * @brief  Writes to the LCD RAM.
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAM(uint16_t data_code)
{
	SPI_I2S_SendData(LCD_SPI, data_code >> 8);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	SPI_I2S_ReceiveData(LCD_SPI);
	
	SPI_I2S_SendData(LCD_SPI, data_code & 0xFF);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	SPI_I2S_ReceiveData(LCD_SPI);
}
/**
  * @brief  Reads the LCD RAM.
  * @param  None
  * @retval LCD RAM Value.
  */
//读一个字数据
uint16_t LCD_ReadRAM(void)
{
	uint16_t tmp = 0;
	uint8_t i = 0;
  
	/* LCD_SPI prescaler: 4 */
	LCD_SPI->CR1 &= 0xFFC7;
	LCD_SPI->CR1 |= 0x0008;
  
	for(i = 0; i < 5; i++)
	{
		SPI_I2S_SendData(LCD_SPI, 0xFF);
		while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
		{
		}
		/* One byte of invalid dummy data read after the start byte */
		while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
		{    
		}
		SPI_I2S_ReceiveData(LCD_SPI); 
	}
	SPI_I2S_SendData(LCD_SPI, 0xFF);
	/* Read upper byte */
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	/* Read lower byte */
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}
	tmp = SPI_I2S_ReceiveData(LCD_SPI);
  
  
	SPI_I2S_SendData(LCD_SPI, 0xFF);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	/* Read lower byte */
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}
	tmp = ((tmp & 0xFF) << 8) | SPI_I2S_ReceiveData(LCD_SPI);
	
	/* LCD_SPI prescaler: 2 */
	LCD_SPI->CR1 &= 0xFFC7;
	return tmp;
}

/**
  * @brief  Reset LCD control line(/CS) and Send Start-Byte
  * @param  Start_Byte: the Start-Byte to be sent
  * @retval None
  */
static void LCD_nCS_StartByte(uint8_t Start_Byte)
{
	LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_RESET);
	SPI_I2S_SendData(LCD_SPI, Start_Byte);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	SPI_I2S_ReceiveData(LCD_SPI);
}
/**
  * @brief  Writes index to select the LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
static void LCD_WriteRegIndex(uint8_t LCD_Reg)
{
	/* Reset LCD control line(/CS) and Send Start-Byte */
	LCD_nCS_StartByte(START_BYTE | SET_INDEX);
	
	/* Write 16-bit Reg Index (High Byte is 0) */
	SPI_I2S_SendData(LCD_SPI, 0x00);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	SPI_I2S_ReceiveData(LCD_SPI);
	
	SPI_I2S_SendData(LCD_SPI, LCD_Reg);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	SPI_I2S_ReceiveData(LCD_SPI);
	LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}
/**
  * @brief  Prepare to write to the LCD RAM.
  * @param  None
  * @retval None
  */
static void LCD_WriteRAM_Prepare(void)
{
	LCD_WriteRegIndex(34); /* Select GRAM Reg */
	/* Reset LCD control line(/CS) and Send Start-Byte */
	LCD_nCS_StartByte(START_BYTE | LCD_WRITE_REG);
}
static void LCD_ReadRAM_Prepare(void)
{
	LCD_WriteRegIndex(34); /* Select GRAM Reg */
	/* Reset LCD control line(/CS) and Send Start-Byte */
	LCD_nCS_StartByte(START_BYTE | LCD_READ_REG);
}
/**
  * @brief  Writes to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
//写寄存器
static void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
	/* Write 16-bit Index (then Write Reg) */
	LCD_WriteRegIndex(LCD_Reg);
	
	/* Write 16-bit Reg */
	
	/* Reset LCD control line(/CS) and Send Start-Byte */
	LCD_nCS_StartByte(START_BYTE | LCD_WRITE_REG);	
	LCD_WriteRAM( LCD_RegValue );
	LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  LCD_Reg: address of the selected register.
  * @retval LCD Register Value.
  */
//读寄存器
static uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
	uint16_t tmp = 0;
  
	/* Write 16-bit Index (then Read Reg) */
	LCD_WriteRegIndex(LCD_Reg);
  	/* Read 16-bit Reg */
  	/* Reset LCD control line(/CS) and Send Start-Byte */
  	LCD_nCS_StartByte(START_BYTE | LCD_READ_REG);
  
	tmp = LCD_ReadRAM( );
	
	LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);

	return tmp;
}

#define w_data_prepare()				LCD_WriteRAM_Prepare()
#define r_data_prepare()				LCD_ReadRAM_Prepare()
#define rw_data_end()					LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET)

/* 读取指定地址的GRAM */
unsigned short LCD_ReadGram(unsigned int x,unsigned int y)
{
	unsigned short temp;
	LCD_SetCursor(x,y);
	LCD_ReadRAM_Prepare();
	temp = LCD_ReadRAM();
	LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
	return temp;
}
void LCD_WriteGram(unsigned int x,unsigned int y,unsigned short c)
{
	/* wirte */
    LCD_SetCursor(x,y);
    LCD_WriteRAM_Prepare();
    LCD_WriteRAM(c);
	rw_data_end();
	
}
/**
  * @brief  Displays a pixel.
  * @param  x: pixel x.
  * @param  y: pixel y.  
  * @retval None
  */
static void PutPixel(int16_t x, int16_t y)
{ 
  if(x < 0 || x > 239 || y < 0 || y > 319)
  {
    return;  
  }
  //LCD_DrawLine(x, y, 1, LCD_DIR_HORIZONTAL);
  LCD_WriteGram(x,y,TextColor);
}
unsigned short BGR2RGB(unsigned short c)
{
	unsigned short r,g,b,rgb;
	
	b = (c>>0)  & 0x1f ;
	g = (c>>5)  & 0x3f ;
	r = (c>>11) & 0x1f ;
	
	rgb = ( b<<11 ) + ( g<<5 ) + ( r<<0 );
	
	return (rgb);
}

void lcd_data_bus_test(void)
{
	unsigned short temp1;
	unsigned short temp2;
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    LCD_WriteReg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );

	LCD_WriteGram(0,0,0x5555);
	LCD_WriteGram(0,1,0xaaaa);

	temp1 = BGR2RGB( LCD_ReadGram(0,0) );
	temp2 = BGR2RGB( LCD_ReadGram(1,0) );

	if( (temp1 == 0x5555) && (temp2 == 0xaaaa) )
	{
		//ok!
	}else{
		while(1);//error!
	}
}
void lcd_init(void)
{
	lcd_port_init();
	
	deviceid = LCD_ReadReg(0x00);
	
	if( deviceid != 0x9325)
	{
		while (1);
	}

	/* Start Initial Sequence ------------------------------------------------*/
    LCD_WriteReg(LCD_REG_0, 0x0001); /* Start internal OSC. */
    LCD_WriteReg(LCD_REG_1, 0x0000); /* Set SS and SM bit */
    LCD_WriteReg(LCD_REG_2, 0x0700); /* Set 1 line inversion */
    LCD_WriteReg(LCD_REG_3, 0x1018);//(1<<12)|(0<<5)|(1<<4) | (1<<3));// /* Set GRAM write direction and BGR=1. */
    LCD_WriteReg(LCD_REG_4, 0x0000); /* Resize register */
    LCD_WriteReg(LCD_REG_8, 0x0202); /* Set the back porch and front porch */
    LCD_WriteReg(LCD_REG_9, 0x0000); /* Set non-display area refresh cycle ISC[3:0] */
    LCD_WriteReg(LCD_REG_10, 0x0000); /* FMARK function */
    LCD_WriteReg(LCD_REG_12, 0x0000); /* RGB interface setting */
    LCD_WriteReg(LCD_REG_13, 0x0000); /* Frame marker Position */
    LCD_WriteReg(LCD_REG_15, 0x0000); /* RGB interface polarity */

    /* Power On sequence -----------------------------------------------------*/
    LCD_WriteReg(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(LCD_REG_17, 0x0007); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
    LCD_WriteReg(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude */
    _delay_(20);                      /* Dis-charge capacitor power voltage (200ms) */
    LCD_WriteReg(LCD_REG_16, 0x1590); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(LCD_REG_17, 0x0227); /* DC1[2:0], DC0[2:0], VC[2:0] */
    _delay_(5);                       /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_18, 0x009c); /* VREG1OUT voltage */
    _delay_(5);                       /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_19, 0x1900); /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(LCD_REG_41, 0x0023); /* VCM[4:0] for VCOMH */
	LCD_WriteReg(LCD_REG_43, 0x000e);
    _delay_(5);                       /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_32, 0x0000); /* GRAM horizontal Address */
    LCD_WriteReg(LCD_REG_33, 0x0000); /* GRAM Vertical Address */

    /* Adjust the Gamma Curve (ILI9325)---------------------------------------*/
    LCD_WriteReg(LCD_REG_48, 0x0007);
    LCD_WriteReg(LCD_REG_49, 0x0707);
    LCD_WriteReg(LCD_REG_50, 0x0006);
    LCD_WriteReg(LCD_REG_53, 0x0704);
    LCD_WriteReg(LCD_REG_54, 0x1f04);
    LCD_WriteReg(LCD_REG_55, 0x0004);
    LCD_WriteReg(LCD_REG_56, 0x0000);
    LCD_WriteReg(LCD_REG_57, 0x0706);
    LCD_WriteReg(LCD_REG_60, 0x0701);
    LCD_WriteReg(LCD_REG_61, 0x000f);

    /* Set GRAM area ---------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_80, 0x0000); /* Horizontal GRAM Start Address */
    LCD_WriteReg(LCD_REG_81, 0x00EF); /* Horizontal GRAM End Address */
    LCD_WriteReg(LCD_REG_82, 0x0000); /* Vertical GRAM Start Address */
    LCD_WriteReg(LCD_REG_83, 0x013F); /* Vertical GRAM End Address */

    LCD_WriteReg(LCD_REG_96,  0x2700); /* Gate Scan Line(GS=1, scan direction is G320~G1) */
    LCD_WriteReg(LCD_REG_97,  0x0001); /* NDL,VLE, REV */
    LCD_WriteReg(LCD_REG_106, 0x0000); /* set scrolling line */

    /* Partial Display Control -----------------------------------------------*/
    LCD_WriteReg(LCD_REG_128, 0x0000);
    LCD_WriteReg(LCD_REG_129, 0x0000);
    LCD_WriteReg(LCD_REG_130, 0x0000);
    LCD_WriteReg(LCD_REG_131, 0x0000);
    LCD_WriteReg(LCD_REG_132, 0x0000);
    LCD_WriteReg(LCD_REG_133, 0x0000);

    /* Panel Control ---------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_144, 0x0010);
    LCD_WriteReg(LCD_REG_146, 0x0000);
    LCD_WriteReg(LCD_REG_147, 0x0003);
    LCD_WriteReg(LCD_REG_149, 0x0110);
    LCD_WriteReg(LCD_REG_151, 0x0000);
    LCD_WriteReg(LCD_REG_152, 0x0000);
	
	//display on sequence
    LCD_WriteReg(LCD_REG_7,   0x0133); /* 262K color and display ON */ 
	LCD_WriteReg(LCD_REG_32,  0x0000);
	LCD_WriteReg(LCD_REG_33,  0x0000);
	
	//清屏
	LCD_Clear(Blue);
	LCD_Clear(White);
	LCD_SetFont(&LCD_DEFAULT_FONT);
	LCD_SetColors(Red,Blue);
	
	LCD_DrawLine(0,0,320,LCD_DIR_HORIZONTAL);
	LCD_DrawLine(0,239,320,LCD_DIR_HORIZONTAL);
	
	LCD_DrawLine(0,0,240,LCD_DIR_VERTICAL);
	LCD_DrawLine(319,0,240,LCD_DIR_VERTICAL);
			
//	LCD_DrawHLine(Red,0,319,0);
//	LCD_DrawHLine(Red,0,319,239);
//	LCD_DrawVLine(Red,0, 0, LCD_PIXEL_HEIGHT);
//	LCD_DrawVLine(Red,319, 0, LCD_PIXEL_HEIGHT);		
//	LCD_DrawHLine(Red,0,LCD_PIXEL_WIDTH,LCD_PIXEL_HEIGHT-1);
//		
//	LCD_DisplayChar(0,0,'F');
//	LCD_DisplayChar(304,0,'F');
//	LCD_DisplayChar(0,216,'F');
//	LCD_DisplayChar(304,216,'F');
//	
//	LCD_DrawVLine(Red,0,0,LCD_PIXEL_HEIGHT -1);
//
//	LCD_DrawVLine(Red,LCD_PIXEL_WIDTH - 1,0,LCD_PIXEL_HEIGHT -1);
//	
	LCD_DisplayStringLine(48,"012345678901234");
	
	LCD_DrawRect(160,160,24,32);
	
	LCD_DrawCircle(100,100,64);
	
	LCD_DrawFullRect(200,80,64,32);
	
	LCD_DrawFullCircle(100,100,32);
	
	LCD_DrawUniLine(120,80,160,200);
	
	//读一次触摸屏
	LCD_ReadTouchXY(0);
	LCD_ReadTouchXY(1);
		
}
/**
  * @brief  Sets the cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position. 
  * @retval None
  */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  
	LCD_WriteReg(LCD_REG_32, Ypos);
	LCD_WriteReg(LCD_REG_33, 319 - Xpos);
}

/**
  * @brief  Clears the hole LCD.
  * @param  Color: the color of the background.
  * @retval None
  */
void LCD_Clear(uint16_t Color)
{
	uint32_t index = 0;

	LCD_SetCursor(0,0); 
	LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
	for(index = 0; index < ( LCD_PIXEL_WIDTH*LCD_PIXEL_HEIGHT ); index++)
	{
		LCD_WriteRAM(Color);
	}  
	rw_data_end();
}
/**
  * @brief  Sets the LCD Text and Background colors.
  * @param  _TextColor: specifies the Text Color.
  * @param  _BackColor: specifies the Background Color.
  * @retval None
  */
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor)
{
  TextColor = _TextColor; 
  BackColor = _BackColor;
}

/**
  * @brief  Gets the LCD Text and Background colors.
  * @param  _TextColor: pointer to the variable that will contain the Text 
            Color.
  * @param  _BackColor: pointer to the variable that will contain the Background 
            Color.
  * @retval None
  */
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor)
{
  *_TextColor = TextColor; *_BackColor = BackColor;
}

/**
  * @brief  Sets the Text color.
  * @param  Color: specifies the Text color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetTextColor(__IO uint16_t Color)
{
  TextColor = Color;
}


/**
  * @brief  Sets the Background color.
  * @param  Color: specifies the Background color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetBackColor(__IO uint16_t Color)
{
  BackColor = Color;
}

/**
  * @brief  Sets the Text Font.
  * @param  fonts: specifies the font to be used.
  * @retval None
  */
void LCD_SetFont(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}

/**
  * @brief  Gets the Text Font.
  * @param  None.
  * @retval the used font.
  */
sFONT *LCD_GetFont(void)
{
  return LCD_Currentfonts;
}
/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: the Line where to display the character shape.
  * @param  Ypos: start column address.
  * @param  c: pointer to the character data.
  * @retval None
  */
void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c)
{
  uint32_t index = 0, i = 0;
  
  LCD_SetCursor(Xpos, Ypos);
  
  for(index = 0; index < LCD_Currentfonts->Height; index++)
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(i = 0; i < LCD_Currentfonts->Width; i++)
    {
  
      if((((c[index] & ((0x80 << ((LCD_Currentfonts->Width / 12 ) * 8 ) ) >> i)) == 0x00) &&(LCD_Currentfonts->Width <= 12))||
        (((c[index] & (0x1 << i)) == 0x00)&&(LCD_Currentfonts->Width > 12 )))

      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      } 
    }
	rw_data_end();
    Ypos++;
    LCD_SetCursor(Xpos, Ypos);
  }
}
/**
  * @brief  Displays one character (16dots width, 24dots height).
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  Column: start column address.
  * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
  * @retval None
  */
void LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii)
{
	Ascii -= 32;
	LCD_DrawChar(Line, Column, &LCD_Currentfonts->table[Ascii * LCD_Currentfonts->Height]);
}
/**
  * @brief  Displays a maximum of 20 char on the LCD.
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  *ptr: pointer to string to display on LCD.
  * @retval None
  */
void LCD_DisplayStringLine(uint16_t Line, uint8_t *ptr)
{
	uint16_t refcolumn = 0 ;
	/* Send the string character by character on lCD */
	while ( *ptr != 0 )
	{
		/* Display one character on LCD */
		LCD_DisplayChar(refcolumn, Line, *ptr);
		/* Decrement the column position by 16 */
		refcolumn += LCD_Currentfonts->Width;
		/* Point on the next character */
		ptr++;
	}
}
//绘制水平线
void LCD_DrawHLine(unsigned short c, unsigned int x1, unsigned int x2, unsigned int y)
{
	LCD_SetCursor(x1, y);
    w_data_prepare(); /* Prepare to write GRAM */
    while (x1 < x2)
    {
        LCD_WriteRAM(c);
        x1++;
    }
	rw_data_end();
}
//绘制垂直线
void LCD_DrawVLine(unsigned short c, unsigned int x, unsigned int y1, unsigned int y2)
{
	/* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
	LCD_WriteReg(0x0003,(1<<12)|(0<<5)|(1<<4) | (0<<3) );

    LCD_SetCursor(x, y1);
    w_data_prepare(); /* Prepare to write GRAM */
    while (y1 < y2)
    {
        LCD_WriteRAM(c);
        y1++;
    }
	rw_data_end();
	/* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    LCD_WriteReg(0x0003,(1<<12)|(0<<5)|(1<<4) | (1<<3) );
}
/**
  * @brief  Displays a line.
  * @param Xpos: specifies the X position.
  * @param Ypos: specifies the Y position.
  * @param Length: line length.
  * @param Direction: line direction.
  *   This parameter can be one of the following values: Vertical or Horizontal.
  * @retval None
  */
void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction)
{
	uint32_t i = 0;

	LCD_SetCursor(Xpos, Ypos);
	if(Direction == LCD_DIR_HORIZONTAL)
	{
		LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
		for(i = 0; i < Length; i++)
		{
			LCD_WriteRAM(TextColor);
		}
		rw_data_end();
	}
	else
	{
		/* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
		LCD_WriteReg(0x0003,(1<<12)|(0<<5)|(1<<4) | (0<<3) );
		w_data_prepare(); /* Prepare to write GRAM */

		for(i = 0; i < Length; i++)
		{
			LCD_WriteRAM(TextColor);	
		}
		rw_data_end();
		/* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
		LCD_WriteReg(0x0003,(1<<12)|(0<<5)|(1<<4) | (1<<3) );
	}
}
/**
  * @brief  Displays a rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: display rectangle height.
  * @param  Width: display rectangle width.
  * @retval None
  */
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  LCD_DrawLine(Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);
  LCD_DrawLine(Xpos, Ypos+ Height, Width, LCD_DIR_HORIZONTAL);
  
  LCD_DrawLine(Xpos, Ypos, Height, LCD_DIR_VERTICAL);
  LCD_DrawLine(Xpos+ Width,Ypos, Height,LCD_DIR_VERTICAL);
}

/**
  * @brief  Displays a circle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Radius
  * @retval None
  */
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;/* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
    LCD_SetCursor(Xpos + CurX, Ypos + CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
	rw_data_end();
    LCD_SetCursor(Xpos + CurX, Ypos - CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
	rw_data_end();
    LCD_SetCursor(Xpos - CurX, Ypos + CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
	rw_data_end();
    LCD_SetCursor(Xpos - CurX, Ypos - CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
	rw_data_end();
    LCD_SetCursor(Xpos + CurY, Ypos + CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
	rw_data_end();
    LCD_SetCursor(Xpos + CurY, Ypos - CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
	rw_data_end();
    LCD_SetCursor(Xpos - CurY, Ypos + CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
	rw_data_end();
    LCD_SetCursor(Xpos - CurY, Ypos - CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
	rw_data_end();
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

/**
  * @brief  Displays a full rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: rectangle height.
  * @param  Width: rectangle width.
  * @retval None
  */
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  LCD_SetTextColor(TextColor);

  LCD_DrawLine(Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);
  LCD_DrawLine(Xpos, (Ypos + Height), Width, LCD_DIR_HORIZONTAL);
  
  LCD_DrawLine(Xpos, Ypos, Height, LCD_DIR_VERTICAL);
  LCD_DrawLine((Xpos + Width),Ypos, Height, LCD_DIR_VERTICAL);

  Width --;
  Height-=1;
  Xpos++;

  uint16_t bakTextColor = TextColor;
  LCD_SetTextColor(BackColor);

  while(Height--)
  {
    LCD_DrawLine(Xpos, ++Ypos, Width, LCD_DIR_HORIZONTAL);    
  }

  LCD_SetTextColor(bakTextColor);
}

/**
  * @brief  Displays a full circle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Radius
  * @retval None
  */
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;    /* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;
  
  uint16_t bakTextColor = TextColor;
  LCD_SetTextColor(BackColor);

  while (CurX <= CurY)
  {
    if(CurY > 0) 
    {
      LCD_DrawLine(Xpos - CurY, Ypos - CurX, 2*CurY, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(Xpos - CurY, Ypos + CurX, 2*CurY, LCD_DIR_HORIZONTAL);
    }

    if(CurX > 0) 
    {
      LCD_DrawLine(Xpos - CurX, Ypos - CurY, 2*CurX, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(Xpos - CurX, Ypos + CurY, 2*CurX, LCD_DIR_HORIZONTAL);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  LCD_SetTextColor(bakTextColor);
  LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Displays an uni-line (between two points).
  * @param  x1: specifies the point 1 x position.
  * @param  y1: specifies the point 1 y position.
  * @param  x2: specifies the point 2 x position.
  * @param  y2: specifies the point 2 y position.
  * @retval None
  */
void LCD_DrawUniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
  curpixel = 0;
  
  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */
  
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }
  
  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    PutPixel(x, y);             /* Draw the current pixel */
    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }
}

uint16_t LCD_ReadTouchXY(unsigned char mode)
{
	unsigned short tmp;
	unsigned char op;
	SPI_InitTypeDef    SPI_InitStructure;
	
	//需要操作触摸屏是更改SPI时序，
	LCD_CtrlLinesWrite(LCD_TCS_GPIO_PORT, LCD_TCS_PIN, Bit_SET);
	SPI_I2S_DeInit(LCD_SPI);	
	/* SPI Config */
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(LCD_SPI, &SPI_InitStructure);
	
	/* SPI enable */
	SPI_Cmd(LCD_SPI, ENABLE);
	
//	/* LCD_SPI prescaler: 256 */
//	LCD_SPI->CR1 &= 0xFFC7;
//	LCD_SPI->CR1 |= (0x07 << 3 );
	
	if(mode == 0 )
		op = 0x90 ;//| 0x03;
	else
		op = 0xd0 ;//| 0x03;
	
	LCD_CtrlLinesWrite(LCD_TCS_GPIO_PORT, LCD_TCS_PIN, Bit_RESET);
	
	_delay_(10);
	
	SPI_I2S_SendData(LCD_SPI, op);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	SPI_I2S_ReceiveData(LCD_SPI);
	
	_delay_(1000);
	
	SPI_I2S_SendData(LCD_SPI, 0xFF);
	/* Read upper byte */
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	/* Read lower byte */
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}
	tmp = SPI_I2S_ReceiveData(LCD_SPI);
  
  
	SPI_I2S_SendData(LCD_SPI, 0xFF);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	/* Read lower byte */
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}
	tmp = ((tmp & 0xFF) << 8) | SPI_I2S_ReceiveData(LCD_SPI);
	
	tmp >>= 3;
	tmp &= 0xfff;
	
	LCD_CtrlLinesWrite(LCD_TCS_GPIO_PORT, LCD_TCS_PIN, Bit_SET);

	//操作完毕后,恢复到液晶屏SPI的时序。
	SPI_I2S_DeInit(LCD_SPI);
	
	/* SPI Config */
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(LCD_SPI, &SPI_InitStructure);
	
	/* SPI enable */
	SPI_Cmd(LCD_SPI, ENABLE);
	
//	/* LCD_SPI prescaler: 2 */
//	LCD_SPI->CR1 &= 0xFFC7;
	return tmp;
}
//开中断
void LCD_TouchEnable(void)
{
	EXTI_ClearITPendingBit(EXTI_Line7);
	EXTI->IMR |= EXTI_Line7;
}
//关闭中断
void LCD_TouchDisable(void)
{
	EXTI->IMR &= ~EXTI_Line7;
}
