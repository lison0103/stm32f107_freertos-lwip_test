#include "stm32f10x_conf.h"
#include "ili_lcd.h"

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



#define START_BYTE      0x70
#define SET_INDEX       0x00
#define READ_STATUS     0x01
#define LCD_WRITE_REG   0x02
#define LCD_READ_REG    0x03

//******************************************************************************
static unsigned short deviceid = 0 ; //设置一个静态变量用来保存LCD的ID

//******************************************************************************

static void delay(int cnt)
{
    volatile unsigned int dl;
    while(cnt--)
    {
        for(dl=0; dl<500; dl++);
    }
}
static void lcd_port_init(void)
{
	SPI_InitTypeDef    SPI_InitStructure;
  	GPIO_InitTypeDef   GPIO_InitStructure;
	
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
	SPI_I2S_SendData(LCD_SPI, LCD_Reg);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
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

//读一个字节数据
static unsigned short read_data(void)
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
//写一字节数据
static void write_data(unsigned short data_code )
{
	SPI_I2S_SendData(LCD_SPI, data_code >> 8);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	SPI_I2S_SendData(LCD_SPI, data_code & 0xFF);
	while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
}

//写寄存器
static void write_reg(unsigned char reg_addr,unsigned short reg_val)
{
	/* Write 16-bit Index (then Write Reg) */
	LCD_WriteRegIndex(reg_addr);
	/* Write 16-bit Reg */
	/* Reset LCD control line(/CS) and Send Start-Byte */
	LCD_nCS_StartByte(START_BYTE | LCD_WRITE_REG);	
	write_data( reg_val );
	
	LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}
//读寄存器
static unsigned short read_reg(unsigned char reg_addr)
{
	uint16_t tmp = 0;
  
	/* Write 16-bit Index (then Read Reg) */
	LCD_WriteRegIndex(reg_addr);
  	/* Read 16-bit Reg */
  	/* Reset LCD control line(/CS) and Send Start-Byte */
  	LCD_nCS_StartByte(START_BYTE | LCD_READ_REG);
  
	tmp = read_data( );
	
	LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);

	return tmp;
}
#define w_data_prepare()				LCD_WriteRAM_Prepare()
#define r_data_prepare()				LCD_ReadRAM_Prepare()
#define rw_data_end()					LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET)
//******************************************************************************
//返回LCD的ID
unsigned int lcd_getdeviceid(void)
{
	return deviceid;
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

void lcd_SetCursor(unsigned int x,unsigned int y)
{
	write_reg(32,x);    /* 0-239 */
	write_reg(33,y);    /* 0-319 */
}
/* 读取指定地址的GRAM */
unsigned short lcd_read_gram(unsigned int x,unsigned int y)
{
	unsigned short temp;
	lcd_SetCursor(x,y);
	r_data_prepare();
	temp = read_data();
	rw_data_end();
	return temp;
}
void lcd_clear(unsigned short Color)
{
	unsigned int index = 0 ;
	lcd_SetCursor( 0,0 );
	w_data_prepare();	// prepare to write GRAM
	for( index = 0 ; index < ( LCD_WIDTH*LCD_HEIGHT ); index++)
	{
		write_data(Color);
	}
	rw_data_end();
}
void lcd_data_bus_test(void)
{
	unsigned short temp1;
	unsigned short temp2;
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );

    /* wirte */
    lcd_SetCursor(0,0);
    w_data_prepare();
    write_data(0x5555);
    write_data(0xAAAA);
	rw_data_end();

    /* read */
    lcd_SetCursor(0,0);
	
	if( ( deviceid == 0x9325 ) ||
	    ( deviceid == 0x9328 ) ||
		( deviceid == 0x9320 ) )
	{
        temp1 = BGR2RGB( lcd_read_gram(0,0) );
        temp2 = BGR2RGB( lcd_read_gram(1,0) );
	}else{
		temp1 = lcd_read_gram(0,0);
        temp2 = lcd_read_gram(1,0);
	}
	if( (temp1 == 0x5555) && (temp2 == 0xaaaa) )
	{
		//ok!
	}else{
		//error!
	}
}
void lcd_init(void)
{
	lcd_port_init();
	deviceid = read_reg(0x00);
	
	if( (deviceid == 0x9325 ) || (deviceid == 0x9328) )
	{
        write_reg(0x00e7,0x0010);
        write_reg(0x0000,0x0001);  			        //start internal osc
#if defined(_ILI_REVERSE_DIRECTION_)
        write_reg(0x0001,0x0000);                    //Reverse Display
#else
        write_reg(0x0001,0x0100);                    //
#endif
        write_reg(0x0002,0x0700); 				    //power on sequence
        /* [5:4]-ID1~ID0 [3]-AM-1垂直-0水平 */
        write_reg(0x0003,(1<<12)|(1<<5)|(0<<4) | (1<<3) );
        write_reg(0x0004,0x0000);
        write_reg(0x0008,0x0207);
        write_reg(0x0009,0x0000);
        write_reg(0x000a,0x0000); 				//display setting
        write_reg(0x000c,0x0001);				//display setting
        write_reg(0x000d,0x0000); 				//0f3c
        write_reg(0x000f,0x0000);
        //Power On sequence //
        write_reg(0x0010,0x0000);
        write_reg(0x0011,0x0007);
        write_reg(0x0012,0x0000);
        write_reg(0x0013,0x0000);
        delay(15);
        write_reg(0x0010,0x1590);
        write_reg(0x0011,0x0227);
        delay(15);
        write_reg(0x0012,0x009c);
        delay(15);
        write_reg(0x0013,0x1900);
        write_reg(0x0029,0x0023);
        write_reg(0x002b,0x000e);
        delay(15);
        write_reg(0x0020,0x0000);
        write_reg(0x0021,0x0000);
        delay(15);
        write_reg(0x0030,0x0007);
        write_reg(0x0031,0x0707);
        write_reg(0x0032,0x0006);
        write_reg(0x0035,0x0704);
        write_reg(0x0036,0x1f04);
        write_reg(0x0037,0x0004);
        write_reg(0x0038,0x0000);
        write_reg(0x0039,0x0706);
        write_reg(0x003c,0x0701);
        write_reg(0x003d,0x000f);
        delay(15);
        write_reg(0x0050,0x0000);
        write_reg(0x0051,0x00ef);
        write_reg(0x0052,0x0000);
        write_reg(0x0053,0x013f);
#if defined(_ILI_REVERSE_DIRECTION_)
        write_reg(0x0060,0x2700);
#else
        write_reg(0x0060,0xA700);
#endif
        write_reg(0x0061,0x0001);
        write_reg(0x006a,0x0000);
        write_reg(0x0080,0x0000);
        write_reg(0x0081,0x0000);
        write_reg(0x0082,0x0000);
        write_reg(0x0083,0x0000);
        write_reg(0x0084,0x0000);
        write_reg(0x0085,0x0000);
        write_reg(0x0090,0x0010);
        write_reg(0x0092,0x0000);
        write_reg(0x0093,0x0003);
        write_reg(0x0095,0x0110);
        write_reg(0x0097,0x0000);
        write_reg(0x0098,0x0000);
        //display on sequence
        write_reg(0x0007,0x0133);
        write_reg(0x0020,0x0000);
        write_reg(0x0021,0x0000);
    }
    else if( deviceid==0x9320|| deviceid==0x9300)
    {
        write_reg(0x00,0x0000);
#if defined(_ILI_REVERSE_DIRECTION_)
        write_reg(0x0001,0x0100);                    //Reverse Display
#else
        write_reg(0x0001,0x0000);                    // Driver Output Contral.
#endif
        write_reg(0x02,0x0700);	//LCD Driver Waveform Contral.
//		write_reg(0x03,0x1030);	//Entry Mode Set.
        write_reg(0x03,0x1018);	//Entry Mode Set.

        write_reg(0x04,0x0000);	//Scalling Contral.
        write_reg(0x08,0x0202);	//Display Contral 2.(0x0207)
        write_reg(0x09,0x0000);	//Display Contral 3.(0x0000)
        write_reg(0x0a,0x0000);	//Frame Cycle Contal.(0x0000)
        write_reg(0x0c,(1<<0));	//Extern Display Interface Contral 1.(0x0000)
        write_reg(0x0d,0x0000);	//Frame Maker Position.
        write_reg(0x0f,0x0000);	//Extern Display Interface Contral 2.

        delay(15);
        write_reg(0x07,0x0101);	//Display Contral.
        delay(15);

        write_reg(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	//Power Control 1.(0x16b0)
        write_reg(0x11,0x0007);								//Power Control 2.(0x0001)
        write_reg(0x12,(1<<8)|(1<<4)|(0<<0));					//Power Control 3.(0x0138)
        write_reg(0x13,0x0b00);								//Power Control 4.
        write_reg(0x29,0x0000);								//Power Control 7.

        write_reg(0x2b,(1<<14)|(1<<4));

        write_reg(0x50,0);		//Set X Start.
        write_reg(0x51,239);	//Set X End.
        write_reg(0x52,0);		//Set Y Start.
        write_reg(0x53,319);	//Set Y End.

#if defined(_ILI_REVERSE_DIRECTION_)
        write_reg(0x0060,0x2700);  //Driver Output Control.
#else
        write_reg(0x0060,0xA700);
#endif
        write_reg(0x61,0x0001);	//Driver Output Control.
        write_reg(0x6a,0x0000);	//Vertical Srcoll Control.

        write_reg(0x80,0x0000);	//Display Position? Partial Display 1.
        write_reg(0x81,0x0000);	//RAM Address Start? Partial Display 1.
        write_reg(0x82,0x0000);	//RAM Address End-Partial Display 1.
        write_reg(0x83,0x0000);	//Displsy Position? Partial Display 2.
        write_reg(0x84,0x0000);	//RAM Address Start? Partial Display 2.
        write_reg(0x85,0x0000);	//RAM Address End? Partial Display 2.

        write_reg(0x90,(0<<7)|(16<<0));	//Frame Cycle Contral.(0x0013)
        write_reg(0x92,0x0000);	//Panel Interface Contral 2.(0x0000)
        write_reg(0x93,0x0001);	//Panel Interface Contral 3.
        write_reg(0x95,0x0110);	//Frame Cycle Contral.(0x0110)
        write_reg(0x97,(0<<8));	//
        write_reg(0x98,0x0000);	//Frame Cycle Contral.


        write_reg(0x07,0x0173);	//(0x0173)
    }
	//数据总线测试,用于测试硬件连接是否正常.
	lcd_data_bus_test();
	//清屏
	lcd_clear( Blue );
}

//上电
/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void lcd_poweron(void)
{
	/* Power On sequence ---------------------------------------------------------*/
	write_reg(R16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
	write_reg(R17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
	write_reg(R18, 0x0000); /* VREG1OUT voltage */
	write_reg(R19, 0x0000); /* VDV[4:0] for VCOM amplitude */
	delay(20);              /* Dis-charge capacitor power voltage (200ms) */
	write_reg(R16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
	write_reg(R17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
	delay(5);               /* Delay 50 ms */
	write_reg(R18, 0x0139); /* VREG1OUT voltage */
	delay(5);               /* delay 50 ms */
	write_reg(R19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
	write_reg(R41, 0x0013); /* VCM[4:0] for VCOMH */
	delay(5);               /* delay 50 ms */
	write_reg(R7, 0x0173);  /* 262K color and display ON */
}
/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void lcd_displayon(void)
{
	/* Display On */
	write_reg(R7, 0x0173); /* 262K color and display ON */
 
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void lcd_displayoff(void)
{
	/* Display Off */
	write_reg(R7, 0x0);
}
