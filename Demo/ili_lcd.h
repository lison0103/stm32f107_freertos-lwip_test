#ifndef __ILI_LCD_H__
#define __ILI_LCD_H__
#include "fonts.h"

/* LCD color */
#define White				0xFFFF
#define Black				0x0000
#define Grey				0xF7DE
#define Blue				0x001F
#define Blue2				0x051F
#define Red					0xF800
#define Magenta				0xF81F
#define Green				0x07E0
#define Cyan				0x7FFF
#define Yellow				0xFFE0

/*---------------------- Graphic LCD size definitions ------------------------*/
#define LCD_WIDTH			240                 /* Screen Width (in pixels)           */
#define LCD_HEIGHT      	320                 /* Screen Hight (in pixels)           */
#define BPP             	16                  /* Bits per pixel                     */
#define BYPP            	((BPP+7)/8)         /* Bytes per pixel                    */

/** 
  * @brief LCD default font 
  */ 
#define LCD_DEFAULT_FONT         Font16x24
/** 
  * @brief  LCD Size (Width and Height)  
  */ 
#define LCD_PIXEL_WIDTH          320
#define LCD_PIXEL_HEIGHT         240

#define LCD_DIR_HORIZONTAL       0x0000
#define LCD_DIR_VERTICAL         0x0001

#ifdef __cplusplus
extern "C"{
#endif
extern void lcd_init(void);
extern void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_Clear(uint16_t Color);
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor); 
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor);
void LCD_SetTextColor(__IO uint16_t Color);
void LCD_SetBackColor(__IO uint16_t Color);
void LCD_SetFont(sFONT *fonts);
sFONT *LCD_GetFont(void);
void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c);
void LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii);
void LCD_DisplayStringLine(uint16_t Line, uint8_t *ptr);
void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_DrawUniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

uint16_t LCD_ReadTouchXY(unsigned char mode);
void LCD_TouchEnable(void);
void LCD_TouchDisable(void);

#ifdef __cplusplus
}
#endif

#endif //__ILI_LCD_H__