#ifndef __BSP_H__
#define __BSP_H__


#ifdef __cplusplus
extern "C" {
#endif
  
//LED¶Ë¿Ú¶¨Òå
#define LED0 PDout(11)	// PD11
#define LED1 PBout(15)	// PB15	
  
void bsp_init(void);
void LED_Toggle(unsigned char led);
#ifdef _cplusplus
}
#endif

#endif //__BSP_H__
