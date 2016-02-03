#ifndef __BSP_H__
#define __BSP_H__


#ifdef __cplusplus
extern "C" {
#endif
void bsp_init(void);
void LED_Toggle(unsigned char led);
#ifdef _cplusplus
}
#endif

#endif //__BSP_H__
