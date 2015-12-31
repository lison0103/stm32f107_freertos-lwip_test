#ifndef __STM32_ETH_BSP_H__
#define __STM32_ETH_BSP_H__

#include "stm32f107_eth_conf.h"
/* Exported functions ------------------------------------------------------- */
#ifdef __cplusplus
 extern "C" {
#endif

//#define MII_MODE          /* MII mode for STM3210C-EVAL Board (MB784) (check jumpers setting) */
#define RMII_MODE       /* RMII mode for STM3210C-EVAL Board (MB784) (check jumpers setting) */

void  ETH_BSP_Config(void);
uint32_t Eth_Link_PHYITConfig(uint16_t PHYAddress);
void Eth_Link_EXTIConfig(void);
void Eth_Link_ITHandler(uint16_t PHYAddress);

#ifdef __cplusplus
 }
#endif

#endif //__STM32_ETH_BSP_H__
