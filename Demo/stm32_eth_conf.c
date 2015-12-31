#include "stm32_eth_conf.h"
#include "stm32_eth.h"

#define  ETH_DMARxDesc_FrameLengthShift           16
/* ETHERNET errors */
#define  ETH_ERROR              ((uint32_t)0)
#define  ETH_SUCCESS            ((uint32_t)1)


/** @defgroup ETH_Private_Variables
  * @{
  */

#if defined   (__CC_ARM) /*!< ARM Compiler */
  __align(4)
   ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
  __align(4)
   ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
  __align(4)
   uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; /* Ethernet Receive Buffer */
  __align(4)
   uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; /* Ethernet Transmit Buffer */

#elif defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
   ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
  #pragma data_alignment=4
   ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
  #pragma data_alignment=4
   uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; /* Ethernet Receive Buffer */
  #pragma data_alignment=4
   uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; /* Ethernet Transmit Buffer */

#elif defined (__GNUC__) /*!< GNU Compiler */
  ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB] __attribute__ ((aligned (4))); /* Ethernet Rx DMA Descriptor */
  ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB] __attribute__ ((aligned (4))); /* Ethernet Tx DMA Descriptor */
  uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __attribute__ ((aligned (4))); /* Ethernet Receive Buffer */
  uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __attribute__ ((aligned (4))); /* Ethernet Transmit Buffer */

#elif defined  (__TASKING__) /*!< TASKING Compiler */
  __align(4)
   ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB];/* Ethernet Rx MA Descriptor */
  __align(4)
   ETH_DMADESCTypeDef  DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Tx DMA Descriptor */
  __align(4)
   uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; /* Ethernet Receive Buffer */
  __align(4)
   uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; /* Ethernet Transmit Buffer */

#endif /* __CC_ARM */


/* Global pointers on Tx and Rx descriptor used to track transmit and receive descriptors */
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

/* Structure used to hold the last received packet descriptors info */

ETH_DMA_Rx_Frame_infos RX_Frame_Descriptor;
__IO ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;
__IO uint32_t Frame_Rx_index;


/*******************************************************************************
* Function Name  : ETH_RxPkt_ChainMode
* Description    : Receives a packet.
* Input          : None
* Output         : None
* Return         : frame: farme size and location
*******************************************************************************/
FrameTypeDef ETH_RxPkt_ChainMode(void)
{
	u32 framelength = 0;
	FrameTypeDef frame = {0,0};

	/* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
	if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
	{
		frame.length = ETH_ERROR;

		if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
		{
			/* Clear RBUS ETHERNET DMA flag */
			ETH->DMASR = ETH_DMASR_RBUS;
			/* Resume DMA reception */
			ETH->DMARPDR = 0;
		}

		/* Return error: OWN bit set */
		return frame;
	}

	if(((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (u32)RESET) &&
		((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (u32)RESET) &&
		((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (u32)RESET))
	{
		/* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
		framelength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;

		/* Get the addrees of the actual buffer */
		frame.buffer = DMARxDescToGet->Buffer1Addr;
	}
	else
	{
		/* Return ERROR */
		framelength = ETH_ERROR;
	}

	frame.length = framelength;


	frame.descriptor = DMARxDescToGet;

	/* Update the ETHERNET DMA global Rx descriptor with next Rx decriptor */
	/* Chained Mode */
	/* Selects the next DMA Rx descriptor list for next buffer to read */
	DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);

	/* Return Frame */
	return (frame);
}

/*******************************************************************************
* Function Name  : ETH_TxPkt_ChainMode
* Description    : Transmits a packet, from application buffer, pointed by ppkt.
* Input          : - FrameLength: Tx Packet size.
* Output         : None
* Return         : ETH_ERROR: in case of Tx desc owned by DMA
*                  ETH_SUCCESS: for correct transmission
*******************************************************************************/
u32 ETH_TxPkt_ChainMode(u16 FrameLength)
{
	/* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
	if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
	{
		/* Return ERROR: OWN bit set */
		return ETH_ERROR;
	}

	/* Setting the Frame Length: bits[12:0] */
	DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);

	/* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
	DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;

	/* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
	DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

	/* When Tx Buffer unavailable flag is set: clear it and resume transmission */
	if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)
	{
		/* Clear TBUS ETHERNET DMA flag */
		ETH->DMASR = ETH_DMASR_TBUS;
		/* Resume DMA transmission*/
		ETH->DMATPDR = 0;
	}

	/* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */
	/* Chained Mode */
	/* Selects the next DMA Tx descriptor list for next buffer to send */
	DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);


	/* Return SUCCESS */
	return ETH_SUCCESS;
}

