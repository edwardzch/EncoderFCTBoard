#include "fsl_port.h"
#include "fsl_dspi.h"
#include "fsl_dspi_edma.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"

//==============================================================================
// Definitions
//==============================================================================
//#define SPI0_PERIPHERAL SPI0
///* Definition of the clock source */
//#define SPI0_CLOCK_SOURCE DSPI0_CLK_SRC
///* Definition of the clock source frequency */
//#define SPI0_CLK_FREQ 47988736UL
///* SPI0 interrupt vector ID (number). */
//#define SPI0_IRQN SPI0_IRQn
///* SPI0 interrupt handler identifier. */
//#define SPI0_IRQHANDLER SPI0_IRQHandler
//#define EXAMPLE_DSPI_MASTER_DMA_MUX_BASEADDR DMAMUX0
//#define EXAMPLE_DSPI_MASTER_DMA_RX_REQUEST_SOURCE kDmaRequestMux0SPI0Rx
//#define EXAMPLE_DSPI_MASTER_DMA_TX_REQUEST_SOURCE kDmaRequestMux0SPI0Tx
//#define EXAMPLE_DSPI_MASTER_DMA_BASEADDR DMA0
//#define EXAMPLE_DSPI_MASTER_BASEADDR SPI0

//==============================================================================
// Functions
//==============================================================================
void Init_SPI0(void);
__ramfunc status_t MT6835_SPI_Tx(uint8_t* masterTxData, uint8_t* masterRxData, uint8_t transfer_dataSize);
