#include "uart_config.h"
#include "ModBus.h"
//==============================================================================
// DEFINES
//==============================================================================

//==============================================================================
// DATA
//==============================================================================
volatile uint8_t   EncTxData[128] = {0};
//==============================================================================
// Code
//==============================================================================
void Init_UART0(uint32_t SetBaudRate)
{
    uart_config_t uart0_config;

//------------------------------------------------------------------------------ 
// Clock enabled
//------------------------------------------------------------------------------ 
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortE);
    CLOCK_EnableClock(kCLOCK_Dmamux0);
    CLOCK_EnableClock(kCLOCK_Dma0);
    CLOCK_EnableClock(kCLOCK_Uart0);

//------------------------------------------------------------------------------ 
// UART0 pinmux function
// PIN18--PTA1 : RS485RX
// PIN19--PTA2 : RS485TX
//------------------------------------------------------------------------------ 
    PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2); 
    
//------------------------------------------------------------------------------ 
// OUTPUT
// PIN16--PTE25 : RS485DE //DORNA
//------------------------------------------------------------------------------   
    PORT_SetPinMux(PORTE, 25U, kPORT_MuxAsGpio);
    GPIOE->PDDR |= 0x2000000;                                                   //Pin is configured as general-purpose output
    Encoder485DE();                                                                  //Read enabled 

    SIM->SOPT5 = ((SIM->SOPT5 
                 & (~(SIM_SOPT5_UART0TXSRC_MASK)))                              //Mask bits to zero which are setting 
                 | SIM_SOPT5_UART0TXSRC(0));                                    //UART 0 transmit data source select: UART0_TX pin. 
  
//------------------------------------------------------------------------------ 
// DMAMUX0 Configuration, Set channel for UART
//------------------------------------------------------------------------------ 
    DMAMUX0->CHCFG[0] &= ~DMAMUX_CHCFG_SOURCE_MASK;
    DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(kDmaRequestMux0UART0Tx);
    DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK;
//------------------------------------------------------------------------------ 
// DMA0 Configuration
//------------------------------------------------------------------------------ 
    DMA0->CR = 0;
    DMA0->TCD[0].SADDR = (uint32_t)&EncTxData[0];                               //DMA source addr
    DMA0->TCD[0].DADDR = (uint32_t)&UART0->D;                                   //DMA dest   addr
    DMA0->TCD[0].NBYTES_MLNO = 1;
    DMA0->TCD[0].ATTR = 0;                                                      //8bit transfer
    DMA0->TCD[0].SOFF = 1;                                                      //Source address offset increase 1
    DMA0->TCD[0].DOFF = 0;                                                      //Destination address offset not increase 1
    DMA0->TCD[0].SLAST = 0;  							//major_loop finish do not modify the source address
    DMA0->TCD[0].DLAST_SGA = 0;							//major_loop finish do not modify the destination address
    DMA0->TCD[0].CITER_ELINKNO = 16;
    DMA0->TCD[0].BITER_ELINKNO = 16;
    DMA0->TCD[0].CSR = 0;
    DMA0->TCD[0].CSR &=~DMA_CSR_INTMAJOR_MASK;
    DMA0->TCD[0].CSR |=	DMA_CSR_DREQ_MASK;
    DMA0->ERQ |= DMA_ERQ_ERQ0_MASK;   
//------------------------------------------------------------------------------ 
// UART0 Init
//------------------------------------------------------------------------------ 
    uart0_config.baudRate_Bps    = SetBaudRate;
    uart0_config.parityMode      = kUART_ParityDisabled;
    uart0_config.txFifoWatermark = 0;
    uart0_config.rxFifoWatermark = 1;
    uart0_config.idleType        = kUART_IdleTypeStartBit;
    uart0_config.enableTx        = true;
    uart0_config.enableRx        = false;
    uart0_config.enableRxRTS     = false;
    uart0_config.enableTxCTS     = false;
    UART_Init(UART0, &uart0_config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    
//------------------------------------------------------------------------------ 
// Enable UART0 DMA and Interrupt
//------------------------------------------------------------------------------ 
    UART_EnableTxDMA(UART0, true);
    UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable);
    NVIC_SetPriority(UART0_RX_TX_IRQn, 2);
    EnableIRQ(UART0_RX_TX_IRQn);  

    EDMA_EnableChannelInterrupts(DMA0, 0U, kEDMA_MajorInterruptEnable);
    NVIC_SetPriority(DMA0_IRQn, 2);
    EnableIRQ(DMA0_IRQn);  
    
}

void Init_UART1(void)
{
    uart_config_t uart1_config;
//------------------------------------------------------------------------------ 
// Clock enabled
//------------------------------------------------------------------------------ 
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_Dmamux0);
    CLOCK_EnableClock(kCLOCK_Dma0);
    CLOCK_EnableClock(kCLOCK_Uart1);

//------------------------------------------------------------------------------ 
// UART1 pinmux function
// PIN41--PTD0 : RS485RX
// PIN42--PTD1 : RS485TX
//------------------------------------------------------------------------------ 
    PORT_SetPinMux(PORTD, 0U, kPORT_MuxAlt5);
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt5); 
    
//------------------------------------------------------------------------------ 
// OUTPUT
// PIN43--PTD2 : RTU485DE 
//------------------------------------------------------------------------------   
    PORT_SetPinMux(PORTD, 2U, kPORT_MuxAsGpio);
    GPIOD->PDDR |= 0x0004;                                                      //Pin is configured as general-purpose output
    SerialPort485RE();                                                                  //Read enabled 

    SIM->SOPT5 = ((SIM->SOPT5 
                 & (~(SIM_SOPT5_UART1TXSRC_MASK)))                              //Mask bits to zero which are setting 
                 | SIM_SOPT5_UART1TXSRC(0));                                    //UART 0 transmit data source select: UART1_TX pin. 
  
//------------------------------------------------------------------------------ 
// DMAMUX0 Configuration, Set channel for UART1
//------------------------------------------------------------------------------ 
    DMAMUX0->CHCFG[1] &= ~DMAMUX_CHCFG_SOURCE_MASK;
    DMAMUX0->CHCFG[1] |= DMAMUX_CHCFG_SOURCE(kDmaRequestMux0UART1Tx);   
    DMAMUX0->CHCFG[1] |= DMAMUX_CHCFG_ENBL_MASK;
//------------------------------------------------------------------------------ 
// DMA0 TX Configuration
//------------------------------------------------------------------------------ 
    DMA0->CR = 0;
    DMA0->TCD[1].SADDR = (uint32_t)&ModBus.TX.Buffer[0];                           //DMA source addr
    DMA0->TCD[1].DADDR = (uint32_t)&UART1->D;                                   //DMA dest   addr
    DMA0->TCD[1].NBYTES_MLNO = 1;                                               //每次DMA请求需要传输的字节数
    DMA0->TCD[1].ATTR = 0;                                                      //8bit transfer
    DMA0->TCD[1].SOFF = 1;                                                      //源地址偏移增1
    DMA0->TCD[1].DOFF = 0;                                                      //目的地址不变
    DMA0->TCD[1].SLAST = 0;  							//主循环结束后，源地址调整量major_loop finish do not modify the source address
    DMA0->TCD[1].DLAST_SGA = 0;							//主循环结束后，目的地址调整量major_loop finish do not modify the destination address
    DMA0->TCD[1].CITER_ELINKNO = 58;
    DMA0->TCD[1].BITER_ELINKNO = 58;
    DMA0->TCD[1].CSR = 0;
    DMA0->TCD[1].CSR |= DMA_CSR_INTMAJOR_MASK;                                  //打开使能中断
    DMA0->TCD[1].CSR |= DMA_CSR_DREQ_MASK;                                      //主循环结束后停止硬件请求
    DMA0->ERQ |= DMA_ERQ_ERQ1_MASK;                                             //打开DMA使能    
//------------------------------------------------------------------------------ 
// UART0 Init           8，O，1（Modbus，ASCII / RTU）
//------------------------------------------------------------------------------ 
    uart1_config.baudRate_Bps    = 57600;
    uart1_config.parityMode      = kUART_ParityOdd;  
    uart1_config.txFifoWatermark = 0;
    uart1_config.rxFifoWatermark = 1;
    uart1_config.idleType        = kUART_IdleTypeStartBit;
    uart1_config.enableTx        = true;
    uart1_config.enableRx        = true;
    uart1_config.enableRxRTS     = false;
    uart1_config.enableTxCTS     = false;
    UART_Init(UART1, &uart1_config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
        
//------------------------------------------------------------------------------ 
// Enable UART0 DMA and Interrupt
//------------------------------------------------------------------------------ 
    UART_EnableTxDMA(UART1, true);
    UART_EnableInterrupts(UART1, kUART_RxDataRegFullInterruptEnable | kUART_IdleLineInterruptEnable);
    NVIC_SetPriority(UART1_RX_TX_IRQn, 3);
    EnableIRQ(UART1_RX_TX_IRQn);  

    EDMA_EnableChannelInterrupts(DMA0, 1U, kEDMA_MajorInterruptEnable);
    NVIC_SetPriority(DMA1_IRQn, 3);
    EnableIRQ(DMA1_IRQn);  
  
}


