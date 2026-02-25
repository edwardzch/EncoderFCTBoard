#ifndef _UART_CONGFUG_H_
#define _UART_CONGFUG_H_

#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include "fsl_uart_edma.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
//==============================================================================
// Definitions
//==============================================================================
#define Encoder485DE()                          (GPIOE->PSOR = 0x2000000)
#define Encoder485RE()                          (GPIOE->PCOR = 0x2000000)
#define SerialPort485DE()                       (GPIOD->PSOR = 0x0004)
#define SerialPort485RE()                       (GPIOD->PCOR = 0x0004)
//==============================================================================
// Functions
//==============================================================================
void Init_UART0(uint32_t SetBaudRate);
void Init_UART1(void);
#endif /* _UART_CONGFUG_H_ */
