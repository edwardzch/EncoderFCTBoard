/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_FUNCTION_H
#define __DELAY_FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#define delay_ms HAL_Delay
/* exported functions ------------------------------------------------------- */
void delay_us(uint32_t us);


#ifdef __cplusplus
}
#endif

#endif
