/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_FUNCTION_H
#define __DELAY_FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#define delay_ms HAL_Delay

#define PWR_CTRL_Enable() 					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
/* exported functions ------------------------------------------------------- */
void delay_us(uint32_t us);


#ifdef __cplusplus
}
#endif

#endif
