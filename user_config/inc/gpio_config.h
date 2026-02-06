/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_CONFIG_H
#define __GPIO_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* exported functions ------------------------------------------------------- */
uint16_t Get_GPIO_Output_Status(void);
uint8_t Get_Relay_Status_By_StationID(uint8_t station_id);
#ifdef __cplusplus
}
#endif

#endif
