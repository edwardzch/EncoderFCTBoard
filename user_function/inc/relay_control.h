/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __RELAY_CONTROL_H
#define __RELAY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"

/* defines -------------------------------------------------------------------*/
#define RELAY_COUNT     8

/* function prototypes -------------------------------------------------------*/
void Relay_Init(void);
void Relay_On(uint8_t relayNum);
void Relay_Off(uint8_t relayNum);
void Relay_Toggle(uint8_t relayNum);
void Relay_AllOn(void);
void Relay_AllOff(void);
uint8_t Relay_GetStatus(uint8_t relayNum);
void Relay_SetMultiple(uint8_t mask, uint8_t state);

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_CONTROL_H */
