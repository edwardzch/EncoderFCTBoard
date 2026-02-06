/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_CONFIG_H
#define __UART_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#define Usart1TxEnable() 					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define Usart1RxEnable()					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
         	
#define Usart1TxSize         0x100
#define Usart1RxSize         0x100

typedef struct
{
	uint8_t *Data;
	uint8_t DataSize;
} strUsart1Tx;


typedef struct{
  uint8_t  		TxData[Usart1TxSize];
  uint8_t  		RxData[Usart1RxSize];	
	uint16_t    DataCnt;          // 接收到的数据长度
	uint8_t     StringFlag;       // 字符串接收完成标志
	strUsart1Tx Tx;
} strUsart1;	

extern volatile strUsart1   Usart1;

void uart_config(void);
void Usart1TransmitterDMA(volatile strUsart1Tx * p);
void DisableUARTReceive(UART_HandleTypeDef *huart);
void EnableUARTReceive(UART_HandleTypeDef *huart);
void Usart1_Print(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif
