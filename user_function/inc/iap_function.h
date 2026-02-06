/* iap_function.h */
#ifndef __IAP_FUNCTION_H
#define __IAP_FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/*
  APP 起始地址，根据你工程实际设置（你给出的是 0x08005000）
  Bootloader 占用 0x08000000 ~ 0x08004FFF (0x5000 = 20KB)
*/
#define FLASH_BASE_ADDR    0x08000000U
#define APP_ADDRESS        0x08005000U
#define FLASH_TOTAL_SIZE   (512U * 1024U)    /* 512KB */
#define FLASH_PAGE_SIZE    0x800U            /* 2KB */
#define IAP_HEADER1        0x55U
#define IAP_HEADER2        0xAAU
#define IAP_MAX_PAYLOAD    1024U

// 替代方案：直接使用数值
#define RCC_APB1ENR1_TAMPEN	SET_BIT(RCC->APB1ENR1, 0x00000001)// TAMPEN 通常是 APB1ENR1 的第 0 位

/* exported functions */
uint16_t IAP_CRC16_Calc(uint8_t *data, uint32_t len);
void IAP_Flash_Write(uint32_t address, uint8_t *data, uint32_t length);
void IAP_Flash_EraseApp(void);
void IAP_JumpToApplication(void);
void IAP_Run(void);
void IAP_ParseFrame(uint8_t *buf, uint16_t len);

void IAP_RequestUpdate(void);
uint8_t Check_IAP_Flag(void);

void APP_RelocateVectorTable(void);

#ifdef __cplusplus
}
#endif

#endif /* __IAP_FUNCTION_H */
