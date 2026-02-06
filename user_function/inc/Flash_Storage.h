#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#include "main.h"

/****************************************************************************************
* Flash Partition Table (STM32G491CC 256KB)
* Start Addr    Size    Description
* -----------------------------------------------------------
* 0x0800 0000   20KB    Bootloader
* 0x0800 5000   232KB   APP (Application)
* 0x0803 F000   2KB     Parameter Page A (Main)
* 0x0803 F800   2KB     Parameter Page B (Backup)
* 0x0804 0000   -       End of Flash
****************************************************************************************/

// ================= Flash 地址定义 (STM32G491 256KB) =================
// 倒数第2页 (Page 126)
#define FLASH_ADDR_PAGE_A   0x0803F000
// 倒数第1页 (Page 127)
#define FLASH_ADDR_PAGE_B   0x0803F800

// 页面大小 (STM32G4 2KB) - 已在 HAL 库中定义
// #define FLASH_PAGE_SIZE     2048

// 有效标志 (Magic Number)
#define FLASH_VALID_FLAG    0x5A5A5A5A

// ================= 函数声明 =================
void Flash_SaveParams(int32_t *buffer, uint16_t count);
uint8_t Flash_LoadParams(int32_t *buffer, uint16_t count);

#endif
