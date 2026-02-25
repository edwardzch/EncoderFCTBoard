#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#include "main.h"
#include "Parameter_Module.h" // 必须引用，以获取 PA_SIZE

// ================= Flash 物理配置 =================
// STM32G491CCU3 (256KB Flash)
// Page Size = 2KB (0x800)
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE         0x800
#endif

// 【修复点】补上了 Page A 的定义
#define FLASH_ADDR_PAGE_A       0x0803F000  // Page 126 (倒数第2页)
#define FLASH_ADDR_PAGE_B       0x0803F800  // Page 127 (倒数第1页)

/**
 * ================= Flash 存储布局说明 (Append-Only) =================
 *
 * 1. 记录结构 (Total: 256 Bytes)  <-- 更新为完美填充版数值
 * +----------------+--------------------------+----------------+
 * | Header (8B)    | Data (240B)              | Footer (8B)    |
 * +----------------+--------------------------+----------------+
 * | Magic (4B)     | PA000 (4B)               | CRC32 (4B)     |
 * | Index (4B)     | ...                      | Padding (4B)   |
 * |                | PA059 (4B)               |                |
 * +----------------+--------------------------+----------------+
 * Magic: 0xA5A5A5A5 (标识有效记录)
 * Index: 递增序列号 (1, 2, 3...)
 *
 * 2. Page 布局 (2KB / 2048 Bytes)
 * +----------------+----------------+-----+----------------+
 * | Record 1       | Record 2       | ... | Record 8       |
 * +----------------+----------------+-----+----------------+
 * | Base + 0       | Base + 256     | ... | Base + 1792    |
 * +----------------+----------------+-----+----------------+
 * * 注意：2048 / 256 = 8.0，刚好存 8 条，无浪费。
 */
#define FLASH_MAGIC_NUM         0xA5A5A5A5

// 1. 数据负载大小 (必须是 8 的倍数)
// PA_SIZE 应在 Parameter_Module.h 中定义为 60
#define PARAM_COUNT             PA_SIZE
#define RECORD_DATA_SIZE        (PARAM_COUNT * 4)

// 2. 头部 (8 Bytes)
typedef struct {
    uint32_t Magic;     // 0xA5A5A5A5
    uint32_t Index;     // 序列号
} Flash_Header_t;

// 3. 尾部 (8 Bytes)
typedef struct {
    uint32_t CRC32;     // 校验码
    uint32_t Padding;   // 填充 (保留)
} Flash_Footer_t;

// 单条记录总大小 = 8 + 240 + 8 = 256 Bytes
// 注意：必须保证 RECORD_TOTAL_SIZE % 8 == 0 (64位对齐)
#define RECORD_TOTAL_SIZE       (sizeof(Flash_Header_t) + RECORD_DATA_SIZE + sizeof(Flash_Footer_t))

// 计算一页能存多少条记录 (2048 / 256 = 8)
#define RECORDS_PER_PAGE        (FLASH_PAGE_SIZE / RECORD_TOTAL_SIZE)

// ================= 接口声明 =================
uint8_t Flash_LoadParams(int32_t *buffer, uint16_t count);
void Flash_SaveParams(int32_t *buffer, uint16_t count);
void Flash_EraseAll(void);

#endif
