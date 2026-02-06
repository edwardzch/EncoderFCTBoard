
#include "Flash_Storage.h"
#include <string.h>

#include "Flash_Storage.h"
#include <string.h>

// 内部辅助函数声明
static void Flash_ErasePage(uint32_t pageAddr);
static void Flash_WriteDataWithCRC(uint32_t pageAddr, int32_t *buffer, uint16_t count);
static uint8_t Flash_CheckValidAndCRC(uint32_t pageAddr, int32_t *buffer, uint16_t count);
static uint32_t Soft_CRC32(uint32_t *pData, uint16_t len);

/****************************************************************************************
* 函数名称：Flash_SaveParams
* 函数功能：保存参数到 Flash (双备份机制 + CRC校验)
* 输入参量：
* - buffer: 数据源指针
* - count:  32位数据个数
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
void Flash_SaveParams(int32_t *buffer, uint16_t count)
{
    HAL_FLASH_Unlock();

    // 1. 擦除 Page A 并写入新数据 (含CRC)
    Flash_ErasePage(FLASH_ADDR_PAGE_A);
    Flash_WriteDataWithCRC(FLASH_ADDR_PAGE_A, buffer, count);

    // 2. 擦除 Page B 并写入新数据 (作为备份)
    Flash_ErasePage(FLASH_ADDR_PAGE_B);
    Flash_WriteDataWithCRC(FLASH_ADDR_PAGE_B, buffer, count);

    HAL_FLASH_Lock();
}

/****************************************************************************************
* 函数名称：Flash_LoadParams
* 函数功能：从 Flash 加载参数 (优先加载 Page A, 失败则加载 Page B)
* 输入参量：
* - buffer: 目标缓冲区
* - count:  32位数据个数
* 输出参量：uint8_t (0:成功, 1:失败)
* 编写日期：2026-02-06
****************************************************************************************/
uint8_t Flash_LoadParams(int32_t *buffer, uint16_t count)
{
    // 1. 检查 Page A 是否有效且CRC正确
    if (Flash_CheckValidAndCRC(FLASH_ADDR_PAGE_A, buffer, count) == 0) {
        return 0; // 成功加载
    }
    
    // 2. 如果 Page A 无效, 检查 Page B
    if (Flash_CheckValidAndCRC(FLASH_ADDR_PAGE_B, buffer, count) == 0) {
        // B 有效，恢复到 A (可选，暂不自动恢复以避免复杂情况)
        return 0; // 成功加载
    }
    
    // 3. 都没有有效数据
    return 1; // 失败
}

// ================= 内部底层函数 =================

static void Flash_ErasePage(uint32_t pageAddr)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    // 计算页索引
    uint32_t pageIndex = (pageAddr - 0x08000000) / FLASH_PAGE_SIZE;

    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = FLASH_BANK_1; 
    EraseInitStruct.Page        = pageIndex;
    EraseInitStruct.NbPages     = 1;

    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
}

// 软 CRC32 算法
static uint32_t Soft_CRC32(uint32_t *pData, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for(uint16_t i=0; i<len; i++) {
        uint32_t data = pData[i];
        for(uint8_t j=0; j<32; j++) {
            if (((crc ^ data) & 0x80000000)) {
                crc = (crc << 1) ^ 0x04C11DB7;
            } else {
                crc = (crc << 1);
            }
            data <<= 1;
        }
    }
    return crc;
}

static void Flash_WriteDataWithCRC(uint32_t pageAddr, int32_t *buffer, uint16_t count)
{
    // 计算 CRC (仅计算数据部分)
    uint32_t crc = Soft_CRC32((uint32_t*)buffer, count);
    
    // 构造写入序列: [Magic] [Data...] [CRC] [DummyIfNeed]
    // 必须 64bit (8 bytes) 对齐写入
    
    uint64_t data64 = 0;
    // uint32_t currentTick = 0; // 临时变量 (未使用)
    
    // 1. 写入 Header: [Magic] + [Data0]
    uint32_t d0 = (count > 0) ? (uint32_t)buffer[0] : 0xFFFFFFFF;
    data64 = ((uint64_t)d0 << 32) | (uint64_t)FLASH_VALID_FLAG;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, pageAddr, data64);
    
    uint32_t currentAddr = pageAddr + 8;
    
    // 2. 写入剩余 Data (从 buffer[1] 开始)
    uint16_t i = 1;
    while(i < count) {
        uint32_t val_low = (uint32_t)buffer[i];
        
        // 尝试获取下一个数据，如果没有了，就用 CRC 填充高位
        uint32_t val_high; 
        if (i + 1 < count) {
            val_high = (uint32_t)buffer[i+1];
            // 还没轮到 CRC
        } else {
            val_high = crc; // 数据写完了，补 CRC
            crc = 0; // 标记 CRC 已写入 (通过清零或flag) -> 这里把crc设0不严谨还是用索引判断吧
        }
        
        data64 = ((uint64_t)val_high << 32) | (uint64_t)val_low;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAddr, data64);
        currentAddr += 8;
        i += 2;
    }
    
    // 3. 检查 CRC 是否已写入
    // 如果 count 是偶数 (例如50), buffer[0]在Header, buffer[1]..buffer[49]是49个词(奇数个)。
    // i循环：
    // i=1: low=buf[1], high=buf[2]
    // ...
    // i=49: low=buf[49], high=CRC! (上面循环里 covered)
    // 此时 loop 结束。
    
    // 如果 count 是奇数 (例如1), buffer[0]在Header. i=1. loop条件(1<1)不成立。
    // CRC 还未写入。
    if (i == count) {
         // 剩下 CRC 需要写入，放在低位，高位补F
         data64 = ((uint64_t)0xFFFFFFFF << 32) | (uint64_t)crc;
         HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAddr, data64);
    }
}

static uint8_t Flash_CheckValidAndCRC(uint32_t pageAddr, int32_t *buffer, uint16_t count)
{
    // 1. 检查 Magic
    uint32_t magic = *(__IO uint32_t*)pageAddr;
    if (magic != FLASH_VALID_FLAG) return 1;
    
    // 2. 读取数据 (从Offset 4开始)
    uint32_t *pFlashData = (uint32_t *)(pageAddr + 4);
    for(uint16_t i=0; i<count; i++) {
        buffer[i] = (int32_t)pFlashData[i];
    }
    
    // 3. 读取 Flash 中的 CRC (在数据之后)
    uint32_t stored_crc = pFlashData[count]; 
    
    // 4. 计算当前数据的 CRC
    uint32_t cal_crc = Soft_CRC32((uint32_t*)buffer, count);
    
    if (stored_crc == cal_crc) return 0; // 正常
    else return 2; // CRC 错误
}
