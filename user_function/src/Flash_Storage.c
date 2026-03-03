#include "Flash_Storage.h"
#include <string.h>

// 内部辅助函数声明
static uint32_t Soft_CRC32(const uint32_t *pData, uint32_t len);
static void Flash_Write_Record_Physical(uint32_t addr, uint32_t index, int32_t *data, uint16_t count);
static void Flash_ErasePage_Physical(uint32_t pageAddr);

/****************************************************************************************
* 函数名称：Flash_LoadParams
* 函数功能：扫描双页 Flash，加载最新的有效参数记录到 RAM
* 输入参量：
* - buffer：参数缓冲区指针 (接收加载数据)
* - count：参数个数
* 输出参量：
 * - uint8_t：0=成功, 1=Flash 为空(首次使用, Err.10), 2=CRC 校验全部失败(Err.11)
* 编写日期：2026-2-26
****************************************************************************************/
uint8_t Flash_LoadParams(int32_t *buffer, uint16_t count)
{
    uint32_t pages[2] = {FLASH_ADDR_PAGE_A, FLASH_ADDR_PAGE_B};
    uint32_t max_index = 0;
    uint32_t found_addr = 0;
    uint8_t  found_valid = 0;
    uint8_t  found_any_record = 0;  // 是否找到过任何记录 (含 CRC 错误的)

    // 1. 遍历两个页
    for (int p = 0; p < 2; p++) {
        uint32_t curr_addr = pages[p];
        
        // 遍历页内所有记录
        for (int i = 0; i < RECORDS_PER_PAGE; i++) {
            uint32_t magic = *(__IO uint32_t*)(curr_addr);
            uint32_t index = *(__IO uint32_t*)(curr_addr + 4);

            // 遇到空白 (0xFF)，停止扫描当前页
            if (magic == 0xFFFFFFFF) break;

            if (magic == FLASH_MAGIC_NUM) {
                found_any_record = 1;
                
                uint32_t *pData = (uint32_t *)(curr_addr + 8);
                uint32_t calc_crc = Soft_CRC32(pData, count);
                uint32_t stored_crc = *(__IO uint32_t*)(curr_addr + 8 + RECORD_DATA_SIZE);

                if (calc_crc == stored_crc) {
                    if (!found_valid || index > max_index) {
                        max_index = index;
                        found_addr = curr_addr;
                        found_valid = 1;
                    }
                }
            }
            curr_addr += RECORD_TOTAL_SIZE;
        }
    }

    if (found_valid) {
        uint32_t *src = (uint32_t *)(found_addr + 8);
        for(int i=0; i<count; i++) {
            buffer[i] = (int32_t)src[i];
        }
        return 0; // 成功
    } else if (found_any_record) {
        return 2; // 找到记录但 CRC 全部校验失败
    } else {
        return 1; // Flash 为空 (首次使用)
    }
}

/****************************************************************************************
* 函数名称：Flash_SaveParams
* 函数功能：追加写入参数到 Flash (含满页迁移逻辑)
* 输入参量：
* - buffer：参数缓冲区指针
* - count：参数个数
* 输出参量：无
* 编写日期：2026-2-26
****************************************************************************************/
void Flash_SaveParams(int32_t *buffer, uint16_t count)
{
    uint32_t pages[2] = {FLASH_ADDR_PAGE_A, FLASH_ADDR_PAGE_B};
    uint32_t latest_index = 0;
    int      active_page_idx = -1;
    uint32_t write_addr = 0;

    // 1. 扫描当前状态：寻找最新的 Index 和下一个写入位置
    for (int p = 0; p < 2; p++) {
        uint32_t curr_addr = pages[p];
        
        for (int i = 0; i < RECORDS_PER_PAGE; i++) {
            uint32_t magic = *(__IO uint32_t*)(curr_addr);
            uint32_t index = *(__IO uint32_t*)(curr_addr + 4);

            if (magic == FLASH_MAGIC_NUM) {
                if (index >= latest_index) {
                    latest_index = index;
                    active_page_idx = p;
                }
            } 
            else if (magic == 0xFFFFFFFF) {
                if (active_page_idx == p || active_page_idx == -1) {
                    write_addr = curr_addr;
                    if (active_page_idx == -1) active_page_idx = p;
                    goto CHECK_AND_WRITE; 
                }
                break; 
            }
            curr_addr += RECORD_TOTAL_SIZE;
        }
    }

CHECK_AND_WRITE:

    // 2. 决策：初始化、换页或追加
    
    // [情况 A]: 全新系统
    if (active_page_idx == -1) {
        active_page_idx = 0;
        write_addr = pages[0];
        Flash_ErasePage_Physical(pages[0]);
        Flash_ErasePage_Physical(pages[1]);
        latest_index = 0;
        Flash_Write_Record_Physical(write_addr, 1, buffer, count);
        return;
    }

    // [情况 B]: 需要换页
    uint32_t page_end = pages[active_page_idx] + FLASH_PAGE_SIZE;
    
    if (write_addr == 0 || (write_addr + RECORD_TOTAL_SIZE) > page_end) {
        uint8_t next_page_idx = (active_page_idx == 0) ? 1 : 0;
        uint32_t next_page_addr = pages[next_page_idx];
        Flash_ErasePage_Physical(next_page_addr);
        Flash_Write_Record_Physical(next_page_addr, latest_index + 1, buffer, count);
        Flash_ErasePage_Physical(pages[active_page_idx]);
        return;
    }

    // [情况 C]: 正常追加写入
    Flash_Write_Record_Physical(write_addr, latest_index + 1, buffer, count);
}

/****************************************************************************************
* 函数名称：Flash_Write_Record_Physical
* 函数功能：底层物理写入一条记录 (Header + Data + CRC)
* 输入参量：
* - addr：写入起始地址
* - index：记录序列号
* - data：参数数据指针
* - count：参数个数
* 输出参量：无
* 编写日期：2026-2-26
****************************************************************************************/
static void Flash_Write_Record_Physical(uint32_t addr, uint32_t index, int32_t *data, uint16_t count)
{
    HAL_FLASH_Unlock();

    // 1. 写入 Header (Index | Magic)
    uint64_t header = ((uint64_t)index << 32) | (uint64_t)FLASH_MAGIC_NUM;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, header);
    addr += 8;

    // 2. 写入 Data (每次 64bit)
    for (int i = 0; i < count; i += 2) {
        uint32_t d0 = (uint32_t)data[i];
        uint32_t d1 = (i + 1 < count) ? (uint32_t)data[i+1] : 0xFFFFFFFF;
        
        uint64_t data64 = ((uint64_t)d1 << 32) | (uint64_t)d0;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data64);
        addr += 8;
    }

    // 3. 写入 Footer (Padding | CRC)
    uint32_t crc = Soft_CRC32((uint32_t*)data, count);
    uint64_t footer = ((uint64_t)0xFFFFFFFF << 32) | (uint64_t)crc;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, footer);

    HAL_FLASH_Lock();
}

/****************************************************************************************
* 函数名称：Flash_ErasePage_Physical
* 函数功能：擦除指定 Flash 页
* 输入参量：
* - pageAddr：页起始地址
* 输出参量：无
* 编写日期：2026-2-26
****************************************************************************************/
static void Flash_ErasePage_Physical(uint32_t pageAddr)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef Erase;
    uint32_t PageError;
    
    Erase.TypeErase = FLASH_TYPEERASE_PAGES;
    Erase.Banks     = FLASH_BANK_1;
    Erase.Page      = (pageAddr - 0x08000000) / FLASH_PAGE_SIZE;
    Erase.NbPages   = 1;
    
    HAL_FLASHEx_Erase(&Erase, &PageError);
    HAL_FLASH_Lock();
}

/****************************************************************************************
* 函数名称：Flash_EraseAll
* 函数功能：擦除全部参数存储页 (恢复出厂)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-26
****************************************************************************************/
void Flash_EraseAll(void)
{
    Flash_ErasePage_Physical(FLASH_ADDR_PAGE_A);
    Flash_ErasePage_Physical(FLASH_ADDR_PAGE_B);
}

/****************************************************************************************
* 函数名称：Soft_CRC32
* 函数功能：软件计算 CRC32 校验值
* 输入参量：
* - pData：数据指针 (uint32_t 数组)
* - len：数据个数 (uint32_t 单位)
* 输出参量：
* - uint32_t：CRC32 校验值
* 编写日期：2026-2-26
****************************************************************************************/
static uint32_t Soft_CRC32(const uint32_t *pData, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t data = pData[i];
        crc ^= data;
        for (uint32_t j = 0; j < 32; j++) {
            if (crc & 0x80000000) crc = (crc << 1) ^ 0x04C11DB7;
            else crc <<= 1;
        }
    }
    return crc;
}
