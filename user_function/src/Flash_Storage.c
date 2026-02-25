#include "Flash_Storage.h"
#include <string.h>

// 内部辅助函数声明
static uint32_t Soft_CRC32(const uint32_t *pData, uint32_t len);
static void Flash_Write_Record_Physical(uint32_t addr, uint32_t index, int32_t *data, uint16_t count);
static void Flash_ErasePage_Physical(uint32_t pageAddr);

/****************************************************************************************
* 函数名称：Flash_LoadParams
* 函数功能：扫描双页，加载最新的有效参数
****************************************************************************************/
uint8_t Flash_LoadParams(int32_t *buffer, uint16_t count)
{
    uint32_t pages[2] = {FLASH_ADDR_PAGE_A, FLASH_ADDR_PAGE_B};
    uint32_t max_index = 0;
    uint32_t found_addr = 0;
    uint8_t  found_valid = 0;

    // 1. 遍历两个页
    for (int p = 0; p < 2; p++) {
        uint32_t curr_addr = pages[p];
        
        // 遍历页内所有记录
        for (int i = 0; i < RECORDS_PER_PAGE; i++) {
            // 读取 Header (直接读 Flash 地址)
            uint32_t magic = *(__IO uint32_t*)(curr_addr);
            uint32_t index = *(__IO uint32_t*)(curr_addr + 4);

            // 遇到空白 (0xFF)，说明后面没数据了，停止扫描当前页
            if (magic == 0xFFFFFFFF) break;

            // 检查 Magic 是否有效
            if (magic == FLASH_MAGIC_NUM) {
                // 计算数据区 CRC
                uint32_t *pData = (uint32_t *)(curr_addr + 8); // Header 8 bytes
                uint32_t calc_crc = Soft_CRC32(pData, count);
                
                // 读取存入的 CRC (在 Data 之后)
                uint32_t stored_crc = *(__IO uint32_t*)(curr_addr + 8 + RECORD_DATA_SIZE);

                if (calc_crc == stored_crc) {
                    // 找到有效记录，比对 Index
                    // 注意：这里处理了 Index 溢出翻转的情况 (虽然 uint32 很难溢出)
                    if (!found_valid || index > max_index) {
                        max_index = index;
                        found_addr = curr_addr;
                        found_valid = 1;
                    }
                }
            }
            // 指向下一条记录
            curr_addr += RECORD_TOTAL_SIZE;
        }
    }

    if (found_valid) {
        // 搬运数据到 RAM
        // 数据起始地址 = found_addr + Header(8)
        uint32_t *src = (uint32_t *)(found_addr + 8);
        for(int i=0; i<count; i++) {
            buffer[i] = (int32_t)src[i];
        }
        return 0; // 成功
    } else {
        return 1; // 未找到有效数据 (可能是第一次使用)
    }
}

/****************************************************************************************
* 函数名称：Flash_SaveParams
* 函数功能：追加写入参数 (含满页迁移逻辑 / 完美填充边界处理)
* 修正说明：修复了当页面正好写满时无法触发换页的逻辑隐患
****************************************************************************************/
void Flash_SaveParams(int32_t *buffer, uint16_t count)
{
    uint32_t pages[2] = {FLASH_ADDR_PAGE_A, FLASH_ADDR_PAGE_B};
    uint32_t latest_index = 0;
    int      active_page_idx = -1;
    uint32_t write_addr = 0;

    // ========================================================================
    // 1. 扫描当前状态：寻找最新的 Index 和 下一个写入位置
    // ========================================================================
    for (int p = 0; p < 2; p++) {
        uint32_t curr_addr = pages[p];
        
        // 遍历页内所有记录 (如果是完美填充方案，RECORDS_PER_PAGE = 8)
        for (int i = 0; i < RECORDS_PER_PAGE; i++) {
            uint32_t magic = *(__IO uint32_t*)(curr_addr);
            uint32_t index = *(__IO uint32_t*)(curr_addr + 4);

            if (magic == FLASH_MAGIC_NUM) {
                // 发现有效记录，更新最新序列号
                if (index >= latest_index) {
                    latest_index = index;
                    active_page_idx = p;
                }
            } 
            else if (magic == 0xFFFFFFFF) {
                // 发现空白处 (0xFF)
                // 逻辑：如果我们正在追踪活动页(active_page_idx == p)，或者还没找到活动页(-1)
                // 那么这个空白处就是潜在的写入点
                if (active_page_idx == p || active_page_idx == -1) {
                    write_addr = curr_addr;
                    
                    // 如果系统是全空的，暂定当前页为活动页
                    if (active_page_idx == -1) active_page_idx = p;
                    
                    // 找到写入点后，直接跳出双层循环，去执行写入
                    goto CHECK_AND_WRITE; 
                }
                // 如果发现空白，但不是在活动页（比如活动页是B，但扫描A发现空白），跳过
                break; 
            }
            curr_addr += RECORD_TOTAL_SIZE;
        }
        // 如果内层循环结束都没 break，说明这一页被填满了（完美填充 8 条）
        // 此时 write_addr 保持为 0，active_page_idx 指向这个满页
    }

CHECK_AND_WRITE:

    // ========================================================================
    // 2. 决策：是初始化、换页、还是追加？
    // ========================================================================
    
    // [情况 A]: 全新系统 (两个页都空，或者扫描异常)
    if (active_page_idx == -1) {
        active_page_idx = 0;
        write_addr = pages[0];
        
        // 擦除两页以确保干净
        Flash_ErasePage_Physical(pages[0]);
        Flash_ErasePage_Physical(pages[1]);
        latest_index = 0;
        
        // 执行写入 (Index = 1)
        Flash_Write_Record_Physical(write_addr, 1, buffer, count);
        return;
    }

    // [情况 B]: 需要换页 (Page Swap / Garbage Collection)
    // 触发条件：
    // 1. write_addr 为 0 (说明扫描完发现页满了，没找到空白)
    // 2. write_addr 超出了当前页的范围 (剩余空间不足写一条记录)
    uint32_t page_end = pages[active_page_idx] + FLASH_PAGE_SIZE;
    
    if (write_addr == 0 || (write_addr + RECORD_TOTAL_SIZE) > page_end) {
        // --- 执行页迁移 ---
        uint8_t next_page_idx = (active_page_idx == 0) ? 1 : 0;
        uint32_t next_page_addr = pages[next_page_idx];

        // 1. 擦除新页
        Flash_ErasePage_Physical(next_page_addr);

        // 2. 在新页起始处写入最新数据 (序列号递增: Index + 1)
        Flash_Write_Record_Physical(next_page_addr, latest_index + 1, buffer, count);

        // 3. 擦除旧页 (标记旧页废弃，完成切换)
        Flash_ErasePage_Physical(pages[active_page_idx]);
        
        return; // 完成
    }

    // [情况 C]: 正常追加写入 (Append)
    // 还有空间，直接在 write_addr 处写入
    Flash_Write_Record_Physical(write_addr, latest_index + 1, buffer, count);
}

/****************************************************************************************
* 函数名称：Flash_Write_Record_Physical
* 函数功能：底层物理写入 (处理 64-bit 拼接)
****************************************************************************************/
static void Flash_Write_Record_Physical(uint32_t addr, uint32_t index, int32_t *data, uint16_t count)
{
    HAL_FLASH_Unlock();

    // 1. 写入 Header (Index | Magic)
    // 小端模式：低地址存低位。我们希望内存看是 [Magic][Index]
    // 所以 64bit 应该是 (Index << 32) | Magic
    uint64_t header = ((uint64_t)index << 32) | (uint64_t)FLASH_MAGIC_NUM;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, header);
    addr += 8;

    // 2. 写入 Data (Buffer)
    // 每次写入 2 个 int32 (64bit)
    // 注意：Count 必须是偶数，或者 RECORD_DATA_SIZE 是 8 的倍数
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

static void Flash_ErasePage_Physical(uint32_t pageAddr)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef Erase;
    uint32_t PageError;
    
    Erase.TypeErase = FLASH_TYPEERASE_PAGES;
    Erase.Banks     = FLASH_BANK_1;
    // 计算页号: (Addr - Base) / 2048
    Erase.Page      = (pageAddr - 0x08000000) / FLASH_PAGE_SIZE;
    Erase.NbPages   = 1;
    
    HAL_FLASHEx_Erase(&Erase, &PageError);
    HAL_FLASH_Lock();
}

void Flash_EraseAll(void)
{
    Flash_ErasePage_Physical(FLASH_ADDR_PAGE_A);
    Flash_ErasePage_Physical(FLASH_ADDR_PAGE_B);
}

// 标准 CRC32
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
