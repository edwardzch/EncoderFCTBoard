/****************************************************************************************
  * @file     函数名称：iap_function.c
  * @brief    IAP函数配置
  *           
****************************************************************************************/
#include "iap_function.h"
#include <stdarg.h> // 用于处理可变参数
#include <string.h>
#include "uart_config.h"

/*
| Area         						| Starting address| Size   | End address  | 说明            					 |
| ------------------------|-----------------| -------| -------------| ---------------------------|
| BootLoader 	 						| 0x08000000      | 16 KB  | 0x08003FFF   | 存放 IAP 程序               |
| APP Main Area						| 0x08004000      | 480 KB | 0x08079FFF   | 存放用户程序                |
| Reserved area (optional)| 0x0807A000      | 28 KB  | 0x0807FFFF   | 存放参数、备份、升级标志等   |

| 字段      | 长度 | 说明                                     |
| ------- | -- | -------------------------------------- |
| header1 | 1  | 固定 `0x55`                              |
| header2 | 1  | 固定 `0xAA`                              |
| length  | 2  | 数据长度（小于等于 1024）                        |
| address | 4  | FLASH 写入地址                             |
| payload | n  | 数据内容（最多 1024 字节）                       |
| crc16   | 2  | `length+address+payload` CRC16(Modbus) |


上电/复位
   │
   ▼
[IAP Bootloader 启动]
   │
   ├─ 串口等待 3s，是否收到 "IAP" ?
   │
   ├─ NO ─→ 跳转 APP (0x08004000)
   │
   └─ YES
        │
        ▼
   "Update Mode" 回复
        │
   擦除 APP Flash
        │
   循环接收数据帧
        │
   ├─ 校验 CRC 正确？
   │      ├─ NO → 回复 "CRCERR"
   │      └─ YES → 写入 Flash → 回复 "OK"
        │
   ├─ 是否收到结束帧？
   │      ├─ NO → 等待下一个包
   │      └─ YES → 回复 "DONE"
        │
        ▼
   跳转到 APP
*/

/* 计算 APP 区的擦除页数量 */
#define APP_OFFSET          (APP_ADDRESS - FLASH_BASE_ADDR)
#define APP_AREA_SIZE       (FLASH_TOTAL_SIZE - APP_OFFSET)
#define APP_NBPAGES         ( (APP_AREA_SIZE + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE )

//****************************************************************************************
//* 函数名称：IAP_CRC16_Calc()
//* 函数功能：计算 CRC16-Modbus 校验值
//* 输入参量：data -> 数据指针；len -> 数据长度
//* 输出参量：返回16位 CRC 校验值
//* 编写日期：2025-9-02
//****************************************************************************************/
uint16_t IAP_CRC16_Calc(uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

//****************************************************************************************
//* 函数名称：IAP_Flash_Write()
//* 函数功能：向 Flash 指定地址写入数据（以双字为单位）
//* 输入参量：address -> 写入地址；data -> 数据指针；length -> 数据长度（字节数）
//* 输出参量：无
//* 编写日期：2025-9-02
//****************************************************************************************/
void IAP_Flash_Write(uint32_t address, uint8_t *data, uint32_t length)
{
    if (length == 0) return;
    /* 地址对齐检查（可选） */
    /* 注意：写入不能跨越受保护区域，建议在调用端检查地址合法性 */

    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < length; i += 8)
    {
        uint64_t data64 = 0xFFFFFFFFFFFFFFFFULL;
        uint32_t copy_len = (length - i >= 8) ? 8U : (length - i);
        memcpy(&data64, &data[i], copy_len);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, data64) != HAL_OK)
        {
            /* 写失败处理：可返回错误码或打印信息 */
            Usart1_Print("FLASH_WR_ERR\r\n");
            break;
        }
    }

    HAL_FLASH_Lock();
}

//****************************************************************************************
//* 函数名称：IAP_Flash_EraseApp()
//* 函数功能：擦除应用程序所在区域的 Flash
//* 输入参量：无
//* 输出参量：无
//* 编写日期：2025-9-02
//****************************************************************************************/
void IAP_Flash_EraseApp(void)
{
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;

    HAL_FLASH_Unlock();

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Page = APP_OFFSET / FLASH_PAGE_SIZE;
    eraseInit.NbPages = (uint32_t)APP_NBPAGES;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
    {
        /* 擦除失败可记录 pageError */
        Usart1_Print("FLASH_ERASE_ERR\r\n");
    }

    HAL_FLASH_Lock();
}

//****************************************************************************************
//* 函数名称：IAP_JumpToApplication()
//* 函数功能：跳转到应用程序入口
//* 输入参量：无
//* 输出参量：无
//* 编写日期：2025-9-02
//****************************************************************************************/
void IAP_JumpToApplication(void)
{
    uint32_t appStack = *(uint32_t*)APP_ADDRESS;
    uint32_t appResetHandler = *(uint32_t*)(APP_ADDRESS + 4);

    /* 关闭中断，确保跳转过程不被打断 */
    __disable_irq();

    /* 可选择停用 SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* 停用外设（通过 HAL）以清理状态 */
    HAL_DeInit();

    /* 设置主堆栈指针 (MSP) */
    __set_MSP(appStack);

    /* 使能中断（如果需要，跳转后由 APP 配置） */
    __enable_irq();

    /* 跳转到复位处理函数 */
    ((void (*)(void))appResetHandler)();
}

//****************************************************************************************
//* 函数名称：IAP_RequestUpdate()
//* 函数功能：写入 BootLoader 标志，触发系统复位
//* 输入参量：无
//* 输出参量：无
//* 编写日期：2025-09-02
//****************************************************************************************/
void IAP_RequestUpdate(void)
{
    /* 使能访问备份域 */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    /* 使能 TAMP 时钟（部分系列需） */
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_TAMPEN);

    /* 写 TAMP 的 Backup 寄存器 BKP0R（或使用 RTC backup） */
    WRITE_REG(TAMP->BKP0R, 0xA5A5U);

    /* 系统复位 */
    NVIC_SystemReset();
}

//****************************************************************************************
//* 函数名称：Check_IAP_Flag()
//* 函数功能：检查是否需要进入 IAP 模式（读取备份寄存器）
//* 输入参量：无
//* 输出参量：1=需要升级，0=跳 APP
//* 编写日期：2025-09-02
//****************************************************************************************/
uint8_t Check_IAP_Flag(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_TAMPEN);

    uint32_t flag = READ_REG(TAMP->BKP0R);
    if (flag == 0xA5A5U)
    {
        /* 清除标志，避免重复进入 */
        WRITE_REG(TAMP->BKP0R, 0x0000U);
        return 1U;
    }
    return 0U;
}

//****************************************************************************************
//* 函数名称：IAP_ParseFrame()
//* 函数功能：解析一帧 IAP 数据（由 USART1 IDLE 中断触发）
//* 输入参量：buf -> 数据缓冲区；len -> 数据长度
//* 输出参量：无
//* 编写日期：2025-9-02
//****************************************************************************************/
void IAP_ParseFrame(uint8_t *buf, uint16_t len)
{
    /* 最小帧： header(2) + len(2) + addr(4) + crc(2) -> 10 字节 */
    if (len < 10) return;

    if (buf[0] != IAP_HEADER1 || buf[1] != IAP_HEADER2) return;

    uint16_t payload_len = (uint16_t)(buf[2] | (buf[3] << 8));
    uint32_t address = (uint32_t)(buf[4] | (buf[5] << 8) | (buf[6] << 16) | (buf[7] << 24));

    /* 检查长度边界 */
    if (payload_len > IAP_MAX_PAYLOAD) 
    {
        Usart1_Print("LEN_ERR\r\n");
        return;
    }

    /* 校验整帧长度（header+len+addr+payload+crc） */
    if ((uint32_t)len < (uint32_t)(10 + payload_len)) return;

    uint16_t crc_recv = (uint16_t)(buf[8 + payload_len] | (buf[9 + payload_len] << 8));
    /* CRC 计算：对 Length(2)+Address(4)+Payload(N) 做 CRC */
    uint16_t crc_calc = IAP_CRC16_Calc(&buf[2], (uint32_t)(6 + payload_len));

    if (crc_recv != crc_calc)
    {
        Usart1_Print("CRCERR\r\n");
        return;
    }

    /* 文件结束标志 */
    if (payload_len == 0U && address == 0xFFFFFFFFU)
    {
        Usart1_Print("DONE\r\n");
        /* 跳转到应用 */
        IAP_JumpToApplication();
        return;
    }

    /* 地址合法性检查：必须在 APP 区域内 */
    if ((address < APP_ADDRESS) || (address + payload_len > FLASH_BASE_ADDR + FLASH_TOTAL_SIZE))
    {
        Usart1_Print("ADDR_ERR\r\n");
        return;
    }

    /* 写 Flash */
    IAP_Flash_Write(address, &buf[8], payload_len);
    Usart1_Print("OK\r\n");
}

//****************************************************************************************
//* 函数名称：IAP_Run()
//* 函数功能：进入 IAP 模式（在 BootLoader main 中由 Check_IAP_Flag 决定进入）
//* 输入参量：无
//* 输出参量：无
//* 编写日期：2025-9-02
//****************************************************************************************/
void IAP_Run(void)
{
    /* 告知上位机进入升级模式 */
    Usart1_Print("Update Mode\r\n");

    /* 擦除 APP 区域 */
    IAP_Flash_EraseApp();

    /* 进入 IAP 循环：接收由中断向 Usart1.RxData 填充，IDLE 中断时调用 IAP_ParseFrame */
    while (1)
    {
        /* 在中断中处理，不在此阻塞；可做看门狗喂养或低功耗 */
        HAL_Delay(100);
    }
}

//****************************************************************************************
//* 函数名称：APP_RelocateVectorTable()
//* 函数功能：将中断向量表重定位到 APP 起始地址
//* 输入参量：无
//* 输出参量：无
//* 编写日期：2025-9-02
//****************************************************************************************/
void APP_RelocateVectorTable(void)
{
    SCB->VTOR = APP_ADDRESS;
    __DSB();
    __ISB();
}
