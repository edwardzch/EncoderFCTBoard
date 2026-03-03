/****************************************************************************************
* @file      Parameter_Module.c
* @brief     参数管理模块源文件
* @date      2026-02-26
****************************************************************************************/
#include "Parameter_Module.h"
#include "Flash_Storage.h"
#include "DigitalTube_Control.h" // 引用 DTC_SetError
#include "modbus_function.h"    // FirmwareVersion
#include <string.h>

// 全局参数缓冲区
int32_t PA_Buffer[PA_SIZE];
int32_t DP_Buffer[DP_SIZE];

/****************************************************************************************
* 函数名称：PM_Init
* 函数功能：初始化参数模块，设置出厂默认值并尝试从 Flash 加载
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-26
****************************************************************************************/
void PM_Init(void)
{
    // 1. 设置出厂默认值 (用于 Flash 读取失败时兜底)
    memset(PA_Buffer, 0, sizeof(PA_Buffer));
    memset(DP_Buffer, 0, sizeof(DP_Buffer));
    
    // PA 参数出厂默认值
    PA_Buffer[0] = 5;     // PA000: Modbus 从机地址, 默认 5
    PA_Buffer[1] = 576;   // PA001: 波特率/100, 默认 576 (57600)
    PA_Buffer[2] = 1;     // PA002: 校验位, 默认 1 (odd)
    PA_Buffer[3] = 0;     // PA003: 继电器 K1-K4, 默认全部断开
    PA_Buffer[4] = 0;     // PA004: 继电器 K5-K8, 默认全部断开
    PA_Buffer[5] = 0;     // PA005: 恢复出厂设置 (输入 1234 触发)
    
    // DP 参数初始值 (只读, 运行时更新)
    DP_Buffer[0] = (int32_t)(FirmwareVersion * 10);  // DP000: 固件版本 (1.0 → 10)
    DP_Buffer[1] = 0;     // DP001: 继电器状态 (实时更新)
    
    // 2. 尝试从 Flash 加载 (成功则覆盖上面的默认值)
    uint8_t flash_result = Flash_LoadParams(PA_Buffer, PA_SIZE);
    if (flash_result == 1) {
        DTC_SetError(10); // Err.10: Flash 为空 (首次使用), 使用默认值
    } else if (flash_result == 2) {
        DTC_SetError(11); // Err.11: Flash CRC 校验失败, 数据损坏
    }
}

/****************************************************************************************
* 函数名称：PM_SaveParams
* 函数功能：保存 PA 参数缓冲区到 Flash
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-26
****************************************************************************************/
void PM_SaveParams(void)
{
    Flash_SaveParams(PA_Buffer, PA_SIZE);
    while (FLASH->SR & FLASH_SR_BSY);  // 等待 Flash 编程完全完成, 避免后续外设操作干扰
}

/****************************************************************************************
* 函数名称：PM_GetConfig
* 函数功能：获取指定参数的属性配置 (格式、范围、符号等)
* 输入参量：
* - group：参数组 (0=PA, 1=DP)
* - index：参数编号
* 输出参量：
* - PM_ParamConfig_t：参数属性配置结构体
* 编写日期：2026-2-26
****************************************************************************************/
PM_ParamConfig_t PM_GetConfig(uint8_t group, uint16_t index)
{
    PM_ParamConfig_t cfg;
    // 默认配置: 16位无符号十进制
    cfg.Sign = UNSIGNED;
    cfg.Format = FMT_DEC;
    cfg.Width = BIT_16;
    cfg.Min = 0;
    cfg.Max = 9999;

    // ================= PA 组 =================
    if (group == 0) {
        switch (index) {
            case 0:  // PA000: Modbus 从机地址 (1-255)
                cfg.Min = 1;
                cfg.Max = 255;
                break;
            case 1:  // PA001: 波特率/100
                cfg.Min = 12;
                cfg.Max = 1152;
                break;
            case 2:  // PA002: 校验位 (0=none, 1=odd, 2=even)
                cfg.Min = 0;
                cfg.Max = 2;
                break;
            case 3:  // PA003: 继电器 K1-K4 (二进制)
            case 4:  // PA004: 继电器 K5-K8 (二进制)
                cfg.Format = FMT_BIN;
                cfg.Min = 0;
                cfg.Max = 0xF;
                break;
            case 5:  // PA005: 恢复出厂设置 (输入 1234 触发)
                cfg.Min = 0;
                cfg.Max = 9999;
                break;
            case 10: // PA010: ReadReg 地址 (HEX)
            case 11: // PA011: WriteReg 地址 (HEX)
            case 12: // PA012: WriteReg 数据 (HEX)
                cfg.Format = FMT_HEX;
                cfg.Min = 0;
                cfg.Max = 0xFFFF;
                break;
            default:
                break;
        }
    }
    
    // ================= DP 组 (只读) =================
    if (group == 1) {
        switch (index) {
            case 0:  // DP000: 固件版本
                cfg.Min = 0;
                cfg.Max = 9999;
                break;
            case 1:  // DP001: 继电器状态 (8位二进制)
                cfg.Format = FMT_BIN;
                cfg.Min = 0;
                cfg.Max = 0xFF;
                break;
            case 10: // DP010: ReadReg 返回值 (HEX)
                cfg.Format = FMT_HEX;
                cfg.Min = 0;
                cfg.Max = 0xFFFF;
                break;
            default:
                break;
        }
    }
    return cfg;
}
