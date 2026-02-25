/****************************************************************************************
* @file      Parameter_Module.c
* @brief     参数管理模块源文件
* @author    Gemini
* @date      2026-02-09
****************************************************************************************/
#include "Parameter_Module.h"
#include "Flash_Storage.h"
#include "DigitalTube_Control.h" // 引用 DTC_SetError
#include <string.h>

// 全局参数缓冲区
int32_t PA_Buffer[PA_SIZE];
int32_t DP_Buffer[DP_SIZE];

/****************************************************************************************
* 函数名称：PM_Init
* 函数功能：初始化参数模块
****************************************************************************************/
void PM_Init(void)
{
    // 1. 设置出厂默认值 (用于 Flash 读取失败时兜底)
    memset(PA_Buffer, 0, sizeof(PA_Buffer));
    memset(DP_Buffer, 0, sizeof(DP_Buffer));
    
    // 2. 尝试从 Flash 加载
    // Load_PA_From_Flash -> 增加返回值判断
    if (Flash_LoadParams(PA_Buffer, PA_SIZE) != 0) {
        DTC_SetError(1); // Err.01: Flash 空或 CRC 错误
    }
}

/****************************************************************************************
* 函数名称：PM_SaveParams
* 函数功能：保存参数到 Flash
****************************************************************************************/
void PM_SaveParams(void)
{
    Flash_SaveParams(PA_Buffer, PA_SIZE);
}

/****************************************************************************************
* 函数名称：PM_GetConfig
* 函数功能：获取每个参数的属性配置
****************************************************************************************/
PM_ParamConfig_t PM_GetConfig(uint8_t group, uint16_t index)
{
    PM_ParamConfig_t cfg;
    // 默认配置: 16位有符号十进制
    cfg.Sign = SIGNED;
    cfg.Format = FMT_DEC;
    cfg.Width = BIT_16;
    cfg.Min = -9999;
    cfg.Max = 9999;

    // 自定义特殊参数示例
    if (group == 0 && index == 0) { // PA000: 32位大数
        cfg.Min = -6000; 
        cfg.Max = 6000; 
    }
    if (group == 0 && index == 1) { // PA001: 16进制
        cfg.Format = FMT_HEX; 
        cfg.Min = 0; 
        cfg.Max = 0xFFFF;
    }
    
    // DP 组配置 (只读在 UI 层控制，这里只定义格式)
    if (group == 1 && index == 0) { // DP000: 2进制
				cfg.Width = BIT_32; 
        cfg.Format = FMT_DEC; 
        cfg.Min = 0; 
        cfg.Max = 0xF;
    }
    return cfg;
}
