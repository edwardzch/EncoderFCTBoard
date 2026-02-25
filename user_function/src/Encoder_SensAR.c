/****************************************************************************************
 * @file      Encoder_SensAR.c
 * @brief     SensAR 编码器协议实现 (ASCII 协议)
 ****************************************************************************************/
#include "Encoder_SensAR.h"
#include "Encoder_MultiturnMag.h"  // for MotorEncoder_t
#include "encoder_driver.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ================= 全局变量 =================
SensAR_t g_SensAR = {0};

extern volatile uint8_t Work_Alarm;

/****************************************************************************************
* 函数名称：SensAR_Init
* 函数功能：初始化 SensAR 编码器模块，清零状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SensAR_Init(void)
{
    memset(&g_SensAR, 0, sizeof(g_SensAR));
}

/****************************************************************************************
* 函数名称：SensAR_Crc
* 函数功能：CRC 计算（简单累加校验）
* 输入参量：data 数据指针，len 数据长度
* 输出参量：校验值
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t SensAR_Crc(const char *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc += *data++;
    }
    return crc;
}

/****************************************************************************************
* 函数名称：SensAR_TX
* 函数功能：发送 SensAR 编码器 ASCII 命令
* 输入参量：cmd 命令字符串
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SensAR_TX(const char *cmd)
{
    uint8_t len = strlen(cmd);
    uint8_t crc = SensAR_Crc(cmd, len);
    
    // 格式: "command"<XX>\r
    int tx_len = snprintf(g_SensAR.TxData, SENSAR_TX_SIZE, "%s<%02X>\r", cmd, crc);
    if (tx_len > 0 && tx_len < SENSAR_TX_SIZE) {
        EncDrv_SendDMA((uint8_t*)g_SensAR.TxData, tx_len, SENSAR_RX_SIZE);
    }
}

/****************************************************************************************
* 函数名称：SensAR_RxHandler
* 函数功能：接收处理（逐字节），存储接收数据
* 输入参量：byte 接收到的字节
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SensAR_RxHandler(uint8_t byte)
{
    g_SensAR.RxData[g_SensAR.RxDataCnt++] = byte;
    if (g_SensAR.RxDataCnt >= SENSAR_RX_SIZE) {
        g_SensAR.RxDataCnt = 0;
    }
}

/****************************************************************************************
* 函数名称：SensAR_ParseHallValue
* 函数功能：解析接收数据中的 Hall 值
* 输入参量：无
* 输出参量：Hall 值
* 编写日期：2026-2-24
****************************************************************************************/
static uint16_t SensAR_ParseHallValue(void)
{
    // 从响应中解析数字 (简化版)
    // 格式: "gethallvalue N<CRC>-->"
    // 查找数字部分
    char *p = (char*)g_SensAR.RxData;
    while (*p && (*p < '0' || *p > '9')) p++;
    return (uint16_t)atoi(p);
}

/****************************************************************************************
* 函数名称：SensAR_RxComplete
* 函数功能：接收完成处理，解析响应帧并更新 SensAR 数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SensAR_RxComplete(void)
{
    if (g_SensAR.RxDataCnt < 3) return;
    
    // 检查结束标志 "-->"
    if (g_SensAR.RxData[g_SensAR.RxDataCnt-3] != '-' ||
        g_SensAR.RxData[g_SensAR.RxDataCnt-2] != '-' ||
        g_SensAR.RxData[g_SensAR.RxDataCnt-1] != '>') {
        return;  // 未完成
    }
    
    // 根据当前标志处理
    if (g_SensAR.Flag.bit.Enter) {
        g_SensAR.Flag.all = 0;
        g_SensAR.CycleFlag = 1;
    } else if (g_SensAR.Flag.bit.getprdinfo) {
        // 解析产品信息
        memcpy(g_SensAR.PrdInfo, g_SensAR.RxData, g_SensAR.RxDataCnt > 63 ? 63 : g_SensAR.RxDataCnt);
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.gethallvalue1) {
        g_SensAR.Hall_1 = SensAR_ParseHallValue();
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.gethallvalue2) {
        g_SensAR.Hall_2 = SensAR_ParseHallValue();
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.gethallvalue3) {
        g_SensAR.Hall_3 = SensAR_ParseHallValue();
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.gethallvalue4) {
        g_SensAR.Hall_4 = SensAR_ParseHallValue();
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.gethallvalue5) {
        g_SensAR.Hall_5 = SensAR_ParseHallValue();
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.gethallvalue6) {
        g_SensAR.Hall_6 = SensAR_ParseHallValue();
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.gethallvalue7) {
        g_SensAR.Hall_7 = SensAR_ParseHallValue();
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.setprdinfo) {
        g_SensAR.Flag.all = 0;
    } else if (g_SensAR.Flag.bit.ver) {
        memcpy(g_SensAR.Version, g_SensAR.RxData, g_SensAR.RxDataCnt > 31 ? 31 : g_SensAR.RxDataCnt);
        g_SensAR.Flag.all = 0;
    }
    
    g_SensAR.RxDataCnt = 0;
}

/****************************************************************************************
* 函数名称：SensAR_Test
* 函数功能：测试函数主入口，根据测试项目执行对应操作
* 输入参量：testItem 测试项目编号
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SensAR_Test(uint8_t testItem)
{
    if (g_SensAR.Flag.bit.Enter) {
        g_SensAR.CycleFlag = 0;
        SensAR_TX("\r");
        return;
    }
    
    switch (testItem) {
        case SensAR_Test_Stop:
            break;
            
        case SensAR_Test_Enter:
            g_SensAR.Flag.bit.Enter = 1;
            SensAR_TX("\r");
            break;
            
        case SensAR_Test_GetPrdInfo:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.getprdinfo = 1;
            SensAR_TX("getprdinfo");
            break;
            
        case SensAR_Test_GetHallValue1:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.gethallvalue1 = 1;
            SensAR_TX("gethallvalue 1");
            break;
            
        case SensAR_Test_GetHallValue2:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.gethallvalue2 = 1;
            SensAR_TX("gethallvalue 2");
            break;
            
        case SensAR_Test_GetHallValue3:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.gethallvalue3 = 1;
            SensAR_TX("gethallvalue 3");
            break;
            
        case SensAR_Test_GetHallValue4:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.gethallvalue4 = 1;
            SensAR_TX("gethallvalue 4");
            break;
            
        case SensAR_Test_GetHallValue5:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.gethallvalue5 = 1;
            SensAR_TX("gethallvalue 5");
            break;
            
        case SensAR_Test_GetHallValue6:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.gethallvalue6 = 1;
            SensAR_TX("gethallvalue 6");
            break;
            
        case SensAR_Test_GetHallValue7:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.gethallvalue7 = 1;
            SensAR_TX("gethallvalue 7");
            break;
            
        case SensAR_Test_SetPrdInfo:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.setprdinfo = 1;
            // setprdinfo 需要组合参数，这里简化
            SensAR_TX("setprdinfo");
            break;
            
        case SensAR_Test_Version:
            g_SensAR.TestContent = SensAR_Test_Stop;
            g_SensAR.Flag.bit.ver = 1;
            SensAR_TX("ver");
            break;
            
        default:
            break;
    }
}

/****************************************************************************************
* 函数名称：SensAR_Modbus_IsMyAddr
* 函数功能：判断 Modbus 地址是否属于 SensAR 模块 (0x0700-0x07FF)
* 输入参量：addr Modbus 寄存器地址
* 输出参量：1=属于本模块，0=不属于
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t SensAR_Modbus_IsMyAddr(uint16_t addr)
{
    return (addr >= 0x0700 && addr <= 0x07FF);
}

/****************************************************************************************
* 函数名称：SensAR_Modbus_Read
* 函数功能：Modbus 03H 读取处理
* 输入参量：addr Modbus 寄存器地址
* 输出参量：对应地址的数据值
* 编写日期：2026-2-24
****************************************************************************************/
uint16_t SensAR_Modbus_Read(uint16_t addr)
{
    switch (addr) {
        case 0x0700: return g_SensAR.Hall_1;
        case 0x0701: return g_SensAR.Hall_2;
        case 0x0702: return g_SensAR.Hall_3;
        case 0x0703: return g_SensAR.Hall_4;
        case 0x0704: return g_SensAR.Hall_5;
        case 0x0705: return g_SensAR.Hall_6;
        case 0x0706: return g_SensAR.Hall_7;
        case 0x0707: return g_SensAR.CycleFlag;
        default: return 0xFFFF;
    }
}

/****************************************************************************************
* 函数名称：SensAR_Modbus_Write
* 函数功能：Modbus 06H 写入处理
* 输入参量：addr Modbus 寄存器地址，value 写入值
* 输出参量：0=成功，非0=错误
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t SensAR_Modbus_Write(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case 0x0700:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_Enter;
            return 0;
        case 0x0701:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_GetPrdInfo;
            return 0;
        case 0x0702:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_GetHallValue1;
            return 0;
        case 0x0703:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_GetHallValue2;
            return 0;
        case 0x0704:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_GetHallValue3;
            return 0;
        case 0x0705:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_GetHallValue4;
            return 0;
        case 0x0706:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_GetHallValue5;
            return 0;
        case 0x0707:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_GetHallValue6;
            return 0;
        case 0x0708:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_GetHallValue7;
            return 0;
        case 0x0709:
            if (value == 1) g_SensAR.TestContent = SensAR_Test_Version;
            return 0;
        default:
            return 1;
    }
}
