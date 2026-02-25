/****************************************************************************************
 * @file      Encoder_Tamagawa.c
 * @brief     多摩川编码器协议实现 (移植自 TamagawaEncoder.c)
 * @author    Gemini (移植优化)
 * @date      2026-02-09
 ****************************************************************************************/
#include "Encoder_Tamagawa.h"
#include "Encoder_MultiturnMag.h"  // for MotorEncoder_t, Enc_CRC8
#include "encoder_driver.h"
#include <string.h>

// ================= 全局变量 =================
Tmgw_t g_Tmgw = {0};
Tmgw_MTP_t g_TmgwMTP = {0};

extern volatile uint8_t Work_Alarm;

/****************************************************************************************
* 函数名称：Tmgw_Init
* 函数功能：初始化多摩川编码器模块，清零状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_Init(void)
{
    memset(&g_Tmgw, 0, sizeof(g_Tmgw));
    memset(&g_TmgwMTP, 0, sizeof(g_TmgwMTP));
}

/****************************************************************************************
* 函数名称：Tmgw_TX
* 函数功能：发送多摩川编码器命令，根据 cmd 类型组帧并启动 DMA 发送
* 输入参量：cmd 命令码，addr 地址，data 数据
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_TX(uint8_t cmd, uint16_t addr, uint8_t data)
{
    g_Tmgw.TimeoutCnt++;
    g_Tmgw.TxID = cmd;
    g_Tmgw.TxData[0] = cmd;
    
    switch (cmd) {
        case 0x1A:  // 读取全部数据
            EncDrv_SendDMA(g_Tmgw.TxData, 1, 11);
            break;
        case 0x62:  // 多圈复位
        case 0xC2:  // 单圈复位
            EncDrv_SendDMA(g_Tmgw.TxData, 1, 6);
            break;
            
        case 0xEA:  // 读取 EEPROM

            g_Tmgw.TxData[1] = (uint8_t)addr;
            g_Tmgw.TxData[2] = Enc_CRC8(g_Tmgw.TxData, 2);
            EncDrv_SendDMA(g_Tmgw.TxData, 3, 4);
            break;
            
        case 0x32:  // 写入 EEPROM

            g_Tmgw.TxData[1] = (uint8_t)addr;
            g_Tmgw.TxData[2] = data;
            g_Tmgw.TxData[3] = Enc_CRC8(g_Tmgw.TxData, 3);
            EncDrv_SendDMA(g_Tmgw.TxData, 4, 4);
            break;
            
        default:
            break;
    }
    
    if (g_Tmgw.TimeoutCnt > 5) {
        Work_Alarm = 0x01;  // 超时报警
    }
}

/****************************************************************************************
* 函数名称：Tmgw_RxHandler
* 函数功能：接收处理（逐字节），存储接收数据
* 输入参量：byte 接收到的字节
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_RxHandler(uint8_t byte)
{
    g_Tmgw.RxData[g_Tmgw.RxDataCnt++] = byte;
    if (g_Tmgw.RxDataCnt >= TMGW_RX_SIZE) {
        g_Tmgw.RxDataCnt = 0;
    }
}

/****************************************************************************************
* 函数名称：Tmgw_RxComplete
* 函数功能：接收完成处理，解析响应帧并更新编码器数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_RxComplete(void)
{
    if (g_Tmgw.RxDataCnt < 1) return;
    
    uint8_t cmd = g_Tmgw.RxData[0];
    uint8_t crc_ok = 0;
    
    switch (cmd) {
        case 0x1A:
            if (g_Tmgw.RxDataCnt >= 11) {
                g_Tmgw.XorCrcData = Enc_CRC8(g_Tmgw.RxData, 10);
                crc_ok = (g_Tmgw.XorCrcData == g_Tmgw.RxData[10]);
                if (!crc_ok) {
                    g_Tmgw.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_Tmgw.RxDataCnt = 0;
                    return;
                }
                g_Tmgw.TimeoutCnt = 0;
                g_Tmgw.RxID = cmd;
                g_Tmgw.Status.all = g_Tmgw.RxData[1];
                g_Tmgw.SingleTurnPosition = ((uint32_t)g_Tmgw.RxData[4] << 16) | 
                                            ((uint32_t)g_Tmgw.RxData[3] << 8) | 
                                            g_Tmgw.RxData[2];
                g_Tmgw.ResolutionID = g_Tmgw.RxData[5];
                g_Tmgw.Resolution = 1 << g_Tmgw.ResolutionID;
                g_Tmgw.SinglePoleResolution = g_Tmgw.Resolution / 5;
                g_Tmgw.Phase = g_Tmgw.SingleTurnPosition % g_Tmgw.SinglePoleResolution;
                g_Tmgw.PhaseAngle = 360 - ((g_Tmgw.Phase * 360) / g_Tmgw.SinglePoleResolution);
                g_Tmgw.MultiTurnPosition = ((uint16_t)g_Tmgw.RxData[7] << 8) | g_Tmgw.RxData[6];
                g_Tmgw.Error.all = g_Tmgw.RxData[9];
                g_Tmgw.XorCrcError = 0;
                
                // 同步到共享结构
                g_MotorEncoder.ActualSpeed = 0;  // 多摩川无直接速度
            }
            break;
            
        case 0x02:
        case 0x62:
        case 0xC2:
            if (g_Tmgw.RxDataCnt >= 6) {
                g_Tmgw.XorCrcData = Enc_CRC8(g_Tmgw.RxData, 5);
                crc_ok = (g_Tmgw.XorCrcData == g_Tmgw.RxData[5]);
                if (!crc_ok) {
                    g_Tmgw.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_Tmgw.RxDataCnt = 0;
                    return;
                }
                g_Tmgw.TimeoutCnt = 0;
                g_Tmgw.RxID = cmd;
                g_Tmgw.Status.all = g_Tmgw.RxData[1];
                g_Tmgw.SingleTurnPosition = ((uint32_t)g_Tmgw.RxData[4] << 16) | 
                                            ((uint32_t)g_Tmgw.RxData[3] << 8) | 
                                            g_Tmgw.RxData[2];
                g_Tmgw.XorCrcError = 0;
            }
            break;
            
        case 0xEA:
            if (g_Tmgw.RxDataCnt >= 4) {
                g_Tmgw.TimeoutCnt = 0;
                g_Tmgw.RxID = cmd;
                g_Tmgw.Eeprom.ReturnAddress = g_Tmgw.RxData[1] & 0x7F;
                
                if (g_Tmgw.Eeprom.ReturnAddress == TMGW_PNS_ADDR) {
                    g_Tmgw.Eeprom.ReturnPage = g_Tmgw.RxData[2];
                } else {
                    uint8_t page = g_Tmgw.Eeprom.SetPage;
                    uint8_t addr_idx = g_Tmgw.Eeprom.ReturnAddress;
                    if (page < TMGW_PAGE_NUMBER && addr_idx < TMGW_EEPROM_ADDR) {
                        g_Tmgw.Eeprom.DataBuffer[page][addr_idx] = g_Tmgw.RxData[2];
                    }
                    g_Tmgw.Eeprom.SetAddress++;
                }
            }
            break;
            
        default:
            break;
    }
    
    g_Tmgw.RxDataCnt = 0;
}

/****************************************************************************************
* 函数名称：Tmgw_Test
* 函数功能：测试函数主入口，根据测试项目执行对应操作
* 输入参量：testItem 测试项目编号
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_Test(uint8_t testItem)
{
    switch (testItem) {
        case Tmgw_Test_Stop:              break;
        case Tmgw_Test_MultiturnReset:    Tmgw_MultiturnReset(); break;
        case Tmgw_Test_GetAllData:        Tmgw_GetAllData(); break;
        case Tmgw_Test_ReadMTP:           Tmgw_ReadMTP(); break;
        case Tmgw_Test_SingleTurnReset:   Tmgw_SingleTurnReset(); break;
        case Tmgw_Test_ReadMTPDefine:     Tmgw_ReadMTPDefine(); break;
        default: break;
    }
}

/****************************************************************************************
* 函数名称：Tmgw_MultiturnReset
* 函数功能：多圈复位操作
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_MultiturnReset(void)
{
    if (g_Tmgw.Cnt.CF62 < 10) {
        Tmgw_TX(0x62, 0x00, 0x00);
        g_Tmgw.Cnt.CF62++;
        if (g_Tmgw.Cnt.CF62 == 10) {
            g_MotorEncoder.TestItem = Tmgw_Test_GetAllData;
        }
    }
}

/****************************************************************************************
* 函数名称：Tmgw_SingleTurnReset
* 函数功能：单圈复位操作
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_SingleTurnReset(void)
{
    if (g_Tmgw.Cnt.CFC2 < 10) {
        Tmgw_TX(0xC2, 0x00, 0x00);
        g_Tmgw.Cnt.CFC2++;
        if (g_Tmgw.Cnt.CFC2 == 10) {
            g_MotorEncoder.TestItem = Tmgw_Test_GetAllData;
        }
    }
}

/****************************************************************************************
* 函数名称：Tmgw_GetAllData
* 函数功能：获取编码器全部数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_GetAllData(void)
{
    Tmgw_TX(0x1A, 0x00, 0x00);
}

/****************************************************************************************
* 函数名称：Tmgw_ReadMTP
* 函数功能：读取 MTP 数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_ReadMTP(void)
{
    if (g_Tmgw.Eeprom.SetPage < TMGW_PAGE_NUMBER) {
        if (g_Tmgw.Eeprom.SetAddress < 127) {
            Tmgw_TX(0xEA, g_Tmgw.Eeprom.SetAddress, 0x00);
        } else {
            g_Tmgw.Eeprom.SetPage++;
            Tmgw_TX(0x32, g_Tmgw.Eeprom.SetAddress, g_Tmgw.Eeprom.SetPage);
            g_Tmgw.Eeprom.SetAddress = 0;
        }
    } else {
        g_Tmgw.Eeprom.SetPage = 0;
        g_Tmgw.Eeprom.ReturnPage = 0;
        g_Tmgw.Eeprom.SetAddress = 0;
        g_Tmgw.Eeprom.ReturnAddress = 0;
        g_MotorEncoder.TestItem = Tmgw_Test_ReadMTPDefine;
    }
}

/****************************************************************************************
* 函数名称：Tmgw_ReadMTPDefine
* 函数功能：读取 MTP 定义数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Tmgw_ReadMTPDefine(void)
{
    // 从 EEPROM 数据解析 MTP 参数
    // 简化版本, 具体解析逻辑根据实际需要补充
    g_MotorEncoder.TestItem = Tmgw_Test_GetAllData;
}

/****************************************************************************************
* 函数名称：Tmgw_Modbus_IsMyAddr
* 函数功能：判断 Modbus 地址是否属于多摩川编码器模块 (0x0600-0x06FF)
* 输入参量：addr Modbus 寄存器地址
* 输出参量：1=属于本模块，0=不属于
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t Tmgw_Modbus_IsMyAddr(uint16_t addr)
{
    return (addr >= 0x0600 && addr <= 0x06FF);
}

/****************************************************************************************
* 函数名称：Tmgw_Modbus_Read
* 函数功能：Modbus 03H 读取处理
* 输入参量：addr Modbus 寄存器地址
* 输出参量：对应地址的数据值
* 编写日期：2026-2-24
****************************************************************************************/
uint16_t Tmgw_Modbus_Read(uint16_t addr)
{
    switch (addr) {
        case 0x0600: return g_Tmgw.Status.all;
        case 0x0601: return g_Tmgw.ResolutionID;
        case 0x0602: return (uint16_t)(g_Tmgw.SingleTurnPosition >> 16);
        case 0x0603: return (uint16_t)(g_Tmgw.SingleTurnPosition & 0xFFFF);
        case 0x0604: return g_Tmgw.MultiTurnPosition;
        case 0x0605: return g_Tmgw.Error.all;
        case 0x0606: return g_Tmgw.PhaseAngle;
        case 0x0607: return (uint16_t)(g_Tmgw.Resolution >> 16);
        case 0x0608: return (uint16_t)(g_Tmgw.Resolution & 0xFFFF);
        case 0x0609: return g_TmgwMTP.RatedSpeed;
        case 0x060A: return g_TmgwMTP.MaximumSpeed;
        case 0x060B: return g_TmgwMTP.NumberOfPolePairs;
        default: return 0xFFFF;
    }
}

/****************************************************************************************
* 函数名称：Tmgw_Modbus_Write
* 函数功能：Modbus 06H 写入处理
* 输入参量：addr Modbus 寄存器地址，value 写入值
* 输出参量：0=成功，非0=错误
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t Tmgw_Modbus_Write(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case 0x0600:  // 多圈复位
            if (value == 1) {
                g_MotorEncoder.TestItem = Tmgw_Test_MultiturnReset;
            }
            return 0;
            
        case 0x0601:  // 单圈复位
            if (value == 1) {
                g_MotorEncoder.TestItem = Tmgw_Test_SingleTurnReset;
            }
            return 0;
            
        case 0x0602:  // 获取全部数据
            if (value == 1) {
                g_MotorEncoder.TestItem = Tmgw_Test_GetAllData;
            }
            return 0;
            
        case 0x0603:  // 读取 MTP
            if (value == 1) {
                g_MotorEncoder.TestItem = Tmgw_Test_ReadMTP;
            }
            return 0;
            
        default:
            return 1;
    }
}
