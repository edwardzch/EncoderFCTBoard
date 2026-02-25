/****************************************************************************************
 * @file      Encoder_MultiturnOpt.c
 * @brief     多圈光编码器协议框架 (移植自 MultiturnOpticalEncoder.c)
 * @note      原文件 1530 行，包含 35 个函数。这是骨架实现，具体逻辑需要补充。
 ****************************************************************************************/
#include "Encoder_MultiturnOpt.h"
#include "Encoder_MultiturnMag.h"  // for MotorEncoder_t, Enc_CRC8
#include "encoder_driver.h"
#include <string.h>

// ================= 全局变量 =================
MulOpt_t g_MulOpt = {0};

extern volatile uint8_t Work_Alarm;

/****************************************************************************************
* 函数名称：MulOpt_Init
* 函数功能：初始化多圈光编码器模块，清零状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulOpt_Init(void)
{
    memset(&g_MulOpt, 0, sizeof(g_MulOpt));
}

/****************************************************************************************
* 函数名称：MulOpt_TX
* 函数功能：发送多圈光编码器命令，根据 cmd 类型组帧并启动 DMA 发送
* 输入参量：cmd 命令码，addr 地址，data 数据
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulOpt_TX(uint8_t cmd, uint8_t addr, uint8_t data)
{
    g_MulOpt.TimeoutCnt++;
    g_MulOpt.TxID = cmd;
    g_MulOpt.TxData[0] = cmd;
    
    switch (cmd) {
        case 0x1A:  // 读取全部数据
            EncDrv_SendDMA(g_MulOpt.TxData, 1, 11);
            break;
        case 0x62:  // 多圈复位
        case 0x25:  // Hall 读取
        case 0x3D:  // 电池/温度
            EncDrv_SendDMA(g_MulOpt.TxData, 1, 8);
            break;
            
        case 0x6D:  // 初始化命令

            g_MulOpt.TxData[1] = 0x63;  // 'c'
            g_MulOpt.TxData[2] = 0x65;  // 'e'
            g_MulOpt.TxData[3] = 0x64;  // 'd'
            g_MulOpt.TxData[4] = Enc_CRC8(g_MulOpt.TxData, 4);
            EncDrv_SendDMA(g_MulOpt.TxData, 5, 6);
            break;
            
        case 0xAD:  // 批量读 EEPROM

            g_MulOpt.TxData[1] = g_MulOpt.EepromPage;
            g_MulOpt.TxData[2] = 0x00;
            g_MulOpt.TxData[3] = 0x80;
            g_MulOpt.TxData[4] = Enc_CRC8(g_MulOpt.TxData, 4);
            EncDrv_SendDMA(g_MulOpt.TxData, 5, 0x80 + 5);
            break;
            
        case 0x35:  // 批量写 EEPROM

            g_MulOpt.TxData[1] = g_MulOpt.EepromPage;
            g_MulOpt.TxData[2] = addr;
            g_MulOpt.TxData[3] = 1;
            g_MulOpt.TxData[4] = data;
            g_MulOpt.TxData[5] = Enc_CRC8(g_MulOpt.TxData, 5);
            EncDrv_SendDMA(g_MulOpt.TxData, 6, 4);
            break;
            
        default:
            break;
    }
    
    if (g_MulOpt.TimeoutCnt > 5) {
        Work_Alarm = 0x01;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_RxHandler
* 函数功能：接收处理（逐字节），存储接收数据
* 输入参量：byte 接收到的字节
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulOpt_RxHandler(uint8_t byte)
{
    g_MulOpt.RxData[g_MulOpt.RxDataCnt++] = byte;
    if (g_MulOpt.RxDataCnt >= MULOPT_RX_SIZE) {
        g_MulOpt.RxDataCnt = 0;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_RxComplete
* 函数功能：接收完成处理，解析响应帧并更新编码器数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulOpt_RxComplete(void)
{
    if (g_MulOpt.RxDataCnt < 1) return;
    
    uint8_t cmd = g_MulOpt.RxData[0];
    uint8_t crc_ok = 0;
    
    switch (cmd) {
        case 0x1A:
            if (g_MulOpt.RxDataCnt >= 11) {
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, 10);
                crc_ok = (g_MulOpt.CrcData == g_MulOpt.RxData[10]);
                if (!crc_ok) {
                    g_MulOpt.CrcError = 1;
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Status.all = g_MulOpt.RxData[1];
                g_MulOpt.SingleTurnPosition = ((uint32_t)g_MulOpt.RxData[4] << 16) | 
                                              ((uint32_t)g_MulOpt.RxData[3] << 8) | 
                                              g_MulOpt.RxData[2];
                g_MulOpt.ResolutionID = g_MulOpt.RxData[5];
                g_MulOpt.MultiTurnPosition = ((uint16_t)g_MulOpt.RxData[7] << 8) | g_MulOpt.RxData[6];
                g_MulOpt.Error = g_MulOpt.RxData[9];
                g_MulOpt.CrcError = 0;
            }
            break;
            
        case 0x62:
            if (g_MulOpt.RxDataCnt >= 6) {
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, 5);
                crc_ok = (g_MulOpt.CrcData == g_MulOpt.RxData[5]);
                if (!crc_ok) {
                    g_MulOpt.CrcError = 1;
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.CrcError = 0;
            }
            break;
            
        case 0x25:
            if (g_MulOpt.RxDataCnt >= 8) {
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Hall = (g_MulOpt.RxData[5] >> 4) & 0x0F;
                g_MulOpt.MTAB = g_MulOpt.RxData[5] & 0x0F;
            }
            break;
            
        case 0x3D:
            if (g_MulOpt.RxDataCnt >= 6) {
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Battery = ((uint16_t)g_MulOpt.RxData[2] << 8) | g_MulOpt.RxData[1];
                g_MulOpt.Temperature = ((uint16_t)g_MulOpt.RxData[4] << 8) | g_MulOpt.RxData[3];
            }
            break;
            
        case 0x6D:
            if (g_MulOpt.RxDataCnt >= 6) {
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
            }
            break;
            
        default:
            break;
    }
    
    g_MulOpt.RxDataCnt = 0;
}

/****************************************************************************************
* 函数名称：MulOpt_Test
* 函数功能：测试函数主入口，根据测试项目执行对应操作
* 输入参量：testItem 测试项目编号
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulOpt_Test(uint8_t testItem)
{
    switch (testItem) {
        case MulOpt_Test_Stop: break;
        
        case MulOpt_Test_MultiturnReset:
            if (g_MulOpt.CF62 < 100) {
                MulOpt_TX(0x62, 0x00, 0x00);
                g_MulOpt.CF62++;
            } else {
                g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            }
            break;
            
        case MulOpt_Test_GetAllData:
            MulOpt_TX(0x1A, 0x00, 0x00);
            break;
            
        case MulOpt_Test_ReadAllEeprom:
            if (g_MulOpt.EepromPage < MULOPT_PAGE_NUMBER) {
                MulOpt_TX(0xAD, 0x00, 0x00);
                g_MulOpt.EepromPage++;
            } else {
                g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            }
            break;
            
        case MulOpt_Test_Initialize:
            if (g_MulOpt.CFced < 5) {
                MulOpt_TX(0x6D, 0x00, 0x00);
                g_MulOpt.CFced++;
            } else {
                g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            }
            break;
            
        case MulOpt_Test_Hall:
            MulOpt_TX(0x25, 0x00, 0x00);
            break;
            
        case MulOpt_Test_TB:
            MulOpt_TX(0x3D, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;
            
        default:
            break;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_Modbus_IsMyAddr
* 函数功能：判断 Modbus 地址是否属于多圈光编码器模块 (0x0300-0x03FF)
* 输入参量：addr Modbus 寄存器地址
* 输出参量：1=属于本模块，0=不属于
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t MulOpt_Modbus_IsMyAddr(uint16_t addr)
{
    return (addr >= 0x0300 && addr <= 0x03FF);
}

/****************************************************************************************
* 函数名称：MulOpt_Modbus_Read
* 函数功能：Modbus 03H 读取处理
* 输入参量：addr Modbus 寄存器地址
* 输出参量：对应地址的数据值
* 编写日期：2026-2-24
****************************************************************************************/
uint16_t MulOpt_Modbus_Read(uint16_t addr)
{
    switch (addr) {
        case 0x0300: return g_MulOpt.MTAB;
        case 0x0301: return g_MulOpt.Hall;
        case 0x0302: return g_MulOpt.Status.all;
        case 0x0303: return g_MulOpt.HallResult;
        case 0x0304: return g_MulOpt.Error;
        case 0x0305: return g_MulOpt.ResolutionID;
        case 0x0306: return g_MulOpt.EepromData[3][20];  // HW 版本
        case 0x0307: return g_MulOpt.EepromData[3][21];
        case 0x0308: return g_MulOpt.EepromData[3][22];
        case 0x0309: return g_MulOpt.EepromData[3][23];
        case 0x030A: return g_MulOpt.Battery;
        case 0x030B: return g_MulOpt.Temperature;
        case 0x0310: return g_MulOpt.MSinMax;
        case 0x0311: return g_MulOpt.MCosMax;
        case 0x0312: return g_MulOpt.NSinMax;
        case 0x0313: return g_MulOpt.NCosMax;
        case 0x0314: return g_MulOpt.SSinMax;
        case 0x0315: return g_MulOpt.SCosMax;
        case 0x0316: return g_MulOpt.MSin;
        case 0x0317: return g_MulOpt.MCos;
        case 0x0318: return g_MulOpt.NSin;
        case 0x0319: return g_MulOpt.NCos;
        case 0x031A: return g_MulOpt.SSin;
        case 0x031B: return g_MulOpt.SCos;
        case 0x031C: return g_MulOpt.LightIntensity;
        case 0x032C: return g_MulOpt.DACValue;
        default: return 0xFFFF;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_Modbus_Write
* 函数功能：Modbus 06H 写入处理
* 输入参量：addr Modbus 寄存器地址，value 写入值
* 输出参量：0=成功，非0=错误
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t MulOpt_Modbus_Write(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case 0x0300:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_ReadAllEeprom;
            return 0;
        case 0x0301:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_Initialize;
            return 0;
        case 0x0302:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_Hall;
            return 0;
        case 0x0303:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_WriteResult;
            return 0;
        case 0x0304:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_GetAllData;
            return 0;
        case 0x0305:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_MultiturnReset;
            return 0;
        case 0x0306:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_TB;
            return 0;
        case 0x0307:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_OIP;
            return 0;
        case 0x0308:
            if (value == 1) g_MotorEncoder.TestItem = MulOpt_Test_CIP;
            return 0;
        default:
            return 1;
    }
}
