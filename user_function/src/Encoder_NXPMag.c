/****************************************************************************************
 * @file      Encoder_NXPMag.c
 * @brief     NXP 磁编码器协议实现 (移植自 NXPMagneticEncoder.c)
 ****************************************************************************************/
#include "Encoder_NXPMag.h"
#include "Encoder_MultiturnMag.h"  // for MotorEncoder_t, Enc_CRC8
#include "encoder_driver.h"
#include <string.h>

// ================= 全局变量 =================
NXP_t g_NXP = {0};

extern volatile uint8_t Work_Alarm;

/****************************************************************************************
* 函数名称：NXP_Init
* 函数功能：初始化 NXP 磁编码器模块，清零状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void NXP_Init(void)
{
    memset(&g_NXP, 0, sizeof(g_NXP));
}

/****************************************************************************************
* 函数名称：NXP_TX
* 函数功能：发送 NXP 编码器命令，根据 cmd 类型组帧并启动 DMA 发送
* 输入参量：cmd 命令码，addr 地址，data 数据
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void NXP_TX(uint8_t cmd, uint16_t addr, uint8_t data)
{
    g_NXP.TimeoutCnt++;
    g_NXP.TxID = cmd;
    g_NXP.TxData[0] = cmd;
    
    switch (cmd) {
        case 0x1A:  // 读取全部数据
            EncDrv_SendDMA(g_NXP.TxData, 1, 11);
            break;
        case 0x62:  // 多圈复位
        case 0x02:  // 读取单圈
            EncDrv_SendDMA(g_NXP.TxData, 1, 6);
            break;
        case 0xDA:  // 初始化
            EncDrv_SendDMA(g_NXP.TxData, 1, 3);
            break;
            
        case 0xEA:  // 读取 EEPROM

            g_NXP.TxData[1] = (uint8_t)addr;
            g_NXP.TxData[2] = Enc_CRC8(g_NXP.TxData, 2);
            EncDrv_SendDMA(g_NXP.TxData, 3, 4);
            break;
            
        case 0x32:  // 写入 EEPROM

            g_NXP.TxData[1] = (uint8_t)addr;
            g_NXP.TxData[2] = data;
            g_NXP.TxData[3] = Enc_CRC8(g_NXP.TxData, 3);
            EncDrv_SendDMA(g_NXP.TxData, 4, 4);
            break;
            
        case 0xA2:  // 读取固件版本

            g_NXP.TxData[1] = (uint8_t)(addr & 0xFF);
            g_NXP.TxData[2] = (uint8_t)((addr >> 8) & 0xFF);
            g_NXP.TxData[3] = Enc_CRC8(g_NXP.TxData, 3);
            EncDrv_SendDMA(g_NXP.TxData, 4, 6);
            break;
            
        default:
            break;
    }
    
    if (g_NXP.TimeoutCnt > 5) {
        Work_Alarm = 0x01;
    }
}

/****************************************************************************************
* 函数名称：NXP_RxHandler
* 函数功能：接收处理（逐字节），存储接收数据
* 输入参量：byte 接收到的字节
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void NXP_RxHandler(uint8_t byte)
{
    g_NXP.RxData[g_NXP.RxDataCnt++] = byte;
    if (g_NXP.RxDataCnt >= NXP_RX_SIZE) {
        g_NXP.RxDataCnt = 0;
    }
}

/****************************************************************************************
* 函数名称：NXP_RxComplete
* 函数功能：接收完成处理，解析响应帧并更新编码器数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void NXP_RxComplete(void)
{
    if (g_NXP.RxDataCnt < 1) return;
    
    uint8_t cmd = g_NXP.RxData[0];
    uint8_t crc_ok = 0;
    
    switch (cmd) {
        case 0x1A:
            if (g_NXP.RxDataCnt >= 11) {
                g_NXP.XorCrcData = Enc_CRC8(g_NXP.RxData, 10);
                crc_ok = (g_NXP.XorCrcData == g_NXP.RxData[10]);
                if (!crc_ok) {
                    g_NXP.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_NXP.RxDataCnt = 0;
                    return;
                }
                g_NXP.TimeoutCnt = 0;
                g_NXP.RxID = cmd;
                g_NXP.Status.all = g_NXP.RxData[1];
                g_NXP.SingleTurnPosition = ((uint32_t)g_NXP.RxData[4] << 16) | 
                                           ((uint32_t)g_NXP.RxData[3] << 8) | 
                                           g_NXP.RxData[2];
                g_NXP.ResolutionID = g_NXP.RxData[5];
                g_NXP.MultiTurnPosition = ((uint16_t)g_NXP.RxData[7] << 8) | g_NXP.RxData[6];
                g_NXP.Error.all = g_NXP.RxData[9];
                g_NXP.XorCrcError = 0;
            }
            break;
            
        case 0x02:
        case 0x62:
            if (g_NXP.RxDataCnt >= 6) {
                g_NXP.XorCrcData = Enc_CRC8(g_NXP.RxData, 5);
                crc_ok = (g_NXP.XorCrcData == g_NXP.RxData[5]);
                if (!crc_ok) {
                    g_NXP.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_NXP.RxDataCnt = 0;
                    return;
                }
                g_NXP.TimeoutCnt = 0;
                g_NXP.RxID = cmd;
                g_NXP.Status.all = g_NXP.RxData[1];
                g_NXP.SingleTurnPosition = ((uint32_t)g_NXP.RxData[4] << 16) | 
                                           ((uint32_t)g_NXP.RxData[3] << 8) | 
                                           g_NXP.RxData[2];
                g_NXP.XorCrcError = 0;
            }
            break;
            
        case 0xDA:
            if (g_NXP.RxDataCnt >= 3) {
                g_NXP.XorCrcData = Enc_CRC8(g_NXP.RxData, 2);
                crc_ok = (g_NXP.XorCrcData == g_NXP.RxData[2]);
                if (!crc_ok) {
                    g_NXP.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_NXP.RxDataCnt = 0;
                    return;
                }
                g_NXP.TimeoutCnt = 0;
                g_NXP.RxID = cmd;
                g_NXP.Status.all = g_NXP.RxData[1];
                g_NXP.XorCrcError = 0;
            }
            break;
            
        case 0xEA:
            if (g_NXP.RxDataCnt >= 4) {
                g_NXP.TimeoutCnt = 0;
                g_NXP.RxID = cmd;
                g_NXP.Eeprom.ReturnAddress = g_NXP.RxData[1] & 0x7F;
                g_NXP.Eeprom.Busy = (g_NXP.RxData[1] >> 7) & 0x01;
                uint8_t page = g_NXP.Eeprom.SetPage;
                uint8_t addr_idx = g_NXP.Eeprom.ReturnAddress;
                if (page < NXP_PAGE_NUMBER && addr_idx < NXP_EEPROM_ADDR) {
                    g_NXP.Eeprom.DataBuffer[page][addr_idx] = g_NXP.RxData[2];
                }
                g_NXP.Eeprom.SetAddress++;
            }
            break;
            
        case 0xA2:
            if (g_NXP.RxDataCnt >= 6) {
                g_NXP.TimeoutCnt = 0;
                g_NXP.RxID = cmd;
                g_NXP.FWVersion = ((uint16_t)g_NXP.RxData[2] << 8) | g_NXP.RxData[1];
            }
            break;
            
        default:
            break;
    }
    
    g_NXP.RxDataCnt = 0;
}

/****************************************************************************************
* 函数名称：NXP_Test
* 函数功能：测试函数主入口，根据测试项目执行对应操作
* 输入参量：testItem 测试项目编号
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void NXP_Test(uint8_t testItem)
{
    switch (testItem) {
        case NXP_Test_Stop: break;
        case NXP_Test_MultiturnReset:
            // NXP 无多圈复位
            break;
        case NXP_Test_GetAllData:
            NXP_TX(0x1A, 0x00, 0x00);
            break;
        case NXP_Test_ReadAllEeprom:
            if (g_NXP.Eeprom.SetAddress < 127) {
                NXP_TX(0xEA, g_NXP.Eeprom.SetAddress, 0x00);
            } else {
                g_NXP.Eeprom.SetPage++;
                if (g_NXP.Eeprom.SetPage >= NXP_PAGE_NUMBER) {
                    g_MotorEncoder.TestItem = NXP_Test_Stop;
                } else {
                    g_NXP.Eeprom.SetAddress = 0;
                }
            }
            break;
        case NXP_Test_Initialize:
            if (g_NXP.Cnt.CFDA < 10) {
                NXP_TX(0xDA, 0x00, 0x00);
                g_NXP.Cnt.CFDA++;
            } else {
                g_MotorEncoder.TestItem = NXP_Test_Stop;
            }
            break;
        case NXP_Test_Speed:
            NXP_TX(0x1A, 0x00, 0x00);
            break;
        case NXP_Test_FirmwareVersion:
            NXP_TX(0xA2, 0x0080, 0x00);
            break;
        default: break;
    }
}

/****************************************************************************************
* 函数名称：NXP_Modbus_IsMyAddr
* 函数功能：判断 Modbus 地址是否属于 NXP 编码器模块 (0x0500-0x05FF)
* 输入参量：addr Modbus 寄存器地址
* 输出参量：1=属于本模块，0=不属于
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t NXP_Modbus_IsMyAddr(uint16_t addr)
{
    return (addr >= 0x0500 && addr <= 0x05FF);
}

/****************************************************************************************
* 函数名称：NXP_Modbus_Read
* 函数功能：Modbus 03H 读取处理
* 输入参量：addr Modbus 寄存器地址
* 输出参量：对应地址的数据值
* 编写日期：2026-2-24
****************************************************************************************/
uint16_t NXP_Modbus_Read(uint16_t addr)
{
    switch (addr) {
        case 0x0500: return g_NXP.Status.all;
        case 0x0501: return g_NXP.ResolutionID;
        case 0x0502: return (uint16_t)(g_NXP.SingleTurnPosition >> 16);
        case 0x0503: return (uint16_t)(g_NXP.SingleTurnPosition & 0xFFFF);
        case 0x0504: return g_NXP.MultiTurnPosition;
        case 0x0505: return g_NXP.Error.all;
        case 0x0506: return g_NXP.FWVersion;
        default: return 0xFFFF;
    }
}

/****************************************************************************************
* 函数名称：NXP_Modbus_Write
* 函数功能：Modbus 06H 写入处理
* 输入参量：addr Modbus 寄存器地址，value 写入值
* 输出参量：0=成功，非0=错误
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t NXP_Modbus_Write(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case 0x0500:
            if (value == 1) g_MotorEncoder.TestItem = NXP_Test_GetAllData;
            return 0;
        case 0x0501:
            if (value == 1) g_MotorEncoder.TestItem = NXP_Test_ReadAllEeprom;
            return 0;
        case 0x0502:
            if (value == 1) g_MotorEncoder.TestItem = NXP_Test_Initialize;
            return 0;
        case 0x0503:
            if (value == 1) g_MotorEncoder.TestItem = NXP_Test_Speed;
            return 0;
        case 0x0504:
            if (value == 1) g_MotorEncoder.TestItem = NXP_Test_FirmwareVersion;
            return 0;
        default:
            return 1;
    }
}
