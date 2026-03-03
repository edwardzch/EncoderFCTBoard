/****************************************************************************************
 * @file      Encoder_MGTMag.c
 * @brief     MGT 磁编码器协议框架 (移植自 MGTMagneticEncoder.c, 33KB)
 ****************************************************************************************/
#include "Encoder_MGTMag.h"
#include "encoder_driver.h"
#include "modbus_function.h"
#include <string.h>

// ================= 全局变量 =================
MGT_t g_MGT = {0};

extern volatile uint8_t Work_Alarm;

/****************************************************************************************
* 函数名称：MGT_Init
* 函数功能：初始化 MGT 磁编码器模块，清零状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MGT_Init(void)
{
    memset(&g_MGT, 0, sizeof(g_MGT));
}

/****************************************************************************************
* 函数名称：MGT_TX
* 函数功能：发送 MGT 编码器命令，根据 cmd 类型组帧并启动 DMA 发送
* 输入参量：cmd 命令码，addr 地址，data 数据
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MGT_TX(uint8_t cmd, uint16_t addr, uint8_t data)
{
    g_MGT.TimeoutCnt++;
    g_MGT.TxID = cmd;
    g_MGT.TxData[0] = cmd;
    
    switch (cmd) {
        case 0x1A:  // 读取全部数据
            EncDrv_SendDMA(g_MGT.TxData, 1, 11);
            break;
        case 0x62:  // 多圈复位
            EncDrv_SendDMA(g_MGT.TxData, 1, 6);
            break;
            
        case 0x6D:  // 初始化命令 (ced/OIP/CIP)

            if (data == 1) {  // ced
                g_MGT.TxData[1] = 0x63;
                g_MGT.TxData[2] = 0x65;
                g_MGT.TxData[3] = 0x64;
            } else if (data == 2) {  // CIP
                g_MGT.TxData[1] = 0x43;
                g_MGT.TxData[2] = 0x49;
                g_MGT.TxData[3] = 0x50;
            } else {  // OIP
                g_MGT.TxData[1] = 0x4F;
                g_MGT.TxData[2] = 0x49;
                g_MGT.TxData[3] = 0x50;
            }
            g_MGT.TxData[4] = Enc_CRC8(g_MGT.TxData, 4);
            EncDrv_SendDMA(g_MGT.TxData, 5, 6);
            break;
            
        default:
            break;
    }
    
    if (g_MGT.TimeoutCnt > 5) {
        Work_Alarm = 0x01;
        g_MGT.Error.bit.DC = 1;
    }
}

/****************************************************************************************
* 函数名称：MGT_RxHandler
* 函数功能：接收处理（逐字节），存储接收数据
* 输入参量：byte 接收到的字节
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MGT_RxHandler(uint8_t byte)
{
    g_MGT.RxData[g_MGT.RxDataCnt++] = byte;
    if (g_MGT.RxDataCnt >= MGT_RX_SIZE) {
        g_MGT.RxDataCnt = 0;
    }
}

/****************************************************************************************
* 函数名称：MGT_RxComplete
* 函数功能：接收完成处理，解析响应帧并更新编码器数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MGT_RxComplete(void)
{
    if (g_MGT.RxDataCnt < 1) return;
    
    uint8_t cmd = g_MGT.RxData[0];
    uint8_t crc_ok = 0;
    
    switch (cmd) {
        case 0x1A:
            if (g_MGT.RxDataCnt >= 11) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 10);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[10]);
                if (!crc_ok) {
                    g_MGT.Error.bit.EC = 1;
                    Work_Alarm = 0x02;
                    g_MGT.RxDataCnt = 0;
                    return;
                }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxID = cmd;
                g_MGT.Status = g_MGT.RxData[1];
                g_MGT.SingleTurnPosition = ((uint32_t)g_MGT.RxData[4] << 16) | 
                                           ((uint32_t)g_MGT.RxData[3] << 8) | 
                                           g_MGT.RxData[2];
                g_MGT.ResolutionID = g_MGT.RxData[5];
                g_MGT.MultiTurnPosition = ((uint16_t)g_MGT.RxData[7] << 8) | g_MGT.RxData[6];
                g_MGT.Alarm = g_MGT.RxData[9];
                g_MGT.Error.bit.EC = 0;
            }
            break;
            
        case 0x62:
            if (g_MGT.RxDataCnt >= 6) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 5);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[5]);
                if (!crc_ok) {
                    g_MGT.Error.bit.EC = 1;
                    Work_Alarm = 0x02;
                    g_MGT.RxDataCnt = 0;
                    return;
                }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxID = cmd;
                g_MGT.Error.bit.EC = 0;
            }
            break;
            
        case 0x6D:
            if (g_MGT.RxDataCnt >= 6) {
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxID = cmd;
                g_MGT.Status0x6D = g_MGT.RxData[4];
            }
            break;
            
        case 0xEA:
            if (g_MGT.RxDataCnt == 4) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 3);
                if (g_MGT.CrcData != g_MGT.RxData[3]) {
                    g_MGT.Error.bit.EC = 1; 
                    Work_Alarm = 0x02;
                    g_MGT.RxDataCnt = 0;
                    return;
                }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxID = cmd;
                
                g_MGT.EepromParams.ReturnAddress = g_MGT.RxData[1] & 0x7F;
                if (g_MGT.EepromParams.ReturnAddress == 0x7F) {
                    g_MGT.EepromParams.ReturnPage = g_MGT.RxData[2];
                    g_MGT.EepromData[g_MGT.EepromParams.ReturnPage][g_MGT.EepromParams.ReturnAddress] = g_MGT.RxData[2];
                } else if (g_MGT.EepromParams.ReturnAddress < 0x7F) {
                    g_MGT.EepromData[g_MGT.EepromParams.ReturnPage][g_MGT.EepromParams.ReturnAddress] = g_MGT.RxData[2];
                }
                g_MGT.Error.bit.EC = 0;
            }
            break;
            
        case 0x32:
            if (g_MGT.RxDataCnt == 4) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 3);
                if (g_MGT.CrcData != g_MGT.RxData[3]) {
                    g_MGT.Error.bit.EC = 1; 
                    Work_Alarm = 0x02;
                    g_MGT.RxDataCnt = 0;
                    return;
                }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxID = cmd;
                
                g_MGT.EepromParams.ReturnAddress = g_MGT.RxData[1] & 0x7F;
                if (g_MGT.EepromParams.ReturnAddress == 0x7F) {
                    g_MGT.EepromParams.ReturnPage = g_MGT.RxData[2];
                    g_MGT.EepromData[g_MGT.EepromParams.ReturnPage][g_MGT.EepromParams.ReturnAddress] = g_MGT.RxData[2];
                } else if (g_MGT.EepromParams.ReturnAddress < 0x7F) {
                    g_MGT.EepromData[g_MGT.EepromParams.ReturnPage][g_MGT.EepromParams.ReturnAddress] = g_MGT.RxData[2];
                }
                g_MGT.Error.bit.EC = 0;
            }
            break;
            
        case 0x40:
            if (g_MGT.RxDataCnt >= 3) {
                switch (g_MGT.RxData[1]) {
                    case 0x10:
                        if (g_MGT.RxDataCnt == 7) {
                            g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 6);
                            if (g_MGT.CrcData != g_MGT.RxData[6]) {
                                g_MGT.Error.bit.EC = 1;
                                Work_Alarm = 0x02;
                                g_MGT.RxDataCnt = 0;
                                return;
                            }
                            g_MGT.TimeoutCnt = 0;
                            g_MGT.Error.bit.DC = 0;
                            g_MGT.RxID = cmd;
                            // Dummy map Firmware parsing
                            g_MGT.Error.bit.EC = 0;
                        }
                        break;
                    case 0x45:
                        if (g_MGT.RxDataCnt == 4) {
                            g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 3);
                            if (g_MGT.CrcData != g_MGT.RxData[3]) {
                                g_MGT.Error.bit.EC = 1;
                                Work_Alarm = 0x02;
                                g_MGT.RxDataCnt = 0;
                                return;
                            }
                            g_MGT.TimeoutCnt = 0;
                            g_MGT.ResolutionID = g_MGT.RxData[2];
                            g_MGT.Error.bit.EC = 0;
                        }
                        break;
                    case 0x46:
                        if (g_MGT.RxDataCnt == 3) {
                            if (g_MGT.RxData[2] != 0xF0) {
                                g_MGT.Error.bit.EC = 1;
                                Work_Alarm = 0x02;
                            } else {
                                g_MGT.TimeoutCnt = 0;
                                g_MGT.Error.bit.EC = 0;
                            }
                            g_MGT.RxDataCnt = 0;
                            return;
                        }
                        break;
                }
            }
            break;
            
        default:
            break;
    }
    
    g_MGT.RxDataCnt = 0;
}

/****************************************************************************************
* 函数名称：MGT_Test
* 函数功能：测试函数主入口，根据测试项目执行对应操作
* 输入参量：testItem 测试项目编号
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MGT_Test(uint8_t testItem)
{
    switch (testItem) {
        case MGT_Test_Stop: break;
        
        case MGT_Test_MultiturnReset:
            if (g_MGT.CF62 < 100) {
                MGT_TX(0x62, 0x00, 0x00);
                g_MGT.CF62++;
            } else {
                g_MotorEncoder.TestItem = MGT_Test_Stop;
            }
            break;
            
        case MGT_Test_GetAllData:
            MGT_TX(0x1A, 0x00, 0x00);
            break;
            
        case MGT_Test_ReadAllEeprom:
            if (g_MGT.EepromPage < MGT_PAGE_NUMBER) {
                MGT_TX(0xAD, 0x00, 0x00);
                g_MGT.EepromPage++;
            } else {
                g_MotorEncoder.TestItem = MGT_Test_Stop;
            }
            break;
            
        case MGT_Test_Initialize:
            if (g_MGT.CFCED < 5) {
                MGT_TX(0x6D, 0x00, 1);  // ced
                g_MGT.CFCED++;
            } else {
                g_MotorEncoder.TestItem = MGT_Test_Stop;
            }
            break;
            
        case MGT_Test_OIP:
            if (g_MGT.CFOIP < 5) {
                MGT_TX(0x6D, 0x00, 3);  // OIP
                g_MGT.CFOIP++;
            } else {
                g_MotorEncoder.TestItem = MGT_Test_Stop;
            }
            break;
            
        case MGT_Test_CIP:
            MGT_TX(0x6D, 0x00, 2);  // CIP
            g_MotorEncoder.TestItem = MGT_Test_Stop;
            break;
            
        default:
            break;
    }
}

/****************************************************************************************
* 函数名称：MGT_Modbus_IsMyAddr
* 函数功能：判断 Modbus 地址是否属于 MGT 编码器模块 (0x0400-0x04FF)
* 输入参量：addr Modbus 寄存器地址
* 输出参量：1=属于本模块，0=不属于
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t MGT_Modbus_IsMyAddr(uint16_t addr)
{
    return (addr >= 0x0400 && addr <= 0x04FF);
}

/****************************************************************************************
* 函数名称：MGT_Modbus_Read
* 函数功能：Modbus 03H 读取处理
* 输入参量：addr Modbus 寄存器地址
* 输出参量：对应地址的数据值
* 编写日期：2026-2-24
****************************************************************************************/
uint16_t MGT_Modbus_Read(uint16_t addr)
{
    switch (addr) {
        case 0x0400: return g_MotorEncoder.TestItem;
        case 0x0401: return g_MotorEncoder.ActualSpeed;
        case 0x0402: return g_MGT.EepromData[0][0x6F];
        case 0x0403: return g_MGT.EepromData[0][0x70];
        case 0x0404: return g_MGT.FW;
        case 0x0405: return g_MGT.EepromData[g_MGT.EepromParams.ReturnPage][g_MGT.EepromParams.ReturnAddress];
        case 0x0406: return g_MGT.EepromParams.Data;
        case 0x0407: return (uint16_t)(g_MGT.Firmware & 0xFFFF);
        case 0x0408: return g_MGT.ResolutionID;
        default:
            Communication_Address_Error();
            return 0xFFFF;
    }
}

/****************************************************************************************
* 函数名称：MGT_Modbus_Write
* 函数功能：Modbus 06H 写入处理
* 输入参量：addr Modbus 寄存器地址，value 写入值
* 输出参量：0=成功，非0=错误
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t MGT_Modbus_Write(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case 0x0400:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_ReadAllEeprom;
            return 0;
        case 0x0401:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_Initialize;
            return 0;
        case 0x0402:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_Speed;
            return 0;
        case 0x0403:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_FirmwareVersion;
            return 0;
        case 0x0404:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_WriteAllEeprom;
            return 0;
        case 0x0405:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_GetAllData;
            return 0;
        case 0x0406:
            g_MGT.EepromParams.SetAddress = value >> 8;
            g_MGT.EepromParams.Data = value & 0xFF;
            g_MotorEncoder.TestItem = MGT_Test_Command0x32;
            return 0;
        case 0x0407:
            g_MGT.EepromParams.SetAddress = value >> 8;
            g_MotorEncoder.TestItem = MGT_Test_Command0xEA;
            return 0;
        case 0x0408:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_OIP;
            return 0;
        case 0x0409:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_CIP;
            return 0;
        case 0x040A:
            g_MGT.EepromParams.Address = value & 0xFF;
            return 0;
        case 0x040B:
            g_MGT.EepromParams.Data = value >> 8;
            g_MotorEncoder.TestItem = MGT_Test_Command0x7A;
            return 0;
        case 0x040C:
            g_MGT.EepromParams.Address = value & 0xFF;
            g_MotorEncoder.TestItem = MGT_Test_Command0xA2;
            return 0;
        case 0x040D:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_Firmware;
            return 0;
        case 0x040E:
            g_MotorEncoder.SetResolutionID = (uint8_t)value;
            g_MotorEncoder.TestItem = MGT_Test_SetResolution;
            return 0;
        case 0x040F:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_ReadResolution;
            return 0;
        case 0x0410:
            if (value == 1) {
                // g_MotorEncoder.HWRevAddr = 0x030D;
                // g_MGT.EepromParams.Status.bit.Write = 1;
                g_MotorEncoder.TestItem = MGT_Test_WriteHWRev;
            }
            return 0;
        case 0x0411:
            if (value == 1) {
                // g_MotorEncoder.HWRevAddr = 0x030D;
                g_MotorEncoder.TestItem = MGT_Test_ReadHWRev;
            }
            return 0;
        case 0x0412:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_Timeout;
            return 0;
        default:
            Communication_Address_Error();
            return 1;
    }
}
