/****************************************************************************************
 * @file      Encoder_MGTMag.c
 * @brief     单圈磁编码器协议实现
 * @author    
 * @date      2026-02-09
 * @note      用于测试单圈磁编
 ****************************************************************************************/
#include "Encoder_MGTMag.h"
#include "encoder_driver.h"
#include "modbus_function.h"
#include "encoder_modbus.h"
#include <string.h>
#include <stdio.h>

// ================= 全局变量 =================
MGT_t g_MGT = {0};


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
* 函数名称：MGT_CrcError
* 函数功能：编码器返回数据CRC计算错误处理
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-06
****************************************************************************************/
static void MGT_CrcError(void)
{
    g_MGT.Error.bit.EC = 1;
    Work_Alarm = WORK_ALARM_CRC_ERROR;
    g_MGT.CrcError = 1;
    g_MGT.RxDataCnt = 0;
    g_MGT.TimeoutCnt = 0;
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
    g_MGT.TxData[0] = cmd;

    switch (cmd) {
        case 0x62:  // 多圈复位 — 1字节帧
            EncDrv_SendDMA(g_MGT.TxData, 1, 6);
            break;

        case 0x1A:  // 读取全部数据 — 1字节帧
            EncDrv_SendDMA(g_MGT.TxData, 1, 11);
            break;

        case 0x02:  // 单圈位置读取 — 1字节帧
            EncDrv_SendDMA(g_MGT.TxData, 1, 6);
            break;

        case 0xDA:  // 初始化命令 — 1字节帧
            EncDrv_SendDMA(g_MGT.TxData, 1, 3);
            break;

        case 0xEA:  // EEPROM 8位地址读 — 3字节帧 [cmd, addr, CRC]
            g_MGT.TxData[1] = (uint8_t)(addr & 0xFF);
            g_MGT.TxData[2] = Enc_CRC8(g_MGT.TxData, 2);
            EncDrv_SendDMA(g_MGT.TxData, 3, 4);
            break;

        case 0x32:  // EEPROM 8位地址写 — 4字节帧 [cmd, addr, data, CRC]
            g_MGT.TxData[1] = (uint8_t)(addr & 0xFF);
            g_MGT.TxData[2] = data;
            g_MGT.TxData[3] = Enc_CRC8(g_MGT.TxData, 3);
            EncDrv_SendDMA(g_MGT.TxData, 4, 4);
            break;

        case 0xA2:  // EEPROM 16位地址读 — 4字节帧 [cmd, addr_lo, addr_hi, CRC]
            g_MGT.TxData[1] = (uint8_t)(addr & 0xFF);
            g_MGT.TxData[2] = (uint8_t)((addr >> 8) & 0xFF);
            g_MGT.TxData[3] = Enc_CRC8(g_MGT.TxData, 3);
            EncDrv_SendDMA(g_MGT.TxData, 4, 5);
            break;

        case 0x7A:  // EEPROM 16位地址写 — 5字节帧 [cmd, addr_lo, addr_hi, data, CRC]
            g_MGT.TxData[1] = (uint8_t)(addr & 0xFF);
            g_MGT.TxData[2] = (uint8_t)((addr >> 8) & 0xFF);
            g_MGT.TxData[3] = data;
            g_MGT.TxData[4] = Enc_CRC8(g_MGT.TxData, 4);
            EncDrv_SendDMA(g_MGT.TxData, 5, 5);
            break;

        case 0x6D:  // OIP/CIP 命令 — 5字节帧 [cmd, data, 0x49, 0x50, CRC]
            g_MGT.TxData[1] = data;
            g_MGT.TxData[2] = 0x49;
            g_MGT.TxData[3] = 0x50;
            g_MGT.TxData[4] = Enc_CRC8(g_MGT.TxData, 4);
            EncDrv_SendDMA(g_MGT.TxData, 5, 6);
            break;

        case 0x40:  // 读/设置分辨率 & 读固件
            g_MGT.TxData[1] = (uint8_t)(addr & 0xFF);
            if (g_MGT.TxData[1] == 0x46) {
                // 设置分辨率: 4字节帧 [cmd, 0x46, data, CRC]
                g_MGT.TxData[2] = data;
                g_MGT.TxData[3] = Enc_CRC8(g_MGT.TxData, 3);
                EncDrv_SendDMA(g_MGT.TxData, 4, 4);
            } else if(g_MGT.TxData[1] == 0x45) {
                // 读分辨率/固件: 3字节帧 [cmd, sub, CRC]
                g_MGT.TxData[2] = Enc_CRC8(g_MGT.TxData, 2);
                EncDrv_SendDMA(g_MGT.TxData, 3, 4);
            } else if(g_MGT.TxData[1] == 0x10) {
                // 读分辨率/固件: 3字节帧 [cmd, sub, CRC]
                g_MGT.TxData[2] = Enc_CRC8(g_MGT.TxData, 2);
                EncDrv_SendDMA(g_MGT.TxData, 3, 7);							
						}
            break;

        default:
            break;
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
        // ====== 0x1A: 读取全部数据 — 11字节响应 ======
        case 0x1A:
            if (g_MGT.RxDataCnt >= 11) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 10);
                if (g_MGT.CrcData != g_MGT.RxData[10]) { 
									MGT_CrcError(); 
									return; 
								}
                g_MGT.TimeoutCnt = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.Status.all = g_MGT.RxData[1];
                g_MGT.SingleTurnPosition = ((uint32_t)g_MGT.RxData[4] << 16) |
                                           ((uint32_t)g_MGT.RxData[3] << 8) |
                                           g_MGT.RxData[2];
                g_MGT.ResolutionID = g_MGT.RxData[5];
                g_MGT.MultiTurnPosition = ((uint16_t)g_MGT.RxData[7] << 8) | g_MGT.RxData[6];
                g_MGT.Alarm.all = g_MGT.RxData[9];
            }
            return;

        // ====== 0x02: 单圈位置读取 — 6字节响应 ======
        case 0x02:
            if (g_MGT.RxDataCnt >= 6) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 5);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[5]);
                if (!crc_ok) { MGT_CrcError(); return; }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.Status.all = g_MGT.RxData[1];
                g_MGT.SingleTurnPosition = ((uint32_t)g_MGT.RxData[4] << 16) |
                                           ((uint32_t)g_MGT.RxData[3] << 8) |
                                           g_MGT.RxData[2];
            }
            return;

        // ====== 0xDA: 初始化 — 3字节响应 ======
        case 0xDA:
            if (g_MGT.RxDataCnt >= 3) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 2);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[2]);
                if (!crc_ok) { MGT_CrcError(); return; }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.Status.all = g_MGT.RxData[1];
            }
            return;

        // ====== 0x62: 多圈复位 — 6字节响应 ======
        case 0x62:
            if (g_MGT.RxDataCnt >= 6) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 5);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[5]);
                if (!crc_ok) { MGT_CrcError(); return; }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.Status.all = g_MGT.RxData[1];
                g_MGT.SingleTurnPosition = ((uint32_t)g_MGT.RxData[4] << 16) |
                                           ((uint32_t)g_MGT.RxData[3] << 8) |
                                           g_MGT.RxData[2];
            }
            return;

        // ====== 0xEA: EEPROM 8位地址读 — 4字节响应 ======
        case 0xEA:
            if (g_MGT.RxDataCnt >= 4) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 3);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[3]);
                if (!crc_ok) { MGT_CrcError(); return; }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.Eeprom.ReturnAddress = g_MGT.RxData[1] & 0x7F;
                if (g_MGT.Eeprom.ReturnAddress == 0x7F) {
                    g_MGT.Eeprom.ReturnPage = g_MGT.RxData[2];
                    g_MGT.Eeprom.DataBuffer[g_MGT.Eeprom.ReturnPage][g_MGT.Eeprom.ReturnAddress] = g_MGT.RxData[2];
                } else if (g_MGT.Eeprom.ReturnAddress < 0x7F) {
                    g_MGT.Eeprom.DataBuffer[g_MGT.Eeprom.ReturnPage][g_MGT.Eeprom.ReturnAddress] = g_MGT.RxData[2];
                }
                g_MGT.Error.bit.EC = 0;
            }
            return;

        // ====== 0x32: EEPROM 8位地址写 — 4字节响应 ======
        case 0x32:
            if (g_MGT.RxDataCnt >= 4) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 3);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[3]);
                if (!crc_ok) { MGT_CrcError(); return; }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.Eeprom.ReturnAddress = g_MGT.RxData[1] & 0x7F;
                if (g_MGT.Eeprom.ReturnAddress == 0x7F) {
                    g_MGT.Eeprom.ReturnPage = g_MGT.RxData[2];
                    g_MGT.Eeprom.DataBuffer[g_MGT.Eeprom.ReturnPage][g_MGT.Eeprom.ReturnAddress] = g_MGT.RxData[2];
                } else if (g_MGT.Eeprom.ReturnAddress < 0x7F) {
                    g_MGT.Eeprom.DataBuffer[g_MGT.Eeprom.ReturnPage][g_MGT.Eeprom.ReturnAddress] = g_MGT.RxData[2];
                }
                g_MGT.Error.bit.EC = 0;
            }
            return;

        // ====== 0xA2: EEPROM 16位地址读 — 5字节响应 ======
        case 0xA2:
            if (g_MGT.RxDataCnt >= 5) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 4);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[4]);
                if (!crc_ok) { MGT_CrcError(); return; }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.Eeprom.Address = ((uint16_t)g_MGT.RxData[2] << 8 | g_MGT.RxData[1]) & 0x7FFF;
                g_MGT.Eeprom.Status.bit.Busy = (g_MGT.RxData[2] >> 7) & 0x01;
                if (g_MGT.Eeprom.Status.bit.Busy == 0) {
                    if (g_MGT.Eeprom.Address == 0x0305) {
                        g_MGT.FW = g_MGT.RxData[3];
                    }
                    g_MGT.Eeprom.Data = g_MGT.RxData[3];
                }
            }
            return;

        // ====== 0x7A: EEPROM 16位地址写 — 5字节响应 ======
        case 0x7A:
            if (g_MGT.RxDataCnt >= 5) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 4);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[4]);
                if (!crc_ok) { MGT_CrcError(); return; }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.Eeprom.Address = ((uint16_t)g_MGT.RxData[2] << 8 | g_MGT.RxData[1]) & 0x7FFF;
                g_MGT.Eeprom.Status.bit.Busy = (g_MGT.RxData[2] >> 7) & 0x01;
            }
            return;

        // ====== 0x6D: OIP/CIP — 6字节响应 ======
        case 0x6D:
            if (g_MGT.RxDataCnt >= 6) {
                g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 5);
                crc_ok = (g_MGT.CrcData == g_MGT.RxData[5]);
                if (!crc_ok) { MGT_CrcError(); return; }
                g_MGT.TimeoutCnt = 0;
                g_MGT.Error.bit.DC = 0;
                g_MGT.RxDataCnt = 0;
                g_MGT.RxID = cmd;
                g_MGT.OIPStatus = g_MGT.RxData[4];
                if (g_MGT.Cnt.CFOIP == 5) {
                    g_MotorEncoder.TestItem = MGT_Test_Stop;
                    g_MGT.Cnt.CFOIP = 0;
                    if (g_MGT.OIPStatus == 0) {
                        Work_Alarm = 0x07;
                    }
                }
                if (g_MGT.OIPStatus == 4) {
                    Work_Alarm = 0x08;
                }
            }
            return;

        // ====== 0x40: 分辨率/固件 — 可变长度响应 ======
        case 0x40:
            if (g_MGT.RxDataCnt >= 3) {
                switch (g_MGT.RxData[1]) {
                    case 0x10:  // 固件信息 — 7字节响应
                        if (g_MGT.RxDataCnt >= 7) {
                            g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 6);
                            crc_ok = (g_MGT.CrcData == g_MGT.RxData[6]);
                            if (!crc_ok) { MGT_CrcError(); return; }
                            g_MGT.TimeoutCnt = 0;
                            g_MGT.RxDataCnt = 0;
                            g_MGT.Firmware = ((uint32_t)g_MGT.RxData[2] << 24) |
                                             ((uint32_t)g_MGT.RxData[3] << 16) |
                                             ((uint32_t)g_MGT.RxData[4] << 8) |
                                             g_MGT.RxData[5];
                        }
                        return;

                    case 0x45:  // 读分辨率 — 4字节响应
                        if (g_MGT.RxDataCnt >= 4) {
                            g_MGT.CrcData = Enc_CRC8(g_MGT.RxData, 3);
                            crc_ok = (g_MGT.CrcData == g_MGT.RxData[3]);
                            if (!crc_ok) { MGT_CrcError(); return; }
                            g_MGT.TimeoutCnt = 0;
                            g_MGT.RxDataCnt = 0;
                            g_MGT.ResolutionID = g_MGT.RxData[2];
                        }
                        return;

                    case 0x46:  // 设置分辨率确认 — 3字节响应
                        if (g_MGT.RxDataCnt >= 3) {
                            g_MGT.TimeoutCnt = 0;
                            g_MGT.RxDataCnt = 0;
                            if (g_MGT.RxData[2] != 0xF0) {
                                MGT_CrcError();
                            }
                        }
                        return;
                }
            }
            return;

        default:
            g_MGT.RxDataCnt = 0;
            return;
    }
}

// ======================================================================
//  辅助函数 (与原始函数逐行对应)
// ======================================================================

/****************************************************************************************
* 函数名称：MGT_ReadOnePage
* 函数功能：读取一页 EEPROM 数据（0~0x7F 逐地址读取，读完切页）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_ReadOnePage(void)
{
    if (g_MGT.Eeprom.SetAddress < 0x80) {
        MGT_TX(0xEA, g_MGT.Eeprom.SetAddress, 0x00);
        g_MGT.Eeprom.SetAddress++;
    } else {
        if (g_MGT.Eeprom.Status.bit.Write == 0) {
            g_MGT.Eeprom.SetPage++;
            if (g_MGT.Eeprom.SetPage < MGT_PAGE_NUMBER) {
                MGT_TX(0x32, MGT_PNS_ADDRESS, g_MGT.Eeprom.SetPage);
                g_MGT.Eeprom.Status.bit.Write = 1;
            } else {
                g_MotorEncoder.TestItem = MGT_Test_Stop;
            }
        } else {
            MGT_TX(0xEA, MGT_PNS_ADDRESS, 0x00);
            g_MGT.Eeprom.Status.bit.Write = 0;
            g_MGT.Eeprom.SetAddress = 0;
            g_MGT.Eeprom.ReturnAddress = 0;
        }
    }
}

/****************************************************************************************
* 函数名称：MGT_ReadAllEeprom
* 函数功能：读取所有页 EEPROM 数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_ReadAllEeprom(void)
{
    if (g_MGT.Eeprom.SetPage < MGT_PAGE_NUMBER) {
        if (g_MGT.Eeprom.Status.bit.Busy == 0) {
            MGT_ReadOnePage();
        } else {
            MGT_TX(0xEA, MGT_PNS_ADDRESS, 0x00);
        }
    }
}

/****************************************************************************************
* 函数名称：MGT_WriteOnePage
* 函数功能：写入一页 EEPROM 数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_WriteOnePage(void)
{
    if (g_MGT.Eeprom.ReturnAddress < 0x80) {
        if (g_MGT.Eeprom.Status.bit.Busy == 0) {
            if (g_MGT.Eeprom.Status.bit.Read) {
                MGT_TX(0xEA, g_MGT.Eeprom.ReturnAddress, 0x00);
                g_MGT.Eeprom.Status.bit.Read = 0;
            } else {
                if (g_MGT.Eeprom.ReturnAddress == 0x7E) {
                    g_MGT.Eeprom.SetPage++;
                    MGT_TX(0x32, MGT_PNS_ADDRESS, g_MGT.Eeprom.SetPage);
                    g_MGT.Eeprom.SetAddress = 0;
                    g_MGT.Eeprom.ReturnAddress = 0;
                    g_MGT.Eeprom.Status.bit.Read = 1;
                } else {
                    MGT_TX(0x32, g_MGT.Eeprom.SetAddress, (g_MGT.Eeprom.SetPage + 1));
                    g_MGT.Eeprom.Status.bit.Read = 1;
                    g_MGT.Eeprom.SetAddress++;
                }
            }
        } else {
            MGT_TX(0xEA, g_MGT.Eeprom.ReturnAddress, 0x00);
        }
    }
}

/****************************************************************************************
* 函数名称：MGT_WriteAllEeprom
* 函数功能：写入所有页 EEPROM 数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_WriteAllEeprom(void)
{
    if (g_MGT.Eeprom.SetPage < MGT_PAGE_NUMBER) {
        MGT_WriteOnePage();
    } else {
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MGT_Initialize
* 函数功能：初始化编码器（发 0xDA 命令）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_Initialize(void)
{
    if (g_MGT.Cnt.CFDA < 10) {
        MGT_TX(0xDA, 0x00, 0x00);
        g_MGT.Cnt.CFDA++;
    } else {
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MGT_Speed
* 函数功能：速度测试（连续发 0x1A 读取位置数据）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_Speed(void)
{
    MGT_TX(0x1A, 0x00, 0x00);
    if (g_MGT.Cnt.CF1A < 100) {
        g_MGT.Cnt.CF1A++;
    }
    // 原始: Motor.Flag.bit.ReadyTest = 1; → 新项目由 SpeedCalc_Update 实时处理
}

/****************************************************************************************
* 函数名称：MGT_OpenInternalProtocol
* 函数功能：打开内部协议（发 0x6D 'O' 命令）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_OpenInternalProtocol(void)
{
    if (g_MGT.Cnt.CFOIP < 5) {
        MGT_TX(0x6D, 0x00, 0x4F);  // 'O' = 0x4F
        g_MGT.Cnt.CFOIP++;
    } else {
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MGT_Command0x32
* 函数功能：EEPROM 8位地址单字节写入（等待 busy 后写入）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_Command0x32(void)
{
    if (g_MGT.Eeprom.Status.bit.Busy == 0) {
        MGT_TX(0x32, g_MGT.Eeprom.SetAddress, g_MGT.Eeprom.Data);
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    } else {
        MGT_TX(0xEA, g_MGT.Eeprom.SetAddress, 0x00);
    }
}

/****************************************************************************************
* 函数名称：MGT_Command0x7A
* 函数功能：EEPROM 16位地址单字节写入（等待 busy 后写入）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_Command0x7A(void)
{
    if (g_MGT.Eeprom.Status.bit.Busy == 0) {
        MGT_TX(0x7A, g_MGT.Eeprom.Address, g_MGT.Eeprom.Data);
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    } else {
        MGT_TX(0xA2, g_MGT.Eeprom.Address, 0x00);
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MGT_WriteDate
* 函数功能：写入测试日期到 EEPROM (0x0304 + TestDateCnt)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-06
****************************************************************************************/
static void MGT_WriteDate(void)
{
    if (g_MGT.Eeprom.Status.bit.Busy == 0) {
        MGT_TX(0x7A, (0x0304 + g_MotorEncoder.TestDateCnt), g_MotorEncoder.TestDate);
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    } else {
        MGT_TX(0xA2, (0x0304 + g_MotorEncoder.TestDateCnt), 0x00);
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MGT_SetResolution
* 函数功能：设置分辨率（发 0x40/0x46 命令）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_SetResolution(void)
{
    MGT_TX(0x40, 0x46, g_MotorEncoder.SetResolutionID);
    g_MotorEncoder.TestItem = MGT_Test_Stop;
}

/****************************************************************************************
* 函数名称：MGT_ReadResolution
* 函数功能：读取分辨率（发 0x40/0x45 命令）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_ReadResolution(void)
{
    MGT_TX(0x40, 0x45, 0x00);
    g_MotorEncoder.TestItem = MGT_Test_Stop;
}

/****************************************************************************************
* 函数名称：MGT_WriteHWRev
* 函数功能：写入硬件版本号到 EEPROM (0x030D-0x0310 状态机)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-06
****************************************************************************************/
static void MGT_WriteHWRev(void)
{
    if (g_MGT.Eeprom.Status.bit.Write == 1) {
        // 写入阶段
        switch (g_MotorEncoder.HWRevAddr) {
            case 0x030D:
                MGT_TX(0x7A, g_MotorEncoder.HWRevAddr, 'H');
                g_MGT.Eeprom.Status.bit.Write = 0;
                break;
            case 0x030E:
                MGT_TX(0x7A, g_MotorEncoder.HWRevAddr, 'M');
                g_MGT.Eeprom.Status.bit.Write = 0;
                break;
            case 0x030F:
                MGT_TX(0x7A, g_MotorEncoder.HWRevAddr, 1);
                g_MGT.Eeprom.Status.bit.Write = 0;
                break;
            case 0x0310:
                MGT_TX(0x7A, g_MotorEncoder.HWRevAddr, g_MotorEncoder.HWRevData);
                g_MGT.Eeprom.Status.bit.Write = 0;
                break;
            default:
                break;
        }
    } else {
        if (g_MGT.Eeprom.Status.bit.Read == 1) {
            // 读取确认阶段: busy=0 表示写入成功
            if (g_MGT.Eeprom.Status.bit.Busy == 0) {
                if (g_MotorEncoder.HWRevAddr == 0x0310) {
                    // 全部写完
                    g_MGT.Eeprom.Status.bit.Read = 0;
                    g_MotorEncoder.TestItem = MGT_Test_Stop;
                } else {
                    // 前进到下一地址
                    g_MGT.Eeprom.Status.bit.Write = 1;
                    g_MGT.Eeprom.Status.bit.Read = 0;
                    g_MotorEncoder.HWRevAddr++;
                }
            } else {
                // busy, 继续读取等待
                MGT_TX(0xA2, g_MotorEncoder.HWRevAddr, 0x00);
            }
        } else {
            // 初始状态: 发 0xA2 读取, 等 busy 位清零后再写
            MGT_TX(0xA2, g_MotorEncoder.HWRevAddr, 0x00);
            g_MGT.Eeprom.Status.bit.Read = 1;
        }
    }
}

/****************************************************************************************
* 函数名称：MGT_ReadHWRev
* 函数功能：读取硬件版本号 (0xA2 命令逐地址读取 0x030E-0x0311)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-06
****************************************************************************************/
static void MGT_ReadHWRev(void)
{
    // 按原始逻辑: 每次调用读取当前 HWRevAddr, 逐步递增
    if (g_MotorEncoder.HWRevAddr == 0x030E) {
        g_MotorEncoder.HWRevBuffer[0] = g_MGT.Eeprom.Data;
        g_MotorEncoder.HWRevBuffer[1] = '.';
    } else if (g_MotorEncoder.HWRevAddr == 0x030F) {
        g_MotorEncoder.HWRevBuffer[2] = g_MGT.Eeprom.Data;
        g_MotorEncoder.HWRevBuffer[3] = '.';
    } else if (g_MotorEncoder.HWRevAddr == 0x0310) {
        g_MotorEncoder.HWRevBuffer[4] = g_MGT.Eeprom.Data + 0x30;
        g_MotorEncoder.HWRevBuffer[5] = '.';
    } else if (g_MotorEncoder.HWRevAddr == 0x0311) {
        g_MotorEncoder.HWRevBuffer[6] = g_MGT.Eeprom.Data + 0x30;
        g_MotorEncoder.HWRevBuffer[7] = '\0';
    }

    if (g_MotorEncoder.HWRevAddr < 0x0311) {
        MGT_TX(0xA2, g_MotorEncoder.HWRevAddr, 0x00);
        g_MotorEncoder.HWRevAddr++;
    } else {
        g_MotorEncoder.HWRevAddr = 0;
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MGT_TimeOut
* 函数功能：超时测试（连续发 0x02 读取直到超时）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void MGT_TimeOut(void)
{
    static uint16_t i = 0;
    if (i < 1000) {
        i++;
        MGT_TX(0x02, 0x00, 0x00);
    } else {
        g_MGT.TimeOutFlag = 1;
        i = 0;
        g_MotorEncoder.TestItem = MGT_Test_Stop;
    }
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
        case MGT_Test_Stop:
            break;

        case MGT_Test_MultiturnReset:
            // 原始已注释掉 MgtMagneticEncoderMultiturnReset()
            break;

        case MGT_Test_GetAllData:
            MGT_TX(0x1A, 0x00, 0x00);
            break;

        case MGT_Test_ReadAllEeprom:
            MGT_ReadAllEeprom();
            break;

        case MGT_Test_Initialize:
            MGT_Initialize();
            break;

        case MGT_Test_Speed:
            MGT_Speed();
            break;

        case MGT_Test_FirmwareVersion:
            MGT_TX(0xA2, 0x0305, 0x00);
            g_MotorEncoder.TestItem = MGT_Test_Stop;
            break;

        case MGT_Test_WriteAllEeprom:
            MGT_WriteAllEeprom();
            break;

        case MGT_Test_Command0x32:
            MGT_Command0x32();
            break;

        case MGT_Test_Command0xEA:
            MGT_TX(0xEA, g_MGT.Eeprom.SetAddress, 0x00);
            g_MotorEncoder.TestItem = MGT_Test_Stop;
            break;

        case MGT_Test_OIP:
            MGT_OpenInternalProtocol();
            break;

        case MGT_Test_CIP:
            MGT_TX(0x6D, 0x00, 0x43);  // 'C' = 0x43
            g_MotorEncoder.TestItem = MGT_Test_Stop;
            break;

        case MGT_Test_Command0x7A:
            MGT_Command0x7A();
            break;

        case MGT_Test_Command0xA2:
            MGT_TX(0xA2, g_MGT.Eeprom.Address, 0x00);
            g_MotorEncoder.TestItem = MGT_Test_Stop;
            break;

        case MGT_Test_Firmware:
            MGT_TX(0x40, 0x10, 0x00);
            g_MotorEncoder.TestItem = MGT_Test_Stop;
            break;

        case MGT_Test_WriteDate:
            MGT_WriteDate();
            break;

        case MGT_Test_SetResolution:
            MGT_SetResolution();
            break;

        case MGT_Test_ReadResolution:
            MGT_ReadResolution();
            break;

        case MGT_Test_WriteHWRev:
            MGT_WriteHWRev();
            break;

        case MGT_Test_ReadHWRev:
            MGT_ReadHWRev();
            break;

        case MGT_Test_Timeout:
            MGT_TimeOut();
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
        case 0x0402: return g_MGT.Eeprom.DataBuffer[0][0x6F];
        case 0x0403: return g_MGT.Eeprom.DataBuffer[0][0x70];
        case 0x0404: return g_MGT.FW;        // 读取软件版本 (0xA2@0x0305)
        case 0x0405: return g_MGT.Eeprom.DataBuffer[g_MGT.Eeprom.ReturnPage][g_MGT.Eeprom.ReturnAddress]; // 0xEA 返回值
        case 0x0406: return g_MGT.Eeprom.Data;             // 0xA2 返回值
        case 0x0407: return (uint16_t)(g_MGT.Firmware & 0xFFFF); // 32位 软件版本 低字 Low Word
        case 0x0408: return (uint16_t)((g_MGT.Firmware >> 16) & 0xFFFF); // 32位 软件版本 高字 High Word
        case 0x0409: return g_MGT.ResolutionID;
        default:     return 0xFFFF;
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
            g_MGT.Eeprom.SetAddress = value >> 8;
            g_MGT.Eeprom.Data = value & 0xFF;
            g_MotorEncoder.TestItem = MGT_Test_Command0x32;
            return 0;
        case 0x0407:
            g_MGT.Eeprom.SetAddress = value >> 8;
            g_MotorEncoder.TestItem = MGT_Test_Command0xEA;
            return 0;
        case 0x0408:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_OIP;
            return 0;
        case 0x0409:
            if (value == 1) g_MotorEncoder.TestItem = MGT_Test_CIP;
            return 0;
        case 0x040A:
            g_MGT.Eeprom.Address = value & 0xFFFF;
            return 0;
        case 0x040B:
            g_MGT.Eeprom.Data = value >> 8;
            g_MotorEncoder.TestItem = MGT_Test_Command0x7A;
            return 0;
        case 0x040C:
            g_MGT.Eeprom.Address = value & 0xFFFF;
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
                g_MotorEncoder.HWRevAddr = 0x030D;  // 起始地址
                g_MotorEncoder.TestItem = MGT_Test_WriteHWRev;
            }
            return 0;
        case 0x0411:
            if (value == 1) {
                g_MotorEncoder.HWRevAddr = 0x030D;  // 起始地址
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
