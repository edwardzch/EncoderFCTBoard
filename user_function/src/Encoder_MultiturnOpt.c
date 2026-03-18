/****************************************************************************************
 * @file      Encoder_MGTMag.c
 * @brief     光编磁编码器协议实现
 * @author    
 * @date      2026-03-17
 * @note      用于测试光编
 ****************************************************************************************/
#include "Encoder_MultiturnOpt.h"
#include "encoder_driver.h"
#include "modbus_function.h"
#include "encoder_modbus.h"
#include <string.h>
#include <stdio.h>

// ================= 全局变量定义 =================
MulOpt_t g_MulOpt = {0};
MulOpt_PNH2612_t g_PNH2612 = {0};

// ================= 私有函数声明 =================
static void MNS_SinCos_MaxMin(void);
static void MultiturnOpticalEncoderLedDac(void);
static void MultiturnOpticalEncoderSectorCollect(void);
static void MultiturnOpticalEncoderMTABContinuousCheck(void);
static void MultiturnOpticalEncoderHallContinuousCheck(void);
static void MultiturnOpticalMNSAnalogSynCheckResult(void);

/****************************************************************************************
* 函数名称：MulOpt_Init
* 函数功能：初始化多圈光编码器模块，清零状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulOpt_Init(void) {
    memset(&g_MulOpt, 0, sizeof(g_MulOpt));
    memset(&g_PNH2612, 0, sizeof(g_PNH2612));

    // 初始化M/N/S信号极值
    g_PNH2612.M.SinDataMax = 0;
    g_PNH2612.M.CosDataMax = 0;
    g_PNH2612.N.SinDataMax = 0;
    g_PNH2612.N.CosDataMax = 0;
    g_PNH2612.S.SinDataMax = 0;
    g_PNH2612.S.CosDataMax = 0;

    g_PNH2612.M.SinDataMin = 0xFFFF;
    g_PNH2612.M.CosDataMin = 0xFFFF;
    g_PNH2612.N.SinDataMin = 0xFFFF;
    g_PNH2612.N.CosDataMin = 0xFFFF;
    g_PNH2612.S.SinDataMin = 0xFFFF;
    g_PNH2612.S.CosDataMin = 0xFFFF;

    // 初始化EEPROM页数
    g_MulOpt.Eeprom.Page = 0;
}

// ================= 数据发送函数 =================
void MulOpt_TX(uint8_t cmd, uint8_t addr, uint8_t data) 
{
	
    g_MulOpt.TxID = cmd;
    g_MulOpt.TxData[0] = cmd;

    switch (cmd) {
        case 0x1A:  // 读取全部数据
            EncDrv_SendDMA(g_MulOpt.TxData, 1, 11);
            break;
				
				case 0x9D:  // 读取当前位置值				
				case 0x3D:  // 电池/温度
        case 0x62:  // 多圈复位
            EncDrv_SendDMA(g_MulOpt.TxData, 1, 6);
            break;
				
        case 0x25:  // Hall读取
            EncDrv_SendDMA(g_MulOpt.TxData, 1, 10);
            break;
				
        case 0x6D:  // 内部命令
            g_MulOpt.IPID = data;
            if (data == Encoder_CED_Test) {//EEPROM初始化
                g_MulOpt.TxData[1] = 0x63;
                g_MulOpt.TxData[2] = 0x65;
                g_MulOpt.TxData[3] = 0x64;
            } else if (data == Encoder_CIP_Test) {//关闭内部指令
                g_MulOpt.TxData[1] = 0x43;
                g_MulOpt.TxData[2] = 0x49;
                g_MulOpt.TxData[3] = 0x50;
            } else if(data == Encoder_OIP_Test){//打开内部指令
                g_MulOpt.TxData[1] = 0x4F;
                g_MulOpt.TxData[2] = 0x49;
                g_MulOpt.TxData[3] = 0x50;
            }
            g_MulOpt.TxData[4] = Enc_CRC8((uint8_t *)g_MulOpt.TxData, 4);
            EncDrv_SendDMA((uint8_t *)g_MulOpt.TxData, 5, 6);
            break;
						
        case 0xAD:  // 批量读EEPROM
            g_MulOpt.TxData[1] = g_MulOpt.Eeprom.SetPage;
            g_MulOpt.TxData[2] = g_MulOpt.Eeprom.SetAddress;
            g_MulOpt.TxData[3] = g_MulOpt.Eeprom.SetDNum;
            g_MulOpt.TxData[4] = Enc_CRC8((uint8_t *)g_MulOpt.TxData, 4);
            EncDrv_SendDMA((uint8_t *)g_MulOpt.TxData, 5, 5 + g_MulOpt.Eeprom.SetDNum);
            break;

        case 0x35:  // 批量写EEPROM
						g_MulOpt.TxData[1] = g_MulOpt.Eeprom.SetPage;
						g_MulOpt.TxData[2] = g_MulOpt.Eeprom.SetAddress;
						g_MulOpt.TxData[3] = g_MulOpt.Eeprom.SetDNum;
						uint8_t j;
						for (j = 0; j < g_MulOpt.Eeprom.SetDNum; j++) {
								g_MulOpt.TxData[4 + j] = g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress + j];
						}
						g_MulOpt.TxData[4 + j] = Enc_CRC8((uint8_t *)g_MulOpt.TxData, 4 + j);
						EncDrv_SendDMA((uint8_t *)g_MulOpt.TxData, 5 + g_MulOpt.Eeprom.SetDNum, 5 + g_MulOpt.Eeprom.SetDNum);
            break;

        case 0xD5:  // LED DAC设置
            g_MulOpt.TxData[1] = g_PNH2612.LEDDAC0;
            g_MulOpt.TxData[2] = g_PNH2612.LEDDAC1;
            g_MulOpt.TxData[3] = Enc_CRC8(g_MulOpt.TxData, 3);
            EncDrv_SendDMA(g_MulOpt.TxData, 4, 4);
            break;

        case 0x85:  // 读取3码道ADC数据
            EncDrv_SendDMA(g_MulOpt.TxData, 1, 12);
            break;

        case 0x0D:  // 读取3码道位置值
        case 0x15:  // 读取MN delta值及MT sector
            EncDrv_SendDMA(g_MulOpt.TxData, 1, 8);
            break;

        case 0x75:  // 读取内部报警
            EncDrv_SendDMA(g_MulOpt.TxData, 1, 5);
            break;

        default:
            break;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_RxComplete
* 函数功能：接收完成处理，解析响应帧并更新编码器数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulOpt_RxComplete(void) {
    if (g_MulOpt.RxDataCnt < 1) return;

    uint8_t cmd = g_MulOpt.RxData[0];
    uint8_t crc_ok = 0;

    switch (cmd) {
        case 0x1A:  // 读取全部数据
            if (g_MulOpt.RxDataCnt >= 11) {
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 10);
                crc_ok = (g_MulOpt.XorCrcData == g_MulOpt.RxData[10]);
                if (!crc_ok) {
                    g_MulOpt.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Status.all = g_MulOpt.RxData[1];
                g_MulOpt.SingleTurnPosition = ((uint32_t)g_MulOpt.RxData[4] << 16) | 
                                            ((uint32_t)g_MulOpt.RxData[3] << 8) | 
                                            g_MulOpt.RxData[2];
                g_MulOpt.ResolutionID = g_MulOpt.RxData[5];
                g_MulOpt.MultiTurnPosition = ((uint16_t)g_MulOpt.RxData[7] << 8) | g_MulOpt.RxData[6];
                g_MulOpt.Alarm.all = g_MulOpt.RxData[9];
                g_MulOpt.Error.bit.EC = 0;
            }
            break;

        case 0x62:  // 多圈复位响应
            if (g_MulOpt.RxDataCnt >= 6) {
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 5);
                crc_ok = (g_MulOpt.XorCrcData == g_MulOpt.RxData[5]);
                if (!crc_ok) {
                    g_MulOpt.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Status.all = g_MulOpt.RxData[1];
                g_MulOpt.SingleTurnPosition = ((uint32_t)g_MulOpt.RxData[4] << 16) | 
                                            ((uint32_t)g_MulOpt.RxData[3] << 8) | 
                                            g_MulOpt.RxData[2];
                g_MulOpt.Error.bit.EC = 0;
            }
            break;

        case 0x25:  // Hall读取响应
            if (g_MulOpt.RxDataCnt >= 8) {
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                g_PNH2612.HallSector = (g_MulOpt.RxData[5] >> 4) & 0x0F;
                g_PNH2612.MTABSector = g_MulOpt.RxData[5] & 0x0F;
                g_MulOpt.Hall.Sector = g_PNH2612.HallSector;
                g_MulOpt.MTAB.Sector = g_PNH2612.MTABSector;
            }
            break;

        case 0x3D:  // 电池/温度响应
            if (g_MulOpt.RxDataCnt >= 6) {
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Temperature = g_MulOpt.RxData[3];
                g_MulOpt.BatteryVoltage = g_MulOpt.RxData[4];
            }
            break;

        case 0x6D:  // 初始化响应
            if (g_MulOpt.RxDataCnt >= 6) {
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 5);
                if (g_MulOpt.XorCrcData != g_MulOpt.RxData[5]) {
                    g_MulOpt.XorCrcError = 1; 
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Status0x6D = g_MulOpt.RxData[4];
                g_MulOpt.Error.bit.EC = 0;
            }
            break;

        case 0xEA:  // EEPROM单字节读取响应
            if (g_MulOpt.RxDataCnt == 4) {
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 3);
                if (g_MulOpt.XorCrcData != g_MulOpt.RxData[3]) {
                    g_MulOpt.XorCrcError = 1; 
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                
                g_MulOpt.Eeprom.ReturnAddress = g_MulOpt.RxData[1] & 0x7F;
                if (g_MulOpt.Eeprom.ReturnAddress == MULOPT_PNS_ADDRESS) {
                    g_MulOpt.Eeprom.ReturnPage = g_MulOpt.RxData[2];
                    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.ReturnPage][g_MulOpt.Eeprom.ReturnAddress] = g_MulOpt.RxData[2];
                } else if (g_MulOpt.Eeprom.ReturnAddress < MULOPT_PNS_ADDRESS) {
                    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.ReturnPage][g_MulOpt.Eeprom.ReturnAddress] = g_MulOpt.RxData[2];
                }
                g_MulOpt.Error.bit.EC = 0;
            }
            break;

        case 0x32:  // EEPROM单字节读取响应(简化版)
            if (g_MulOpt.RxDataCnt == 4) {
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 3);
                if (g_MulOpt.XorCrcData != g_MulOpt.RxData[3]) {
                    g_MulOpt.XorCrcError = 1; 
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                
                g_MulOpt.Eeprom.Address = g_MulOpt.RxData[1];
                g_MulOpt.Eeprom.Data = g_MulOpt.RxData[2];
                g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.Page][g_MulOpt.Eeprom.Address] = g_MulOpt.Eeprom.Data;
                g_MulOpt.Error.bit.EC = 0;
                g_MulOpt.Flag.bit.CF32 = 0x00;
            }
            break;

        case 0x85:  // 读取3码道ADC数据响应
            if (g_MulOpt.RxDataCnt == 12) {
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 11);
                if (g_MulOpt.XorCrcData != g_MulOpt.RxData[11]) {
                    g_MulOpt.XorCrcError = 1; 
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;

                // 解析M路数据
                g_PNH2612.M.SinData = g_MulOpt.RxData[1] | ((g_MulOpt.RxData[7] >> 4) << 8);
                g_PNH2612.M.CosData = g_MulOpt.RxData[2] | ((g_MulOpt.RxData[7] & 0x0F) << 8);

                // 解析N路数据
                g_PNH2612.N.SinData = g_MulOpt.RxData[3] | ((g_MulOpt.RxData[8] >> 4) << 8);
                g_PNH2612.N.CosData = g_MulOpt.RxData[4] | ((g_MulOpt.RxData[8] & 0x0F) << 8);

                // 解析S路数据
                g_PNH2612.S.SinData = g_MulOpt.RxData[5] | ((g_MulOpt.RxData[9] >> 4) << 8);
                g_PNH2612.S.CosData = g_MulOpt.RxData[6] | ((g_MulOpt.RxData[9] & 0x0F) << 8);

                // 解析扇区信息
                g_PNH2612.HallSector = (g_MulOpt.RxData[10] & 0x30) >> 4;
                g_PNH2612.MTABSector = (g_MulOpt.RxData[10] & 0x03);

                // 更新全局结构体
                g_MulOpt.Hall.Sector = g_PNH2612.HallSector;
                g_MulOpt.MTAB.Sector = g_PNH2612.MTABSector;

                // 处理数据
                MNS_SinCos_MaxMin();
                MultiturnOpticalEncoderSectorCollect();
                MultiturnOpticalEncoderMTABContinuousCheck();
                MultiturnOpticalEncoderHallContinuousCheck();
                MultiturnOpticalMNSAnalogSynCheckResult();
                MultiturnOpticalEncoderLedDac();

                g_MulOpt.Error.bit.EC = 0;
            }
            break;

        case 0x0D:  // 读取3码道位置值响应
            if (g_MulOpt.RxDataCnt == 8) {
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                g_PNH2612.M.Absolute = g_MulOpt.RxData[1] | (g_MulOpt.RxData[2] << 8);
                g_PNH2612.N.Absolute = g_MulOpt.RxData[3] | (g_MulOpt.RxData[4] << 8);
                g_PNH2612.S.Absolute = g_MulOpt.RxData[5] | (g_MulOpt.RxData[6] << 8);
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 7);
                if (g_MulOpt.XorCrcData != g_MulOpt.RxData[7]) {
                    g_MulOpt.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
            }
            break;

        case 0x15:  // 读取MN delta值及MT sector
            if (g_MulOpt.RxDataCnt == 8) {
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                g_PNH2612.MNdelta = g_MulOpt.RxData[1] | (g_MulOpt.RxData[2] << 8);
                g_PNH2612.MTABSector = g_MulOpt.RxData[3];
                g_PNH2612.Q4D = g_MulOpt.RxData[4];
                g_PNH2612.Q14D = g_MulOpt.RxData[5] | (g_MulOpt.RxData[6] << 8);
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 7);
                if (g_MulOpt.XorCrcData != g_MulOpt.RxData[7]) {
                    g_MulOpt.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
            }
            break;

        case 0x9D:  // 读取当前位置值响应
            if (g_MulOpt.RxDataCnt == 6) {
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.Error.bit.DC = 0;
                g_MulOpt.RxID = cmd;
                g_PNH2612.AbsolutePosition = g_MulOpt.RxData[1] | 
                                          (g_MulOpt.RxData[2] << 8) | 
                                          (g_MulOpt.RxData[3] << 16) | 
                                          (g_MulOpt.RxData[4] << 24);
                g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, 5);
                if (g_MulOpt.XorCrcData != g_MulOpt.RxData[5]) {
                    g_MulOpt.XorCrcError = 1;
                    Work_Alarm = 0x02;
                    g_MulOpt.RxDataCnt = 0;
                    return;
                }
            }
            break;

        case 0xAD:  // 批量读EEPROM响应
            {
                uint8_t dnum = g_MulOpt.RxData[3];
                uint8_t expected_len = 5 + dnum;
                if (g_MulOpt.RxDataCnt >= expected_len) {
                    g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, expected_len - 1);
                    crc_ok = (g_MulOpt.XorCrcData == g_MulOpt.RxData[expected_len - 1]);
                    if (!crc_ok) {
                        g_MulOpt.XorCrcError = 1;
                        Work_Alarm = 0x02;
                        g_MulOpt.RxDataCnt = 0;
                        return;
                    }
                    g_MulOpt.TimeoutCnt = 0;
                    g_MulOpt.Error.bit.DC = 0;
                    g_MulOpt.RxID = cmd;
                    uint8_t page = g_MulOpt.RxData[1];
                    uint8_t addr = g_MulOpt.RxData[2];
                    if (page < MULOPT_PAGE_NUMBER) {
                        for (uint8_t i = 0; i < dnum && (addr + i) < MULOPT_EEPROM_ADDR; i++) {
                            g_MulOpt.Eeprom.DataBuffer[page][addr + i] = g_MulOpt.RxData[4 + i];
                        }
                    }
                    g_MulOpt.Error.bit.EC = 0;
                }
            }
            break;

        case 0x35:  // 批量写EEPROM响应
            {
                uint8_t dnum = g_MulOpt.RxData[3];
                uint8_t expected_len = 5 + dnum;
                if (g_MulOpt.RxDataCnt >= expected_len) {
                    g_MulOpt.XorCrcData = Enc_CRC8(g_MulOpt.RxData, expected_len - 1);
                    crc_ok = (g_MulOpt.XorCrcData == g_MulOpt.RxData[expected_len - 1]);
                    if (!crc_ok) {
                        g_MulOpt.XorCrcError = 1;
                        Work_Alarm = 0x02;
                        g_MulOpt.RxDataCnt = 0;
                        return;
                    }
                    g_MulOpt.TimeoutCnt = 0;
                    g_MulOpt.Error.bit.DC = 0;
                    g_MulOpt.RxID = cmd;
                    uint8_t page = g_MulOpt.RxData[1];
                    uint8_t addr = g_MulOpt.RxData[2];
                    if (page < MULOPT_PAGE_NUMBER) {
                        for (uint8_t i = 0; i < dnum && (addr + i) < MULOPT_EEPROM_ADDR; i++) {
                            g_MulOpt.Eeprom.DataBuffer[page][addr + i] = g_MulOpt.RxData[4 + i];
                        }
                    }
                    g_MulOpt.Error.bit.EC = 0;
                }
            }
            break;

        default:
						g_MulOpt.RxDataCnt = 0;
            break;
    }

    
}


// ================= M/N/S信号极值检测 =================
static void MNS_SinCos_MaxMin(void) {
    /* M路Sin/Cos极值检测 */
    if(g_PNH2612.M.SinData > g_PNH2612.M.SinDataMax){
        g_PNH2612.M.SinDataMax = g_PNH2612.M.SinData;
    }
    if(g_PNH2612.M.SinData < g_PNH2612.M.SinDataMin){
        g_PNH2612.M.SinDataMin = g_PNH2612.M.SinData;
    }
    if(g_PNH2612.M.CosData > g_PNH2612.M.CosDataMax){
        g_PNH2612.M.CosDataMax = g_PNH2612.M.CosData;
    }
    if(g_PNH2612.M.CosData < g_PNH2612.M.CosDataMin){
        g_PNH2612.M.CosDataMin = g_PNH2612.M.CosData;
    }

    /* N路Sin/Cos极值检测 */
    if(g_PNH2612.N.SinData > g_PNH2612.N.SinDataMax){
        g_PNH2612.N.SinDataMax = g_PNH2612.N.SinData;
    }
    if(g_PNH2612.N.SinData < g_PNH2612.N.SinDataMin){
        g_PNH2612.N.SinDataMin = g_PNH2612.N.SinData;
    }
    if(g_PNH2612.N.CosData > g_PNH2612.N.CosDataMax){
        g_PNH2612.N.CosDataMax = g_PNH2612.N.CosData;
    }
    if(g_PNH2612.N.CosData < g_PNH2612.N.CosDataMin){
        g_PNH2612.N.CosDataMin = g_PNH2612.N.CosData;
    }

    /* S路Sin/Cos极值检测 */
    if(g_PNH2612.S.SinData > g_PNH2612.S.SinDataMax){
        g_PNH2612.S.SinDataMax = g_PNH2612.S.SinData;
    }
    if(g_PNH2612.S.SinData < g_PNH2612.S.SinDataMin){
        g_PNH2612.S.SinDataMin = g_PNH2612.S.SinData;
    }
    if(g_PNH2612.S.CosData > g_PNH2612.S.CosDataMax){
        g_PNH2612.S.CosDataMax = g_PNH2612.S.CosData;
    }
    if(g_PNH2612.S.CosData < g_PNH2612.S.CosDataMin){
        g_PNH2612.S.CosDataMin = g_PNH2612.S.CosData;
    }
}

// ================= LED DAC测试 =================
static void MultiturnOpticalEncoderLedDac(void) {
    if(g_MulOpt.Flag.bit.LedDac){
        if(g_PNH2612.LEDTestDACCnt >= 1){
            if(g_PNH2612.LEDTestDACCnt > 1){
                if(g_PNH2612.LEDTestDACCnt == 6){
                    g_PNH2612.LEDTestDACResult = 0x02; // 失败
                    g_MulOpt.Flag.bit.LedDac = 0;
                }else{
                    if(g_PNH2612.MSinDataBuffer[g_PNH2612.LEDTestDACCnt - 2] != 
                       g_PNH2612.MSinDataBuffer[g_PNH2612.LEDTestDACCnt - 1]){
                        g_PNH2612.LEDTestDACResult = 0x01; // 成功
                        g_MulOpt.Flag.bit.LedDac = 0;
                    }
                }
            }
        }
        g_PNH2612.MSinDataBuffer[g_PNH2612.LEDTestDACCnt] = g_PNH2612.M.SinData;
        g_PNH2612.LEDTestDACCnt++;
    }
}

// ================= 扇区数据收集 =================
static void MultiturnOpticalEncoderSectorCollect(void) {
    if(g_MulOpt.Flag.bit.MTABHALL == 1){
        g_MulOpt.Hall.Buffer[0] = g_PNH2612.HallSector;
        g_MulOpt.MTAB.Buffer[0] = g_PNH2612.MTABSector;
        g_MulOpt.Flag.bit.MTABHALL = 0;
    }else if(g_MulOpt.Flag.bit.MTABHALL == 2){
        g_MulOpt.Hall.Buffer[1] = g_PNH2612.HallSector;
        g_MulOpt.MTAB.Buffer[1] = g_PNH2612.MTABSector;
        g_MulOpt.Flag.bit.MTABHALL = 0;
    }else if(g_MulOpt.Flag.bit.MTABHALL == 3){
        g_MulOpt.Hall.Buffer[2] = g_PNH2612.HallSector;
        g_MulOpt.MTAB.Buffer[2] = g_PNH2612.MTABSector;
        g_MulOpt.Flag.bit.MTABHALL = 0;
    }else if(g_MulOpt.Flag.bit.MTABHALL == 4){
        g_MulOpt.Hall.Buffer[3] = g_PNH2612.HallSector;
        g_MulOpt.MTAB.Buffer[3] = g_PNH2612.MTABSector;
        g_MulOpt.Flag.bit.MTABHALL = 0;
    }
}

// ================= MTAB连续检测 =================
static void MultiturnOpticalEncoderMTABContinuousCheck(void) {
    static uint8_t prev_logic = 0xFF;
    static uint8_t start_logic = 0xFF;
    static uint8_t change_cnt = 0;
    static uint8_t circle_done = 0;

    if(circle_done) return;

    uint8_t curr_phys = g_PNH2612.MTABSector;
    uint8_t curr_logic = MTAB_Map[curr_phys & 0x03];

    if(prev_logic == 0xFF) {
        prev_logic = curr_logic;
        start_logic = curr_logic;
        change_cnt = 0;
        circle_done = 0;
        g_MulOpt.MTAB.Result = 0;
        g_MulOpt.MTAB.Check = ENC_STATUS_TESTING;
        g_MulOpt.MTAB.Cnt = 0;
        g_MulOpt.MTAB.GlitchCnt = 0;
        g_MulOpt.MTAB.JitterCnt = 0;
        g_MulOpt.MTAB.CountErrCnt = 0;
        return;
    }

    if(curr_logic == prev_logic) return;

    uint8_t diff = (curr_logic + 4 - prev_logic) & 0x03;

    if(diff == 2) {
        g_MulOpt.MTAB.GlitchCnt++;
        g_MulOpt.MTAB.Check = ENC_STATUS_ERR_GLITCH;
        g_MulOpt.MTAB.Result = 0;
        prev_logic = curr_logic;
    } else if(diff != 1) {
        g_MulOpt.MTAB.JitterCnt++;
        g_MulOpt.MTAB.Check = ENC_STATUS_ERR_JITTER;
        g_MulOpt.MTAB.Result = 0;
        prev_logic = curr_logic;
    } else {
        change_cnt++;
        prev_logic = curr_logic;
    }

    g_MulOpt.MTAB.Cnt = change_cnt;

    if((curr_logic == start_logic) && (change_cnt >= 4)) {
        circle_done = 1;
        if(change_cnt != 4) {
            g_MulOpt.MTAB.CountErrCnt++;
            g_MulOpt.MTAB.Check = ENC_STATUS_ERR_COUNT;
            g_MulOpt.MTAB.Result = 0;
        }

        if((g_MulOpt.MTAB.GlitchCnt == 0) && 
           (g_MulOpt.MTAB.JitterCnt == 0) && 
           (g_MulOpt.MTAB.CountErrCnt == 0)) {
            g_MulOpt.MTAB.Result = 1;
            g_MulOpt.MTAB.Check = ENC_STATUS_PASS;
        }
    }
}

// ================= HALL连续检测 =================
static void MultiturnOpticalEncoderHallContinuousCheck(void) {
    static uint8_t prev_logic = 0xFF;
    static uint8_t start_logic = 0xFF;
    static uint8_t change_cnt = 0;
    static uint8_t circle_done = 0;

    if(circle_done) return;

    uint8_t curr_phys = g_PNH2612.HallSector;
    uint8_t curr_logic = HALL_Map[curr_phys & 0x03];

    if(prev_logic == 0xFF) {
        prev_logic = curr_logic;
        start_logic = curr_logic;
        change_cnt = 0;
        circle_done = 0;
        g_MulOpt.Hall.Result = 0;
        g_MulOpt.Hall.Check = ENC_STATUS_TESTING;
        g_MulOpt.Hall.Cnt = 0;
        g_MulOpt.Hall.GlitchCnt = 0;
        g_MulOpt.Hall.JitterCnt = 0;
        g_MulOpt.Hall.CountErrCnt = 0;
        return;
    }

    if(curr_logic == prev_logic) return;

    uint8_t diff = (curr_logic + 4 - prev_logic) & 0x03;

    if(diff == 2) {
        g_MulOpt.Hall.GlitchCnt++;
        g_MulOpt.Hall.Check = ENC_STATUS_ERR_GLITCH;
        g_MulOpt.Hall.Result = 0;
        prev_logic = curr_logic;
    } else if(diff != 1) {
        g_MulOpt.Hall.JitterCnt++;
        g_MulOpt.Hall.Check = ENC_STATUS_ERR_JITTER;
        g_MulOpt.Hall.Result = 0;
        prev_logic = curr_logic;
    } else {
        change_cnt++;
        prev_logic = curr_logic;
    }

    g_MulOpt.Hall.Cnt = change_cnt;

    if((curr_logic == start_logic) && (change_cnt >= 4)) {
        circle_done = 1;
        if(change_cnt != 4) {
            g_MulOpt.Hall.CountErrCnt++;
            g_MulOpt.Hall.Check = ENC_STATUS_ERR_COUNT;
            g_MulOpt.Hall.Result = 0;
        }

        if((g_MulOpt.Hall.GlitchCnt == 0) && 
           (g_MulOpt.Hall.JitterCnt == 0) && 
           (g_MulOpt.Hall.CountErrCnt == 0)) {
            g_MulOpt.Hall.Result = 1;
            g_MulOpt.Hall.Check = ENC_STATUS_PASS;
        }
    }
}

// ================= MNS模拟量同步检查 =================
static void MultiturnOpticalMNSAnalogSynCheckResult(void) {
    const uint16_t LOWER_LIMIT = 2000;
    const uint16_t UPPER_LIMIT = 2100;

    uint8_t m_ok = (g_PNH2612.M.SinData > LOWER_LIMIT && g_PNH2612.M.SinData < UPPER_LIMIT) &&
                   (g_PNH2612.M.CosData > LOWER_LIMIT && g_PNH2612.M.CosData < UPPER_LIMIT);

    uint8_t n_ok = (g_PNH2612.N.SinData > LOWER_LIMIT && g_PNH2612.N.SinData < UPPER_LIMIT) &&
                   (g_PNH2612.N.CosData > LOWER_LIMIT && g_PNH2612.N.CosData < UPPER_LIMIT);

    uint8_t s_ok = (g_PNH2612.S.SinData > LOWER_LIMIT && g_PNH2612.S.SinData < UPPER_LIMIT) &&
                   (g_PNH2612.S.CosData > LOWER_LIMIT && g_PNH2612.S.CosData < UPPER_LIMIT);

    if(m_ok && n_ok && s_ok) {
        g_PNH2612.MNSAnalogSynchronousCheckResult = 1;
    } else {
        g_PNH2612.MNSAnalogSynchronousCheckResult = 0;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_ReadHWRev
* 函数功能：从EEPROM主缓存解析多圈光学编码器硬件版本号
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-2
****************************************************************************************/
void MulOpt_ReadHWRev(void)
{
    snprintf(g_MotorEncoder.HWRevBuffer, sizeof(g_MotorEncoder.HWRevBuffer), 
             "%c.%c.%d.%d", 
             g_MulOpt.Eeprom.DataBuffer[2][0x70],  // 主版本字符
             g_MulOpt.Eeprom.DataBuffer[2][0x71],  // 次版本字符
             g_MulOpt.Eeprom.DataBuffer[2][0x72],  // 修订号数字
             g_MulOpt.Eeprom.DataBuffer[2][0x73]); // 构建号数字
}

/****************************************************************************************
* 函数名称：MulOpt_ReadFWRev
* 函数功能：从EEPROM主缓存解析多圈光学编码器硬件版本号
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-2
****************************************************************************************/
void MulOpt_ReadFWRev(void)
{
    snprintf(g_MotorEncoder.FWRevBuffer, sizeof(g_MotorEncoder.FWRevBuffer),
             "%d.%d.%d.%c",
             g_MulOpt.Eeprom.DataBuffer[3][20],  // 主版本号
             g_MulOpt.Eeprom.DataBuffer[3][21],  // 次版本号
             g_MulOpt.Eeprom.DataBuffer[3][22],  // 修订号
             g_MulOpt.Eeprom.DataBuffer[3][23]); // 构建标识
}


// ================= 测试功能入口 =================
void MulOpt_Test(uint8_t testItem) {
    switch(testItem) {
        case MulOpt_Test_Stop: 
            break;

        case MulOpt_Test_MultiturnReset:
            if(g_MulOpt.Cnt.CF62 < 10) {
                MulOpt_TX(0x62, 0x00, 0x00);
                g_MulOpt.Cnt.CF62++;
                if(g_MulOpt.Cnt.CF62 == 10) {
                    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
                }
            }
            break;

        case MulOpt_Test_GetAllData:
            MulOpt_TX(0x1A, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_ReadAllEeprom:
            if(g_MulOpt.Eeprom.Page < MULOPT_PAGE_NUMBER) {
                g_MulOpt.Eeprom.SetDNum = 0x80;
                g_MulOpt.Eeprom.SetAddress = 0x00;
                MulOpt_TX(0xAD, 0x00, 0x00);
                g_MulOpt.Eeprom.Page++;
            } else {
                // 读取完成后处理硬件版本和固件版本
								MulOpt_ReadHWRev();
								MulOpt_ReadFWRev();
                g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            }
            break;

        case MulOpt_Test_Initialize:
            if(g_MulOpt.Cnt.CFced < 5) {
                MulOpt_TX(0x6D, 0x00, Encoder_CED_Test);
                g_MulOpt.Cnt.CFced++;
            } else {
                g_MulOpt.Cnt.CFced = 0;
                g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            }
            break;

        case MulOpt_Test_WriteResult:
            g_MulOpt.Eeprom.SetPage = 0x02;
            g_MulOpt.Eeprom.SetAddress = 0x00;
            g_MulOpt.Eeprom.SetDNum = 0x01;
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress] = g_MulOpt.TestResult;
            MulOpt_TX(0x35, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_TB:
            MulOpt_TX(0x3D, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_OIP:
            if(g_MulOpt.Cnt.CFOIP < 5) {
                MulOpt_TX(0x6D, 0x00, Encoder_OIP_Test);
                g_MulOpt.Cnt.CFOIP++;
            } else {
                g_MulOpt.Cnt.CFOIP = 0;
                g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            }
            break;

        case MulOpt_Test_CIP:
            MulOpt_TX(0x6D, 0x00, Encoder_CIP_Test);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_WriteDate:
            g_MulOpt.Eeprom.SetPage = 0x02;
            g_MulOpt.Eeprom.SetAddress = 0x68;
            g_MulOpt.Eeprom.SetDNum = 0x04;
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress] = g_MotorEncoder.TestYear;
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+1] = g_MotorEncoder.TestMoon;
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+2] = g_MotorEncoder.TestDay;
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+3] = g_MotorEncoder.TestHour;
            MulOpt_TX(0x35, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_DAC:
            g_PNH2612.LEDDAC0 = g_PNH2612.LEDTestDAC & 0xFF;
            g_PNH2612.LEDDAC1 = (g_PNH2612.LEDTestDAC >> 8) & 0xFF;
            MulOpt_TX(0xD5, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_ReadAnalogDataTest;
            break;

        case MulOpt_Test_LightIntensity:
            if(g_MulOpt.Flag.bit.LedDac){
                g_PNH2612.LEDTestDAC = (1950 - 50 * g_PNH2612.LEDTestDACCnt);
            }
            g_PNH2612.LEDDAC0 = g_PNH2612.LEDTestDAC & 0xFF;
            g_PNH2612.LEDDAC1 = (g_PNH2612.LEDTestDAC >> 8) & 0xFF;
            MulOpt_TX(0xD5, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_ReadAnalogDataTest:
            MulOpt_TX(0x85, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_ReadInternalAlarm:
            MulOpt_TX(0x75, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_WriteHWRev:
            g_MulOpt.Eeprom.SetPage = 0x02;
            g_MulOpt.Eeprom.SetAddress = 0x70;
            g_MulOpt.Eeprom.SetDNum = 0x04;
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress] = 'H';
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+1] = 'O';
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+2] = 1;
            g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+3] = g_MotorEncoder.HWRevData;
            MulOpt_TX(0x35, 0x00, 0x00);
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_Sector:
            MultiturnOpticalEncoderHallCheck();
            MultiturnOpticalEncoderMTABCheck();
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
            break;

        case MulOpt_Test_MNSPosition:
            if(g_PNH2612.M.PositionFlag > 0) {
                MulOpt_TX(0x1A, 0x00, 0x00);
                MultiturnOpticalEncoderMPositionMark1();
            } else {
                MulOpt_TX(0x0D, 0x00, 0x00);
                MultiturnOpticalEncoderMNSSpeedCheck();
                MultiturnOpticalEncoderMPositionMark();
            }
            break;

        case MulOpt_Test_DACAdjustment:
            MultiturnOpticalEncoderDACAdjustment();
            break;

        case MulOpt_Test_MNSAnalogMaxCheck:
            MultiturnOpticalEncoderMNSAnalogMaxCheck();
            break;

        default:
            break;
    }
}

// ================= HALL检查 =================
static void MultiturnOpticalEncoderHallCheck(void) {
    switch(g_MulOpt.Hall.Buffer[0]) {
        case 0x00:
            if(g_MulOpt.Hall.Buffer[1] == 3 && 
               g_MulOpt.Hall.Buffer[2] == 2 && 
               g_MulOpt.Hall.Buffer[3] == 1) {
                g_MulOpt.Hall.Result = 1;
            } else {
                g_MulOpt.Hall.Result = 0;
            }
            break;

        case 0x01:
            if(g_MulOpt.Hall.Buffer[1] == 0 && 
               g_MulOpt.Hall.Buffer[2] == 3 && 
               g_MulOpt.Hall.Buffer[3] == 2) {
                g_MulOpt.Hall.Result = 1;
            } else {
                g_MulOpt.Hall.Result = 0;
            }
            break;

        case 0x02:
            if(g_MulOpt.Hall.Buffer[1] == 1 && 
               g_MulOpt.Hall.Buffer[2] == 0 && 
               g_MulOpt.Hall.Buffer[3] == 3) {
                g_MulOpt.Hall.Result = 1;
            } else {
                g_MulOpt.Hall.Result = 0;
            }
            break;

        case 0x03:
            if(g_MulOpt.Hall.Buffer[1] == 2 && 
               g_MulOpt.Hall.Buffer[2] == 1 && 
               g_MulOpt.Hall.Buffer[3] == 0) {
                g_MulOpt.Hall.Result = 1;
            } else {
                g_MulOpt.Hall.Result = 0;
            }
            break;
    }
}

// ================= MTAB检查 =================
static void MultiturnOpticalEncoderMTABCheck(void) {
    switch(g_MulOpt.MTAB.Buffer[0]) {
        case 0x00:
            if(g_MulOpt.MTAB.Buffer[1] == 1 && 
               g_MulOpt.MTAB.Buffer[2] == 3 && 
               g_MulOpt.MTAB.Buffer[3] == 2) {
                g_MulOpt.MTAB.Result = 1;
            } else {
                g_MulOpt.MTAB.Result = 0;
            }
            break;

        case 0x01:
            if(g_MulOpt.MTAB.Buffer[1] == 3 && 
               g_MulOpt.MTAB.Buffer[2] == 2 && 
               g_MulOpt.MTAB.Buffer[3] == 0) {
                g_MulOpt.MTAB.Result = 1;
            } else {
                g_MulOpt.MTAB.Result = 0;
            }
            break;

        case 0x02:
            if(g_MulOpt.MTAB.Buffer[1] == 0 && 
               g_MulOpt.MTAB.Buffer[2] == 1 && 
               g_MulOpt.MTAB.Buffer[3] == 3) {
                g_MulOpt.MTAB.Result = 1;
            } else {
                g_MulOpt.MTAB.Result = 0;
            }
            break;

        case 0x03:
            if(g_MulOpt.MTAB.Buffer[1] == 2 && 
               g_MulOpt.MTAB.Buffer[2] == 0 && 
               g_MulOpt.MTAB.Buffer[3] == 1) {
                g_MulOpt.MTAB.Result = 1;
            } else {
                g_MulOpt.MTAB.Result = 0;
            }
            break;
    }
}

// ================= M/N/S速度检查 =================
static void MultiturnOpticalEncoderMNSSpeedCheck(void) {
    // 更新M路缓冲区
    g_PNH2612.M.AbsoluteBuffer[1] = g_PNH2612.M.AbsoluteBuffer[0];
    g_PNH2612.M.AbsoluteBuffer[0] = g_PNH2612.M.Absolute;
    g_PNH2612.M.AbsoluteDifference = g_PNH2612.M.AbsoluteBuffer[0] - g_PNH2612.M.AbsoluteBuffer[1];

    // 更新N路缓冲区
    g_PNH2612.N.AbsoluteBuffer[1] = g_PNH2612.N.AbsoluteBuffer[0];
    g_PNH2612.N.AbsoluteBuffer[0] = g_PNH2612.N.Absolute;
    g_PNH2612.N.AbsoluteDifference = g_PNH2612.N.AbsoluteBuffer[0] - g_PNH2612.N.AbsoluteBuffer[1];

    // 更新S路缓冲区
    g_PNH2612.S.AbsoluteBuffer[1] = g_PNH2612.S.AbsoluteBuffer[0];
    g_PNH2612.S.AbsoluteBuffer[0] = g_PNH2612.S.Absolute;
    g_PNH2612.S.AbsoluteDifference = g_PNH2612.S.AbsoluteBuffer[0] - g_PNH2612.S.AbsoluteBuffer[1];

    // 处理M路溢出（16位有符号数）
    if(g_PNH2612.M.AbsoluteDifference > 16384) {
        g_PNH2612.M.AbsoluteDifference -= 32768;
    } else if(g_PNH2612.M.AbsoluteDifference < -16384) {
        g_PNH2612.M.AbsoluteDifference += 32768;
    }

    // 处理N路溢出
    if(g_PNH2612.N.AbsoluteDifference > 16384) {
        g_PNH2612.N.AbsoluteDifference -= 32768;
    } else if(g_PNH2612.N.AbsoluteDifference < -16384) {
        g_PNH2612.N.AbsoluteDifference += 32768;
    }

    // 处理S路溢出
    if(g_PNH2612.S.AbsoluteDifference > 16384) {
        g_PNH2612.S.AbsoluteDifference -= 32768;
    } else if(g_PNH2612.S.AbsoluteDifference < -16384) {
        g_PNH2612.S.AbsoluteDifference += 32768;
    }

    // 计算M路速度 (RPM) - 固定采样周期为10ms
    g_PNH2612.M.Speed = (g_PNH2612.M.AbsoluteDifference * 6000) >> 16; // 60*1000/65536≈0.9155
    g_PNH2612.N.Speed = (g_PNH2612.N.AbsoluteDifference * 6000) >> 16;
    g_PNH2612.S.Speed = (g_PNH2612.S.AbsoluteDifference * 6000) >> 16;

    // 速度检查（50-70 RPM为正常范围）
    if(g_PNH2612.SamplingCnt < 16000) {
        g_PNH2612.SamplingCnt++;
        
        // M路速度检查
        if(g_PNH2612.M.SpeedFlag == 0) {
            if(g_PNH2612.M.Speed > 70 || g_PNH2612.M.Speed < 50) {
                g_PNH2612.M.SpeedResult = 2; // 速度异常
                g_PNH2612.M.FeedbackSpeed = g_PNH2612.M.Speed;
                g_PNH2612.M.SpeedFlag = 1;   // 标记已检测到异常
            } else {
                g_PNH2612.M.SpeedResult = 1; // 速度正常
                g_PNH2612.M.FeedbackSpeed = g_PNH2612.M.Speed;
            }
        }

        // N路速度检查
        if(g_PNH2612.N.SpeedFlag == 0) {
            if(g_PNH2612.N.Speed > 70 || g_PNH2612.N.Speed < 50) {
                g_PNH2612.N.SpeedResult = 2;
                g_PNH2612.N.FeedbackSpeed = g_PNH2612.N.Speed;
                g_PNH2612.N.SpeedFlag = 1;
            } else {
                g_PNH2612.N.SpeedResult = 1;
                g_PNH2612.N.FeedbackSpeed = g_PNH2612.N.Speed;
            }
        }

        // S路速度检查
        if(g_PNH2612.S.SpeedFlag == 0) {
            if(g_PNH2612.S.Speed > 70 || g_PNH2612.S.Speed < 50) {
                g_PNH2612.S.SpeedResult = 2;
                g_PNH2612.S.FeedbackSpeed = g_PNH2612.S.Speed;
                g_PNH2612.S.SpeedFlag = 1;
            } else {
                g_PNH2612.S.SpeedResult = 1;
                g_PNH2612.S.FeedbackSpeed = g_PNH2612.S.Speed;
            }
        }
    }
}

// ================= M路位置标记检测 =================
static void MultiturnOpticalEncoderMPositionMark(void) {
    if(g_PNH2612.M.PositionFlag == 0) {
        g_PNH2612.MarkPositionBuffer[1] = g_PNH2612.MarkPositionBuffer[0];
        g_PNH2612.MarkPositionBuffer[0] = g_PNH2612.M.Absolute;
        g_PNH2612.MarkPositionDifference = g_PNH2612.MarkPositionBuffer[0] - g_PNH2612.MarkPositionBuffer[1];
        
        // 检测位置突变（标记点）
        if(g_PNH2612.MarkPositionDifference > 30000 || g_PNH2612.MarkPositionDifference < -30000) {
            g_PNH2612.M.PositionFlag = 1;
        }
    }
}

// ================= M路位置标记检测（阶段1） =================
static void MultiturnOpticalEncoderMPositionMark1(void) {
    if(g_PNH2612.M.PositionFlag == 1) {
        g_PNH2612.MarkPositionBuffer[1] = g_PNH2612.MarkPositionBuffer[0];
        g_PNH2612.MarkPositionBuffer[0] = g_PNH2612.M.Absolute;
        g_PNH2612.MarkPositionDifference = g_PNH2612.MarkPositionBuffer[0] - g_PNH2612.MarkPositionBuffer[1];
        
        // 检测位置突变（标记点）
        if(g_PNH2612.MarkPositionDifference > 30000 || g_PNH2612.MarkPositionDifference < -30000) {
            g_PNH2612.M.PositionFlag = 2;
        }
    }
}

// ================= DAC自动调整 =================
static void MultiturnOpticalEncoderDACAdjustment(void) {
    if(g_MulOpt.Flag.bit.LedDacAdjust) {
        if(g_PNH2612.LEDAdjustDACCnt < 20) {
            if(g_PNH2612.LEDAdjustDACCnt == 0) {
                g_PNH2612.LEDAdjustDAC = 1950;
                g_PNH2612.LEDAdjustDACData = -50;
            } else {
                if(g_PNH2612.M.SinData < 1800) {
                    g_PNH2612.LEDAdjustDACData = 50;
                } else if(g_PNH2612.M.SinData > 2200) {
                    g_PNH2612.LEDAdjustDACData = -50;
                }
                g_PNH2612.LEDAdjustDAC += g_PNH2612.LEDAdjustDACData;
            }
            
            g_PNH2612.LEDDAC0 = g_PNH2612.LEDAdjustDAC & 0xFF;
            g_PNH2612.LEDDAC1 = (g_PNH2612.LEDAdjustDAC >> 8) & 0xFF;
            MulOpt_TX(0xD5, 0x00, 0x00);
            g_PNH2612.LEDAdjustDACCnt++;
        } else {
            g_MulOpt.Flag.bit.LedDacAdjust = 0;
            g_PNH2612.LEDAdjustDACCnt = 0;
        }
    }
}

// ================= M/N/S模拟量最大值检查 =================
static void MultiturnOpticalEncoderMNSAnalogMaxCheck(void) {
    if(g_MulOpt.Flag.bit.AMD == 2) {
        if(g_PNH2612.MNSAnalogChekcCnt < 30) {
            g_PNH2612.MNSSinDataMaxBuffer[g_PNH2612.MNSAnalogChekcCnt] = g_PNH2612.M.SinData;
            g_PNH2612.MNSCosDataMaxBuffer[g_PNH2612.MNSAnalogChekcCnt] = g_PNH2612.M.CosData;
            g_PNH2612.MNSAnalogChekcCnt++;
        } else {
            uint16_t sin_max = 0;
            uint16_t cos_max = 0;
            
            // 查找最大值
            for(uint8_t i = 0; i < 30; i++) {
                if(g_PNH2612.MNSSinDataMaxBuffer[i] > sin_max) {
                    sin_max = g_PNH2612.MNSSinDataMaxBuffer[i];
                }
                if(g_PNH2612.MNSCosDataMaxBuffer[i] > cos_max) {
                    cos_max = g_PNH2612.MNSCosDataMaxBuffer[i];
                }
            }
            
            // 检查是否达到目标值
            if(sin_max > 2000 && cos_max > 2000) {
                g_PNH2612.MNSAnalogResult = 1;
            } else {
                g_PNH2612.MNSAnalogResult = 0;
            }
            
            g_MulOpt.Flag.bit.AMD = 0;
            g_PNH2612.MNSAnalogChekcCnt = 0;
        }
    }
}

// ================= Modbus地址检查 =================
uint8_t MulOpt_Modbus_IsMyAddr(uint16_t addr) {
    // 输入寄存器区 (0x0300-0x03FF)
    if(addr >= 0x0300 && addr <= 0x03FF) {
        return 1;
    }
    // 保持寄存器区 (0x0400-0x04FF)
    else if(addr >= 0x0400 && addr <= 0x04FF) {
        return 1;
    }
    return 0;
}

// ================= Modbus寄存器读取 =================
uint16_t MulOpt_Modbus_Read(uint16_t addr) {
    switch(addr) {
        // 状态寄存器
        case 0x0300: return g_MulOpt.MTAB.Sector;
        case 0x0301: return g_MulOpt.Hall.Sector;
        case 0x0302: return g_MulOpt.Status0x6D;
        case 0x0303: return g_MulOpt.Status.all;
        case 0x0304: return g_MulOpt.Alarm.all;
        
        // 位置寄存器
        case 0x0310: return (uint16_t)(g_MulOpt.SingleTurnPosition >> 16);
        case 0x0311: return (uint16_t)g_MulOpt.SingleTurnPosition;
        case 0x0312: return g_MulOpt.MultiTurnPosition;
        
        // 环境数据
        case 0x0320: return (uint16_t)g_MulOpt.Temperature + 100; // 转换为无符号
        case 0x0321: return g_MulOpt.BatteryVoltage;
        
        // M路数据
        case 0x0330: return g_PNH2612.M.SinData;
        case 0x0331: return g_PNH2612.M.CosData;
        case 0x0332: return g_PNH2612.M.Absolute;
        case 0x0333: return g_PNH2612.M.Speed;
        
        // N路数据
        case 0x0340: return g_PNH2612.N.SinData;
        case 0x0341: return g_PNH2612.N.CosData;
        case 0x0342: return g_PNH2612.N.Absolute;
        case 0x0343: return g_PNH2612.N.Speed;
        
        // S路数据
        case 0x0350: return g_PNH2612.S.SinData;
        case 0x0351: return g_PNH2612.S.CosData;
        case 0x0352: return g_PNH2612.S.Absolute;
        case 0x0353: return g_PNH2612.S.Speed;
        
        // 测试结果
        case 0x0360: return g_MulOpt.TestResult;
        case 0x0361: return g_PNH2612.MNSAnalogResult;
        case 0x0362: return g_PNH2612.MNSAnalogSynchronousCheckResult;
        case 0x0363: return g_PNH2612.LEDTestDACResult;
        
        default:
            Communication_Address_Error();
            return 0xFFFF;
    }
}

// ================= Modbus寄存器写入 =================
uint8_t MulOpt_Modbus_Write(uint16_t addr, uint16_t value) {
    switch(addr) {
        // 控制寄存器
        case 0x0400: 
            if(value == 1) {
                g_MotorEncoder.TestItem = MulOpt_Test_ReadAllEeprom;
            }
            return 0;
            
        case 0x0401: 
            g_MotorEncoder.TestItem = (uint8_t)value;
            return 0;
            
        case 0x0402: 
            g_PNH2612.LEDTestDAC = value;
            return 0;
            
        case 0x0403: 
            g_MulOpt.Eeprom.Page = (uint8_t)value;
            return 0;
            
        case 0x0404: 
            g_MulOpt.Eeprom.Address = (uint8_t)value;
            return 0;
            
        case 0x0405: 
            g_MulOpt.Eeprom.Data = (uint8_t)value;
            return 0;
            
        case 0x0406: 
            g_MulOpt.Flag.bit.LedDacAdjust = (value & 0x01);
            return 0;
            
        default:
            Communication_Address_Error();
            return 1;
    }
}
