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
#include <stdlib.h>

// ================= 全局变量定义 =================
MulOpt_t g_MulOpt = {0};
MulOpt_PNH2612_t g_PNH2612 = {0};

// ================= 私有函数声明 =================
static void MulOpt_MNSAnalogMaxSampling(void);
static void MulOpt_LedDac(void);
static void MulOpt_SectorCollect(void);
static void MulOpt_MTABContinuousCheck(void);
static void MulOpt_HallContinuousCheck(void);
static void MulOpt_MNSAnalogSynCheckResult(void);

/****************************************************************************************
* 函数名称：MulOpt_Init
* 函数功能：初始化多圈光编码器模块，清零状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
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

/****************************************************************************************
* 函数名称：MulOpt_CrcError
* 函数功能：编码器返回数据CRC计算错误处理
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulOpt_CrcError(void)
{
		g_MulOpt.Error.bit.EC = 1;
		Work_Alarm = WORK_ALARM_CRC_ERROR;
		g_MulOpt.CrcError = 1;
		g_MulOpt.RxDataCnt = 0;
		g_MulOpt.TimeoutCnt = 0;
}

/****************************************************************************************
* 函数名称：MulOpt_TX
* 函数功能：发送数据命令并启动DMA传输
* 输入参量：uint8_t cmd, uint8_t addr, uint8_t data
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
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
* 编写日期：2026-03-18
****************************************************************************************/
void MulOpt_RxComplete(void) {
    if (g_MulOpt.RxDataCnt < 1) return;

    uint8_t cmd = g_MulOpt.RxData[0];

    switch (cmd) {
        case 0x1A:  // 读取全部数据
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Status.all = g_MulOpt.RxData[1];
                g_MulOpt.SingleTurnPosition = ((uint32_t)g_MulOpt.RxData[4] << 16) | 
                                            ((uint32_t)g_MulOpt.RxData[3] << 8) | 
                                            g_MulOpt.RxData[2];
                g_MulOpt.ResolutionID = g_MulOpt.RxData[5];
                g_MulOpt.MultiTurnPosition = ((uint16_t)g_MulOpt.RxData[7] << 8) | g_MulOpt.RxData[6];
                g_MulOpt.Alarm.all = g_MulOpt.RxData[9];
            break;

        case 0x62:  // 多圈复位响应
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Status.all = g_MulOpt.RxData[1];
                g_MulOpt.SingleTurnPosition = ((uint32_t)g_MulOpt.RxData[4] << 16) | 
                                            ((uint32_t)g_MulOpt.RxData[3] << 8) | 
                                            g_MulOpt.RxData[2];
            break;

        case 0x25:  // Hall读取响应
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
								g_PNH2612.AbsolutePosition = g_MulOpt.RxData[1] | (g_MulOpt.RxData[2] << 8) | (g_MulOpt.RxData[3] << 16) | (g_MulOpt.RxData[4] << 24);
								g_PNH2612.HallSector = (g_MulOpt.RxData[5] & 0x30) >> 4;
								g_PNH2612.MTABSector = (g_MulOpt.RxData[5] & 0x03);
								g_PNH2612.MTABSectorraw = (g_MulOpt.RxData[5] & 0x0C) >> 2;								
                g_MulOpt.Hall.Sector = g_PNH2612.HallSector;
                g_MulOpt.MTAB.Sector = g_PNH2612.MTABSector;
            break;

        case 0x3D:  // 电池/温度响应
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
								g_PNH2612.LEDReadDAC = (g_MulOpt.RxData[1] | (g_MulOpt.RxData[2] << 8))& 0xFFFF;
								g_MulOpt.Temperature = g_MulOpt.RxData[3];
								g_MulOpt.BatteryVoltage = g_MulOpt.RxData[4];
            break;

        case 0x6D:  // 初始化响应
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
								switch(g_MulOpt.IPID){
								case Encoder_CED_Test:
									g_MulOpt.Status0x6D = g_MulOpt.RxData[4];//STATUS：0 清除失败 STATUS：1 清除成功 STATUS：2 清除无效，已清除一次，不能再清除 STATUS：4 指令无效            
									break;
								case Encoder_OIP_Test:
									g_MulOpt.OIPStatus = g_MulOpt.RxData[4];
									if(g_MulOpt.Cnt.CFOIP == 5){
										g_MotorEncoder.TestItem = MulOpt_Test_Stop;      
										g_MulOpt.Cnt.CFOIP = 0;
										if(g_MulOpt.OIPStatus == 0){

										}           
									}
									if(g_MulOpt.OIPStatus == 4){

									}            
									break;
								case Encoder_CIP_Test:
									
									break;            
								}
            break;

        case 0xEA:  // EEPROM单字节读取响应
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                
                g_MulOpt.Eeprom.ReturnAddress = g_MulOpt.RxData[1] & 0x7F;
                if (g_MulOpt.Eeprom.ReturnAddress == MULOPT_PNS_ADDRESS) {
                    g_MulOpt.Eeprom.ReturnPage = g_MulOpt.RxData[2];
										g_MulOpt.Eeprom.Status.bit.Busy = (g_MulOpt.RxData[1] >> 7) & 0xFF;
                    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.ReturnPage][g_MulOpt.Eeprom.ReturnAddress] = g_MulOpt.RxData[2];
                } else if (g_MulOpt.Eeprom.ReturnAddress < MULOPT_PNS_ADDRESS) {
										g_MulOpt.Eeprom.Status.bit.Busy = (g_MulOpt.RxData[1] >> 7) & 0xFF;
                    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.ReturnPage][g_MulOpt.Eeprom.ReturnAddress] = g_MulOpt.RxData[2];
                }
            break;

        case 0x32:  // EEPROM单字节读取响应(简化版)
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Eeprom.Address = g_MulOpt.RxData[1];
                g_MulOpt.Eeprom.Data = g_MulOpt.RxData[2];
            break;

        case 0x85:  // 读取3码道ADC数据响应
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
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
                MulOpt_MNSAnalogMaxSampling();
                MulOpt_SectorCollect();
								MulOpt_LedDac();
                MulOpt_MTABContinuousCheck();
                MulOpt_HallContinuousCheck();
                MulOpt_MNSAnalogSynCheckResult();
            break;

        case 0x0D:  // 读取3码道位置值响应
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_PNH2612.M.Absolute = g_MulOpt.RxData[1] | (g_MulOpt.RxData[2] << 8);
                g_PNH2612.N.Absolute = g_MulOpt.RxData[3] | (g_MulOpt.RxData[4] << 8);
                g_PNH2612.S.Absolute = g_MulOpt.RxData[5] | (g_MulOpt.RxData[6] << 8);
            break;

				case 0x45:  // 
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
            break;

        case 0xCD:  // r值归一化值
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
								g_PNH2612.RMSQUA = g_MulOpt.RxData[1];
								g_PNH2612.RNSQUA = g_MulOpt.RxData[2];
								g_PNH2612.RSSQUA = g_MulOpt.RxData[3];
            break;

        case 0xD5:  //设定led光强值(DAC值)
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
								g_PNH2612.LEDDAC0 = g_MulOpt.RxData[1];
								g_PNH2612.LEDDAC1 = g_MulOpt.RxData[2];  
								if(g_MulOpt.Flag.bit.LedDac){
									g_MotorEncoder.TestItem = MulOpt_Test_AnalogData;    
								}								
            break;
								
        case 0x15:  // 读取MN delta值及MT sector
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_PNH2612.MNdelta = g_MulOpt.RxData[1] | (g_MulOpt.RxData[2] << 8);
                g_PNH2612.MTABSector = g_MulOpt.RxData[3];
                g_PNH2612.Q4D = g_MulOpt.RxData[4];
                g_PNH2612.Q14D = g_MulOpt.RxData[5] | (g_MulOpt.RxData[6] << 8);
            break;

        case 0x5D:  //
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;						
            break;

        case 0xE5:  //
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
								g_PNH2612.Q4D = g_MulOpt.RxData[1];
								g_PNH2612.Q14D = (g_MulOpt.RxData[2] | (g_MulOpt.RxData[3] << 8))& 0xFFFF;								
            break;

        case 0x75:  // 读取内部报警
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
								g_MulOpt.InternalAlarm1 = g_MulOpt.RxData[1];
								g_MulOpt.InternalAlarm2 = g_MulOpt.RxData[2];
								g_MulOpt.InternalAlarm3 = g_MulOpt.RxData[3];
            break;

        case 0xFD:  // 
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
            break;
								
        case 0x9D:  //读取当前位置值 注1：ADC当前采样后，发送 注2:绝对位置为23bit时，ABS3全为0，ABS2的最高位为0；
                g_MulOpt.CrcData = Enc_CRC8(g_MulOpt.RxData, g_MulOpt.RxDataCnt - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[g_MulOpt.RxDataCnt - 1]) {
										MulOpt_CrcError();
                    return;
                }
								g_MulOpt.CrcError = 0;
                g_MulOpt.TimeoutCnt = 0;
                g_MulOpt.RxID = cmd;
                g_PNH2612.AbsolutePosition = g_MulOpt.RxData[1] | 
                                          (g_MulOpt.RxData[2] << 8) | 
                                          (g_MulOpt.RxData[3] << 16) | 
                                          (g_MulOpt.RxData[4] << 24);
            break;

        case 0xAD:  // 批量读EEPROM响应
                g_MulOpt.CrcData = Enc_CRC8((uint8_t *)g_MulOpt.RxData, (g_MulOpt.Eeprom.SetDNum + 5) - 1);
                if (g_MulOpt.CrcData != g_MulOpt.RxData[(g_MulOpt.Eeprom.SetDNum + 5) - 1]) {
                    MulOpt_CrcError();
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
								g_MulOpt.CrcError = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Eeprom.ReturnPage = g_MulOpt.RxData[1];
                g_MulOpt.Eeprom.ReturnAddress = g_MulOpt.RxData[2];
                g_MulOpt.Eeprom.ReturnDNum = g_MulOpt.RxData[3];

                for (uint8_t i = 0; i < g_MulOpt.Eeprom.ReturnDNum; i++) {
                    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.ReturnPage][g_MulOpt.Eeprom.ReturnAddress + i] = g_MulOpt.RxData[4 + i];
                }               
            break;

        case 0x35:  // 批量写EEPROM响应
                g_MulOpt.CrcData = Enc_CRC8((uint8_t *)g_MulOpt.RxData, (g_MulOpt.Eeprom.SetDNum + 5) - 1);
								if (g_MulOpt.CrcData != g_MulOpt.RxData[(g_MulOpt.Eeprom.SetDNum + 5) - 1]) {
                    MulOpt_CrcError();
                    return;
                }
                g_MulOpt.TimeoutCnt = 0;
								g_MulOpt.CrcError = 0;
                g_MulOpt.RxID = cmd;
                g_MulOpt.Eeprom.ReturnPage = g_MulOpt.RxData[1];
                g_MulOpt.Eeprom.ReturnAddress = g_MulOpt.RxData[2];
                g_MulOpt.Eeprom.ReturnDNum = g_MulOpt.RxData[3];

                for (uint8_t i = 0; i < g_MulOpt.Eeprom.ReturnDNum; i++) {
                    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.ReturnPage][g_MulOpt.Eeprom.ReturnAddress + i] = g_MulOpt.RxData[4 + i];
                }               
            break;

        default:
						g_MulOpt.RxDataCnt = 0;
            break;
    }

    
}


// ================= M/N/S信号极值检测 =================

/****************************************************************************************
* 函数名称：MNS_SinCos_MaxMin
* 函数功能：M/N/S信号极值检测
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
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

/****************************************************************************************
* 函数名称：MulOpt_MNSAnalogMaxSampling
* 函数功能：M/N/S信号极值检测采样处理
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
void MulOpt_MNSAnalogMaxSampling(void)
{
  if(g_MulOpt.Flag.bit.AMD == 1){
    MNS_SinCos_MaxMin();
  }else if(g_MulOpt.Flag.bit.AMD == 2){            
      if(g_MulOpt.Flag.bit.MNSAMDOneLapCheck == 1){
        if(g_PNH2612.MNSAnalogChekcCnt < 10000){//60rpm,8k频率，采样9k点，从第1000开始
          g_PNH2612.MNSAnalogChekcCnt ++;
          if(g_PNH2612.MNSAnalogChekcCnt > 1000){
            MNS_SinCos_MaxMin();
          }
        }else{
          g_PNH2612.MNSAnalogChekcCnt = 0;
          g_MulOpt.Flag.bit.MNSAMDOneLapCheck = 2;
          g_MotorEncoder.TestItem = MulOpt_Test_MNSAnalogMaxCheck;
        }                
      }else{
        MNS_SinCos_MaxMin();
        if(g_PNH2612.MNSAnalogChekcCnt < 2000){//60rpm,8k频率，四分之一圈就是2k
          g_PNH2612.MNSAnalogChekcCnt ++;
        }else{
          g_PNH2612.MNSAnalogChekcCnt = 0;
          g_MotorEncoder.TestItem = MulOpt_Test_MNSAnalogMaxCheck;
        }              
      }
    }  
}

// ================= LED DAC测试 =================

/****************************************************************************************
* 函数名称：MulOpt_LedDac
* 函数功能：LED DAC测试
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_LedDac(void) {
    if(g_MulOpt.Flag.bit.LedDac){
      g_MotorEncoder.TestItem = MulOpt_Test_LightIntensity;
      g_PNH2612.MSinDataBuffer[g_PNH2612.LEDTestDACCnt] = g_PNH2612.M.SinData;
      g_PNH2612.LEDTestDACCnt ++;
      if(g_PNH2612.LEDTestDACCnt >= 1){    
        if(g_PNH2612.LEDTestDACCnt > 1){
          if(g_PNH2612.LEDTestDACCnt == 6){
            g_PNH2612.LEDTestDACResult = 0x02;//2失败
            g_MulOpt.Flag.bit.LedDac = 0;
          }else{
            if(g_PNH2612.MSinDataBuffer[g_PNH2612.LEDTestDACCnt - 2] != g_PNH2612.MSinDataBuffer[g_PNH2612.LEDTestDACCnt - 1]){
              g_PNH2612.LEDTestDACResult = 0x01;//1成功
              g_MulOpt.Flag.bit.LedDac = 0;
            }else{
              g_MotorEncoder.TestItem = MulOpt_Test_LightIntensity;
            }
          }
        }else{
          g_MotorEncoder.TestItem = MulOpt_Test_LightIntensity;
        }
      }
    }
}

// ================= 扇区数据收集 =================

/****************************************************************************************
* 函数名称：MulOpt_SectorCollect
* 函数功能：扇区数据收集
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_SectorCollect(void) {
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

/****************************************************************************************
* 函数名称：MulOpt_MTABContinuousCheck
* 函数功能：MTAB连续检测
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MTABContinuousCheck(void) {
    // 利用 static 的默认初始化特性 (重启后为 0 或 指定值)
    static uint8_t prev_logic  = 0xFF;
    static uint8_t start_logic = 0xFF;
    static uint8_t change_cnt  = 0;
    static uint8_t circle_done = 0;

    uint8_t curr_phys, curr_logic, diff;

    // 如果一圈已经结束，停止更新，保持现场供上位机读取
    if(circle_done) return;

    // 1. 读取并映射
    curr_phys = g_PNH2612.MTABSector;
    curr_logic = MTAB_Map[curr_phys & 0x03];

    // 2. 上电第一次初始化
    if(prev_logic == 0xFF)
    {
        prev_logic  = curr_logic;
        start_logic = curr_logic;
        change_cnt  = 0;
        circle_done = 0;
        
        // 确保结构体变量在重启后被正确初始化
        g_MulOpt.MTAB.Result      = 0;
        g_MulOpt.MTAB.Check       = ENC_STATUS_TESTING;
        g_MulOpt.MTAB.Cnt         = 0;
        g_MulOpt.MTAB.GlitchCnt   = 0;
        g_MulOpt.MTAB.JitterCnt   = 0;
        g_MulOpt.MTAB.CountErrCnt = 0;
        return;
    }

    // 3. 无变化退出
    if(curr_logic == prev_logic) return;

    // 4. 计算差值
    diff = (curr_logic + 4 - prev_logic) & 0x03;

    // --- 异常检测与计数 ---

    if(diff == 2) 
    {
        // [异常1] 跨区突变
        g_MulOpt.MTAB.GlitchCnt++;
        g_MulOpt.MTAB.Check = ENC_STATUS_ERR_GLITCH; // 标记状态
        g_MulOpt.MTAB.Result = 0;
        
        // 注意：不 return，更新位置继续测，看后面还有没有错
        prev_logic = curr_logic; 
    }
    else if(diff != 1) 
    {
        // [异常2] 抖动/反转 (diff == 3)
        g_MulOpt.MTAB.JitterCnt++;
        g_MulOpt.MTAB.Check = ENC_STATUS_ERR_JITTER; // 标记状态
        g_MulOpt.MTAB.Result = 0;
        
        // 发生抖动时，不增加 change_cnt，但更新位置
        prev_logic = curr_logic;
    }
    else
    {
        // [正常] 正向旋转 (diff == 1)
        change_cnt++;
        prev_logic = curr_logic;
    }

    // 更新步数给上位机看
    g_MulOpt.MTAB.Cnt = change_cnt;

    // --- 结算逻辑 ---
    // 条件：回到起点 且 至少动了4步
    if((curr_logic == start_logic) && (change_cnt >= 4))
    {
        circle_done = 1; // 标记测试结束

        // 检查步数是否严格为4
        if(change_cnt != 4)
        {
            // [异常3] 步数异常
            g_MulOpt.MTAB.CountErrCnt++;
            g_MulOpt.MTAB.Check = ENC_STATUS_ERR_COUNT;
            g_MulOpt.MTAB.Result = 0;
        }

        // 最终综合判定
        // 只有：无突变、无抖动、且步数正好为4，才算 PASS
        if( (g_MulOpt.MTAB.GlitchCnt == 0) &&
            (g_MulOpt.MTAB.JitterCnt == 0) &&
            (g_MulOpt.MTAB.CountErrCnt == 0) ) // 这里的CntErrCnt等价于change_cnt==4判断
        {
            g_MulOpt.MTAB.Result = 1;
            g_MulOpt.MTAB.Check  = ENC_STATUS_PASS;
        }
        else
        {
            g_MulOpt.MTAB.Result = 0;
            // Check 保持最后一次检测到的错误状态
        }
    }
}

// ================= HALL连续检测 =================

/****************************************************************************************
* 函数名称：MulOpt_HallContinuousCheck
* 函数功能：HALL连续检测
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_HallContinuousCheck(void) {
    static uint8_t prev_logic  = 0xFF;
    static uint8_t start_logic = 0xFF;
    static uint8_t change_cnt  = 0;
    static uint8_t circle_done = 0;

    uint8_t curr_phys, curr_logic, diff;

    if(circle_done) return;

    curr_phys = g_PNH2612.HallSector;
    curr_logic = HALL_Map[curr_phys & 0x03];

    if(prev_logic == 0xFF)
    {
        prev_logic  = curr_logic;
        start_logic = curr_logic;
        change_cnt  = 0;
        circle_done = 0;

        g_MulOpt.Hall.Result      = 0;
        g_MulOpt.Hall.Check       = ENC_STATUS_TESTING;
        g_MulOpt.Hall.Cnt         = 0;
        g_MulOpt.Hall.GlitchCnt   = 0;
        g_MulOpt.Hall.JitterCnt   = 0;
        g_MulOpt.Hall.CountErrCnt = 0;
        return;
    }

    if(curr_logic == prev_logic) return;

    diff = (curr_logic + 4 - prev_logic) & 0x03;

    // --- 异常检测 ---

    if(diff == 2)
    {
        g_MulOpt.Hall.GlitchCnt++;
        g_MulOpt.Hall.Check = ENC_STATUS_ERR_GLITCH;
        g_MulOpt.Hall.Result = 0;
        prev_logic = curr_logic;
    }
    else if(diff != 1)
    {
        g_MulOpt.Hall.JitterCnt++;
        g_MulOpt.Hall.Check = ENC_STATUS_ERR_JITTER;
        g_MulOpt.Hall.Result = 0;
        prev_logic = curr_logic;
    }
    else
    {
        change_cnt++;
        prev_logic = curr_logic;
    }

    g_MulOpt.Hall.Cnt = change_cnt;

    // --- 结算 ---
    if((curr_logic == start_logic) && (change_cnt >= 4))
    {
        circle_done = 1;

        if(change_cnt != 4)
        {
            g_MulOpt.Hall.CountErrCnt++;
            g_MulOpt.Hall.Check = ENC_STATUS_ERR_COUNT;
            g_MulOpt.Hall.Result = 0;
        }

        if( (g_MulOpt.Hall.GlitchCnt == 0) &&
            (g_MulOpt.Hall.JitterCnt == 0) &&
            (g_MulOpt.Hall.CountErrCnt == 0) )
        {
            g_MulOpt.Hall.Result = 1;
            g_MulOpt.Hall.Check  = ENC_STATUS_PASS;
        }
        else
        {
            g_MulOpt.Hall.Result = 0;
        }
    }
}

// ================= MNS模拟量同步检查 =================

/****************************************************************************************
* 函数名称：MulOpt_MNSAnalogSynCheckResult
* 函数功能：MNS模拟量同步检查
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MNSAnalogSynCheckResult(void) {
  // 1. 定义上下限，方便修改 (假设是 12位 ADC, 中点 2048)
  const uint16_t LOWER_LIMIT = 2000;
  const uint16_t UPPER_LIMIT = 2100;

  // 2. 使用临时变量提取结果，避免 if 语句过长
  // 检查 M 组
  uint8_t m_ok = (g_PNH2612.M.SinData > LOWER_LIMIT && g_PNH2612.M.SinData < UPPER_LIMIT) &&
                 (g_PNH2612.M.CosData > LOWER_LIMIT && g_PNH2612.M.CosData < UPPER_LIMIT);
                 
  // 检查 N 组
  uint8_t n_ok = (g_PNH2612.N.SinData > LOWER_LIMIT && g_PNH2612.N.SinData < UPPER_LIMIT) &&
                 (g_PNH2612.N.CosData > LOWER_LIMIT && g_PNH2612.N.CosData < UPPER_LIMIT);
                 
  // 检查 S 组
  uint8_t s_ok = (g_PNH2612.S.SinData > LOWER_LIMIT && g_PNH2612.S.SinData < UPPER_LIMIT) &&
                 (g_PNH2612.S.CosData > LOWER_LIMIT && g_PNH2612.S.CosData < UPPER_LIMIT);

  // 3. 综合判断并赋值 (解决之前提到的 latch 问题)
  if (m_ok && n_ok && s_ok)
  {
    g_PNH2612.MNSAnalogSynchronousCheckResult = 1;
  }
  else
  {
    // 必须有 else，否则一旦置1就无法自动恢复为0
    g_PNH2612.MNSAnalogSynchronousCheckResult = 0;
  }
}

/****************************************************************************************
* 函数名称：MulOpt_ReadHWRev
* 函数功能：从EEPROM主缓存解析硬件版本号
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
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
* 函数功能：从EEPROM主缓存解析软件版本号
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
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




// ================= HALL检查 =================

/****************************************************************************************
* 函数名称：MulOpt_HallCheck
* 函数功能：HALL检查
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_HallCheck(void) {
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

/****************************************************************************************
* 函数名称：MulOpt_MTABCheck
* 函数功能：MTAB检查
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MTABCheck(void) {
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

/****************************************************************************************
* 函数名称：MulOpt_MNSSpeedCheck
* 函数功能：M/N/S速度检查
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MNSSpeedCheck(void) {
		g_PNH2612.M.AbsoluteBuffer[1] = g_PNH2612.M.AbsoluteBuffer[0];
		g_PNH2612.M.AbsoluteBuffer[0] = g_PNH2612.M.Absolute;
		g_PNH2612.M.AbsoluteDifference = g_PNH2612.M.AbsoluteBuffer[0] - g_PNH2612.M.AbsoluteBuffer[1];
		
		g_PNH2612.N.AbsoluteBuffer[1] = g_PNH2612.N.AbsoluteBuffer[0];
		g_PNH2612.N.AbsoluteBuffer[0] = g_PNH2612.N.Absolute;
		g_PNH2612.N.AbsoluteDifference = g_PNH2612.N.AbsoluteBuffer[0] - g_PNH2612.N.AbsoluteBuffer[1];
		
		g_PNH2612.S.AbsoluteBuffer[1] = g_PNH2612.S.AbsoluteBuffer[0];
		g_PNH2612.S.AbsoluteBuffer[0] = g_PNH2612.S.Absolute; 
		g_PNH2612.S.AbsoluteDifference = g_PNH2612.S.AbsoluteBuffer[0] - g_PNH2612.S.AbsoluteBuffer[1];
		
		if(g_PNH2612.M.AbsoluteDifference > 16384){
			g_PNH2612.M.AbsoluteDifference -= 32768;
		}else if(g_PNH2612.M.AbsoluteDifference < -16384){
			g_PNH2612.M.AbsoluteDifference += 32768;
		}
		if(g_PNH2612.N.AbsoluteDifference > 16384){
			g_PNH2612.N.AbsoluteDifference -= 32768;
		}else if(g_PNH2612.N.AbsoluteDifference < -16384){
			g_PNH2612.N.AbsoluteDifference += 32768;
		}
		if(g_PNH2612.S.AbsoluteDifference > 16384){
			g_PNH2612.S.AbsoluteDifference -= 32768;
		}else if(g_PNH2612.S.AbsoluteDifference < -16384){
			g_PNH2612.S.AbsoluteDifference += 32768;
		}  
		
	//  g_PNH2612.M.Speed = ((g_PNH2612.M.AbsoluteDifference * 60 * 8000) >> 15) >> 9;
		g_PNH2612.M.Speed = (g_PNH2612.M.AbsoluteDifference * 1875) >> 16;
		g_PNH2612.N.Speed = (g_PNH2612.M.AbsoluteDifference * 1875) >> 16;
		g_PNH2612.S.Speed = (g_PNH2612.M.AbsoluteDifference * 1875) >> 16;
		
		if(g_PNH2612.SamplingCnt < 16000){
			g_PNH2612.SamplingCnt ++;
			if(g_PNH2612.SamplingCnt > 2){
				if(g_PNH2612.M.SpeedFlag == 0){
					if(g_PNH2612.M.Speed > 70 || g_PNH2612.M.Speed < 50){
						g_PNH2612.M.SpeedResult = 2;
						g_PNH2612.M.FeedbackSpeed = g_PNH2612.M.Speed;
						g_PNH2612.M.SpeedFlag = 1;
					}else{
						g_PNH2612.M.SpeedResult = 1;
						g_PNH2612.M.FeedbackSpeed = g_PNH2612.M.Speed;
					}    
				}

				if(g_PNH2612.N.SpeedFlag == 0){
					if(g_PNH2612.N.Speed > 70 || g_PNH2612.N.Speed < 50){
						g_PNH2612.N.SpeedResult = 2;
						g_PNH2612.N.FeedbackSpeed = g_PNH2612.N.Speed;
						g_PNH2612.N.SpeedFlag = 1;
					}else{
						g_PNH2612.N.SpeedResult = 1;
						g_PNH2612.N.FeedbackSpeed = g_PNH2612.N.Speed;
					}    
				}
				
				if(g_PNH2612.S.SpeedFlag == 0){
					if(g_PNH2612.S.Speed > 70 || g_PNH2612.S.Speed < 50){
						g_PNH2612.S.SpeedResult = 2;
						g_PNH2612.S.FeedbackSpeed = g_PNH2612.S.Speed;
						g_PNH2612.N.SpeedFlag = 1;
					}else{
						g_PNH2612.S.SpeedResult = 1;
						g_PNH2612.S.FeedbackSpeed = g_PNH2612.S.Speed;
					}    
				}
			}
		}
}

// ================= M路位置标记检测 =================

/****************************************************************************************
* 函数名称：MulOpt_MPositionMark
* 函数功能：M路位置标记检测
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MPositionMark(void) {
		if(g_PNH2612.M.PositionFlag == 1){
			g_PNH2612.M.MarkPositionBuffer[1] = g_PNH2612.M.MarkPositionBuffer[0];  
			g_PNH2612.M.MarkPositionBuffer[0] = g_PNH2612.M.Absolute;   
			g_Motor.FctPos_MPos_Diff[0] = (g_Motor.FctPosition >> 9) - (g_PNH2612.M.Absolute >> 1);
			g_PNH2612.M.PositionFlag = 2;
		}else if(g_PNH2612.M.PositionFlag == 3){
			g_PNH2612.M.MarkPositionBuffer[1] = g_PNH2612.M.MarkPositionBuffer[0];  
			g_PNH2612.M.MarkPositionBuffer[0] = g_PNH2612.M.Absolute;   
			g_PNH2612.M.MarkPositionDifference = abs(g_PNH2612.M.MarkPositionBuffer[1] - g_PNH2612.M.MarkPositionBuffer[0]);
			g_Motor.FctPos_MPos_Diff[1] = ((g_Motor.FctPosition - 8388608) >> 9) - (g_PNH2612.M.Absolute >> 1);
			g_PNH2612.M.PositionFlag = 0;
		} 
}

// ================= M路位置标记检测（阶段1） =================

/****************************************************************************************
* 函数名称：MulOpt_MPositionMark1
* 函数功能：M路位置标记检测（阶段1）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MPositionMark1(void) {
		static uint8_t i = 0;
		if(g_PNH2612.M.PositionFlag == 1){
			if(i < 2){
				i ++;      
			}else{
				g_PNH2612.M.PositionFlag = 2; 
			}
			g_Motor.FctPos_MPos_Diff[0] = g_Motor.FctPosition - g_MulOpt.SingleTurnPosition;
		}else if(g_PNH2612.M.PositionFlag == 3){
			g_Motor.FctPos_MPos_Diff[1] = g_Motor.FctPosition - 8388608 - g_MulOpt.SingleTurnPosition;
			g_PNH2612.M.PositionFlag = 0;
			g_MotorEncoder.TestItem = MulOpt_Test_Stop;
		} 
}

// ================= DAC自动调整 =================

/****************************************************************************************
* 函数名称：MulOpt_DACAdjustment
* 函数功能：DAC自动调整
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_DACAdjustment(void) {
		if(g_MulOpt.Flag.bit.LedDacAdjust){
			if(g_PNH2612.LEDAdjustDACCnt < 10){
				g_PNH2612.LEDAdjustDAC = (1872 + (g_PNH2612.LEDAdjustDACData * g_PNH2612.LEDAdjustDACCnt));//默认DAC是1922
				g_PNH2612.LEDAdjustDACCnt ++;    
				g_PNH2612.LEDDAC0 = g_PNH2612.LEDAdjustDAC & 0xFF;
				g_PNH2612.LEDDAC1 = (g_PNH2612.LEDAdjustDAC >> 8) & 0xFF;
				MulOpt_TX(0xD5, 0x00, 0x00); 
				g_MulOpt.Flag.bit.LedDacAdjust = 0;
			}else{
				g_PNH2612.MNSAnalogResult = 3;//代表调节不成功
				//g_PNH2612.LEDAdjustDACCnt = 0;
				g_MotorEncoder.TestItem = MulOpt_Test_Stop;
				return;
			}
		}
		if(g_PNH2612.MNSAnalogWaitCnt < 255){
			g_PNH2612.MNSAnalogWaitCnt ++;
		}else{
			g_PNH2612.MNSAnalogWaitCnt = 0;
			g_MotorEncoder.TestItem = MulOpt_Test_AnalogData;
		}
}

/****************************************************************************************
* 函数名称：MulOpt_AnalogReset
* 函数功能：M/N/S模拟量归零
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
void MulOpt_AnalogReset(void)
{
  g_PNH2612.M.SinDataMax = 0;
  g_PNH2612.M.CosDataMax = 0;
  g_PNH2612.N.SinDataMax = 0;
  g_PNH2612.N.CosDataMax = 0;
  g_PNH2612.S.SinDataMax = 0;
  g_PNH2612.S.CosDataMax = 0;
}

/****************************************************************************************
* 函数名称：MulOpt_MNSAnalogMaxCheck
* 函数功能：M/N/S模拟量最大值检查
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MNSAnalogMaxCheck(void) {
		if(
		// 判断所有变量是否在3100到3800之间
			(g_PNH2612.M.SinDataMax >= 3100 && g_PNH2612.M.SinDataMax <= 3800) &&
			(g_PNH2612.M.CosDataMax >= 3100 && g_PNH2612.M.CosDataMax <= 3800) &&
			(g_PNH2612.N.SinDataMax >= 3100 && g_PNH2612.N.SinDataMax <= 3800) &&
			(g_PNH2612.N.CosDataMax >= 3100 && g_PNH2612.N.CosDataMax <= 3800) &&
			(g_PNH2612.S.SinDataMax >= 3100 && g_PNH2612.S.SinDataMax <= 3800) &&
			(g_PNH2612.S.CosDataMax >= 3100 && g_PNH2612.S.CosDataMax <= 3800) &&
			// 判断差值的绝对值是否小于100
			(abs(g_PNH2612.M.SinDataMax - g_PNH2612.M.CosDataMax) < 100) &&
			(abs(g_PNH2612.N.SinDataMax - g_PNH2612.N.CosDataMax) < 100) &&
			(abs(g_PNH2612.S.SinDataMax - g_PNH2612.S.CosDataMax) < 100)     
		){
			if(g_PNH2612.LEDAdjustDACCnt){//代表是调节后才达到，则需要再进行转一圈判断max值
				if(g_MulOpt.Flag.bit.MNSAMDOneLapCheck == 2){
					g_PNH2612.MNSAnalogResult = 4;//代表调节后成功
					g_MotorEncoder.TestItem = MulOpt_Test_Stop;
				}else{
					MulOpt_AnalogReset();
					g_MulOpt.Flag.bit.MNSAMDOneLapCheck = 1;
					g_MotorEncoder.TestItem = MulOpt_Test_AnalogData;
				}
			}else{
				g_PNH2612.MNSAnalogResult = 1;//代表一次成功
				g_MotorEncoder.TestItem = MulOpt_Test_Stop;
			}
		}else{
			if(
				 (g_PNH2612.M.SinDataMax < 3100 || g_PNH2612.M.CosDataMax < 3100) ||
				 (g_PNH2612.N.SinDataMax < 3100 || g_PNH2612.N.CosDataMax < 3100) ||
				 (g_PNH2612.S.SinDataMax < 3100 || g_PNH2612.S.CosDataMax < 3100)        
			){
				g_PNH2612.LEDAdjustDACData = 50;
			}else if(
				 (g_PNH2612.M.SinDataMax > 3800 || g_PNH2612.M.CosDataMax > 3800) ||
				 (g_PNH2612.N.SinDataMax > 3800 || g_PNH2612.N.CosDataMax > 3800) ||
				 (g_PNH2612.S.SinDataMax > 3800 || g_PNH2612.S.CosDataMax > 3800)               
			){
				g_PNH2612.LEDAdjustDACData = -50;
			}
			g_PNH2612.MNSAnalogResult = 2;//代表调节中
			MulOpt_AnalogReset();
			g_MulOpt.Flag.bit.LedDacAdjust = 1;
			g_MotorEncoder.TestItem = MulOpt_Test_DACAdjustment;
		}
	//  if(g_PNH2612.MNSAnalogResult == 2){
	//    MulOpt_AnalogReset();
	//    g_MulOpt.Flag.bit.LedDacAdjust = 1;
	//    g_MotorEncoder.TestItem = MultiturnOpticalEncoderDACAdjustmentTest;
	//  } 
}

/****************************************************************************************
* 函数名称：MulOpt_Modbus_IsMyAddr
* 函数功能：Modbus地址检查
* 输入参量：uint16_t addr
* 输出参量：返回值
* 编写日期：2026-03-18
****************************************************************************************/
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

/****************************************************************************************
* 函数名称：MulOpt_MultiturnReset
* 函数功能：多圈复位操作
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MultiturnReset(void) {
    if(g_MulOpt.Cnt.CF62 < 10) {
        MulOpt_TX(0x62, 0x00, 0x00);
        g_MulOpt.Cnt.CF62++;
        if(g_MulOpt.Cnt.CF62 == 10) {
            g_MotorEncoder.TestItem = MulOpt_Test_Stop;
        }
    }
}

/****************************************************************************************
* 函数名称：MulOpt_GetAllData
* 函数功能：获取编码器全部数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_GetAllData(void) {
    MulOpt_TX(0x1A, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_ReadAllEeprom
* 函数功能：读取编码器全部 EEPROM 数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_ReadAllEeprom(void) {
    if(g_MulOpt.Eeprom.SetPage < MULOPT_PAGE_NUMBER) {
        g_MulOpt.Eeprom.SetDNum = 0x80;
        g_MulOpt.Eeprom.SetAddress = 0x00;
        MulOpt_TX(0xAD, 0x00, 0x00);
        g_MulOpt.Eeprom.SetPage++;
    } else {
        MulOpt_ReadHWRev();
        MulOpt_ReadFWRev();
        g_MotorEncoder.TestItem = MulOpt_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_Initialize
* 函数功能：编码器初始化流程
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_Initialize(void) {
    if(g_MulOpt.Cnt.CFCED < 5) {							
        if(g_MulOpt.Cnt.CFCED == 4) {
            EncDrv_SetRxWaitDelay(1, ENC_UNIT_S);
        }
        MulOpt_TX(0x6D, 0x00, Encoder_CED_Test);
        g_MulOpt.Cnt.CFCED++;
    } else {
        g_MulOpt.Cnt.CFCED = 0;
        g_MotorEncoder.TestItem = MulOpt_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_AnalogData
* 函数功能：读取模拟量数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_AnalogData(void) {
    MulOpt_TX(0x85, 0x00, 0x00);
}

/****************************************************************************************
* 函数名称：MulOpt_WriteResult
* 函数功能：写入测试结果
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_WriteResult(void) {
    g_MulOpt.Eeprom.SetPage = 0x02;
    g_MulOpt.Eeprom.SetAddress = 0x00;
    g_MulOpt.Eeprom.SetDNum = 0x01;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress] = g_MulOpt.TestResult;
    MulOpt_TX(0x35, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_TB
* 函数功能：读取电池电压/温度数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_TB(void) {
    MulOpt_TX(0x3D, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_OpenInternalProtocol
* 函数功能：打开内部协议
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_OpenInternalProtocol(void) {
    if(g_MulOpt.Cnt.CFOIP < 5) {
        MulOpt_TX(0x6D, 0x00, Encoder_OIP_Test);
        g_MulOpt.Cnt.CFOIP++;
    } else {
        g_MulOpt.Cnt.CFOIP = 0;
        g_MotorEncoder.TestItem = MulOpt_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_CloseInternalProtocol
* 函数功能：关闭内部协议
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_CloseInternalProtocol(void) {
    MulOpt_TX(0x6D, 0x00, Encoder_CIP_Test);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_WriteDate
* 函数功能：写入日期
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_WriteDate(void) {
    g_MulOpt.Eeprom.SetPage = 0x02;
    g_MulOpt.Eeprom.SetAddress = 0x68;
    g_MulOpt.Eeprom.SetDNum = 0x04;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress] = g_MotorEncoder.TestYear;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+1] = g_MotorEncoder.TestMoon;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+2] = g_MotorEncoder.TestDay;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+3] = g_MotorEncoder.TestHour;
    MulOpt_TX(0x35, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_DAC
* 函数功能：写入DAC值
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_DAC(void) {
    g_PNH2612.LEDDAC0 = g_PNH2612.LEDTestDAC & 0xFF;
    g_PNH2612.LEDDAC1 = (g_PNH2612.LEDTestDAC >> 8) & 0xFF;
    MulOpt_TX(0xD5, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_ReadAnalogData;
}

/****************************************************************************************
* 函数名称：MulOpt_LightIntensity
* 函数功能：光强检查
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_LightIntensity(void) {
    if(g_MulOpt.Flag.bit.LedDac){
        g_PNH2612.LEDTestDAC = (1950 - 50 * g_PNH2612.LEDTestDACCnt);
    }
    g_PNH2612.LEDDAC0 = g_PNH2612.LEDTestDAC & 0xFF;
    g_PNH2612.LEDDAC1 = (g_PNH2612.LEDTestDAC >> 8) & 0xFF;
    MulOpt_TX(0xD5, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_ReadAnalogData
* 函数功能：读取模拟量数据测试
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_ReadAnalogData(void) {
    MulOpt_TX(0x85, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_ReadInternalAlarm
* 函数功能：读取内部报警测试
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_ReadInternalAlarm(void) {
    MulOpt_TX(0x75, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_RMRNRSInitialize
* 函数功能：初始化RMRNRS参数
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_RMRNRSInitialize(void) {
    g_MulOpt.Eeprom.SetPage = 0x02;
    g_MulOpt.Eeprom.SetAddress = 0x58;
    g_MulOpt.Eeprom.SetDNum = 0x06;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress] = 90;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+1] = 0;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+2] = 90;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+3] = 0;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+4] = 90;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+5] = 0;
    MulOpt_TX(0x35, 0x00, 0x00); 
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_WriteHWRevTest
* 函数功能：写入硬件版本到EEPROM测试
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_WriteHWRevTest(void) {
    g_MulOpt.Eeprom.SetPage = 0x02;
    g_MulOpt.Eeprom.SetAddress = 0x70;
    g_MulOpt.Eeprom.SetDNum = 0x04;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress] = 'H';
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+1] = 'O';
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+2] = 1;
    g_MulOpt.Eeprom.DataBuffer[g_MulOpt.Eeprom.SetPage][g_MulOpt.Eeprom.SetAddress+3] = g_MotorEncoder.HWRevData;
    MulOpt_TX(0x35, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_SectorTest
* 函数功能：扇区测试
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_SectorTest(void) {
    MulOpt_HallCheck();
    MulOpt_MTABCheck();
    g_MotorEncoder.TestItem = MulOpt_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulOpt_MNSPositionTest
* 函数功能：MNS位置测试
* 输入参量：无
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
static void MulOpt_MNSPositionTest(void) {
    if(g_PNH2612.M.PositionFlag > 0) {
        MulOpt_TX(0x1A, 0x00, 0x00);
        MulOpt_MPositionMark1();
    } else {
        MulOpt_TX(0x0D, 0x00, 0x00);
        MulOpt_MNSSpeedCheck();
        MulOpt_MPositionMark();
    }
}

/****************************************************************************************
* 函数名称：MulOpt_Modbus_Read
* 函数功能：Modbus寄存器读取
* 输入参量：uint16_t addr
* 输出参量：返回值
* 编写日期：2026-03-18
****************************************************************************************/
uint16_t MulOpt_Modbus_Read(uint16_t addr) {
    switch(addr) {
        case 0x0300: return g_MulOpt.MTAB.Sector;
        case 0x0301: return g_MulOpt.Hall.Sector;
        case 0x0302: return g_MulOpt.Status0x6D;
        case 0x0303: return g_MulOpt.Eeprom.DataBuffer[2][0];
        case 0x0304: return g_MulOpt.Status.all;
        case 0x0305: return g_MulOpt.Alarm.all;
        case 0x0306: return g_MulOpt.Eeprom.DataBuffer[0x03][0x14];
        case 0x0307: return g_MulOpt.Eeprom.DataBuffer[0x03][0x15];
        case 0x0308: return g_MulOpt.Eeprom.DataBuffer[0x03][0x16];
        case 0x0309: return g_MulOpt.Eeprom.DataBuffer[0x03][0x17];
        case 0x030A: return g_MulOpt.BatteryVoltage;
        case 0x030B: return g_MulOpt.Temperature;
        case 0x030C: return g_MulOpt.Eeprom.DataBuffer[0x02][0x68];
        case 0x030D: return g_MulOpt.Eeprom.DataBuffer[0x02][0x69];
        case 0x030E: return g_MulOpt.Eeprom.DataBuffer[0x02][0x6A];
        case 0x030F: return g_MulOpt.Eeprom.DataBuffer[0x02][0x6B];
        case 0x0310: return g_PNH2612.M.SinDataMax;
        case 0x0311: return g_PNH2612.M.CosDataMax;
        case 0x0312: return g_PNH2612.N.SinDataMax;
        case 0x0313: return g_PNH2612.N.CosDataMax;
        case 0x0314: return g_PNH2612.S.SinDataMax;
        case 0x0315: return g_PNH2612.S.CosDataMax;
        case 0x0316: return g_PNH2612.M.SinData;
        case 0x0317: return g_PNH2612.M.CosData;
        case 0x0318: return g_PNH2612.N.SinData;
        case 0x0319: return g_PNH2612.N.CosData;
        case 0x031A: return g_PNH2612.S.SinData;
        case 0x031B: return g_PNH2612.S.CosData;
        case 0x031C: return g_PNH2612.LEDTestDACResult;
        case 0x031D: return g_MulOpt.InternalAlarm1;
        case 0x031E: return g_MulOpt.InternalAlarm2;
        case 0x031F: return g_MulOpt.InternalAlarm3;
        case 0x0320: 
            g_MulOpt.Hall.Sector = ((g_MulOpt.Hall.Buffer[0] << 12) | (g_MulOpt.Hall.Buffer[1] << 8) | (g_MulOpt.Hall.Buffer[2] << 4) | (g_MulOpt.Hall.Buffer[3])) & 0xFFFF;
            return g_MulOpt.Hall.Sector;
        case 0x0321: 
            g_MulOpt.MTAB.Sector = ((g_MulOpt.MTAB.Buffer[0] << 12) | (g_MulOpt.MTAB.Buffer[1] << 8) | (g_MulOpt.MTAB.Buffer[2] << 4) | (g_MulOpt.MTAB.Buffer[3])) & 0xFFFF;
            return g_MulOpt.MTAB.Sector;
        case 0x0322: return (g_MulOpt.Hall.Result << 1) | g_MulOpt.MTAB.Result;
        case 0x0323: return abs(g_PNH2612.M.SinData - g_PNH2612.M.CosData);
        case 0x0324: return abs(g_PNH2612.N.SinData - g_PNH2612.N.CosData);
        case 0x0325: return abs(g_PNH2612.S.SinData - g_PNH2612.S.CosData);
        case 0x0326: return g_PNH2612.M.FeedbackSpeed;
        case 0x0327: return g_PNH2612.N.FeedbackSpeed;
        case 0x0328: return g_PNH2612.S.FeedbackSpeed;
        case 0x0329: return g_PNH2612.M.MarkPositionDifference;
        case 0x032A: return (uint16_t)abs(g_Motor.FctPos_MPos_Diff[1] - g_Motor.FctPos_MPos_Diff[0]);
        case 0x032B: return g_PNH2612.MNSAnalogResult;
        case 0x032C: return g_PNH2612.LEDAdjustDAC;
        case 0x032D: return g_MulOpt.Hall.Check;
        case 0x032E: return g_MulOpt.Hall.GlitchCnt;
        case 0x032F: return g_MulOpt.Hall.JitterCnt;
        case 0x0330: return g_MulOpt.Hall.CountErrCnt;
        case 0x0331: return g_MulOpt.MTAB.Check;
        case 0x0332: return g_MulOpt.MTAB.GlitchCnt;
        case 0x0333: return g_MulOpt.MTAB.JitterCnt;
        case 0x0334: return g_MulOpt.MTAB.CountErrCnt;
        case 0x0335: return g_MulOpt.MTAB.CountErrCnt;
        case 0x0336: return g_PNH2612.MNSAnalogSynchronousCheckResult;
        default:
            Communication_Address_Error();
            return 0xFFFF;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_Modbus_Write
* 函数功能：Modbus寄存器写入
* 输入参量：uint16_t addr, uint16_t value
* 输出参量：返回值
* 编写日期：2026-03-18
****************************************************************************************/
uint8_t MulOpt_Modbus_Write(uint16_t addr, uint16_t value) {
    switch(addr) {
        case 0x0300:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_ReadAllEeprom;
            return 0;
        case 0x0301:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_Initialize;
            return 0;
        case 0x0302:
            if(value == 1) {
                g_MulOpt.Flag.bit.AMD = 1;
                g_MotorEncoder.TestItem = MulOpt_Test_AnalogData;
            }
            return 0;
        case 0x0303:
            if(value == 1 || value == 9) {
                g_MulOpt.TestResult = value;
                g_MotorEncoder.TestItem = MulOpt_Test_WriteResult;
            }
            return 0;
        case 0x0304:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_GetAllData;
            return 0;
        case 0x0305:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_MultiturnReset;
            return 0;
        case 0x0306:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_TB;
            return 0;
        case 0x0307:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_OIP;
            return 0;
        case 0x0308:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_CIP;
            return 0;
        case 0x0309:
            g_PNH2612.LEDTestDAC = value;
            g_MulOpt.Flag.bit.AMD = 0;
            g_MotorEncoder.TestItem = MulOpt_Test_DAC;
            return 0;
        case 0x030A:
            if(value == 1) {
                g_MulOpt.Flag.bit.LedDac = 1;
                g_MotorEncoder.TestItem = MulOpt_Test_LightIntensity;
            }
            return 0;
        case 0x030B:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_ReadInternalAlarm;
            return 0;
        case 0x030C:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_RMRNRSInitialize;
            return 0;
        case 0x030D:
            g_MotorEncoder.HWRevData = value;
            g_MotorEncoder.TestItem = MulOpt_Test_WriteHWRev;
            return 0;
        case 0x030E:
            g_MulOpt.Flag.bit.MTABHALL = value & 0xFF; /* bit is single/small bitfield, user code had value */
            return 0;
        case 0x030F:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_Sector;
            return 0;
        case 0x0310:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_MNSPosition;
            return 0;
        case 0x0311:
            if(value == 1) g_PNH2612.M.PositionFlag++;
            return 0;
        case 0x0312:
            if(value == 1) {
                g_MulOpt.Flag.bit.AMD = 2;
                g_MotorEncoder.TestItem = MulOpt_Test_MNSAnalogMaxCheck;
            }
            return 0;
        case 0x0313:
            if(value == 1) g_MotorEncoder.TestItem = MulOpt_Test_ReadAnalogData;
            return 0;
        default:
            Communication_Address_Error();
            return 1;
    }
}

/****************************************************************************************
* 函数名称：MulOpt_Test
* 函数功能：测试函数主入口，根据测试项目执行对应操作
* 输入参量：uint8_t testItem
* 输出参量：无
* 编写日期：2026-03-18
****************************************************************************************/
void MulOpt_Test(uint8_t testItem) {
    switch(testItem) {
        case MulOpt_Test_Stop:               break;
        case MulOpt_Test_MultiturnReset:     MulOpt_MultiturnReset(); break;
        case MulOpt_Test_GetAllData:         MulOpt_GetAllData(); break;
        case MulOpt_Test_ReadAllEeprom:      MulOpt_ReadAllEeprom(); break;
        case MulOpt_Test_Initialize:         MulOpt_Initialize(); break;
        case MulOpt_Test_AnalogData:         MulOpt_AnalogData(); break;
        case MulOpt_Test_WriteResult:        MulOpt_WriteResult(); break;
        case MulOpt_Test_TB:                 MulOpt_TB(); break;
        case MulOpt_Test_OIP:                MulOpt_OpenInternalProtocol(); break;
        case MulOpt_Test_CIP:                MulOpt_CloseInternalProtocol(); break;
        case MulOpt_Test_WriteDate:          MulOpt_WriteDate(); break;
        case MulOpt_Test_DAC:                MulOpt_DAC(); break;
        case MulOpt_Test_LightIntensity:     MulOpt_LightIntensity(); break;
        case MulOpt_Test_ReadAnalogData:     MulOpt_ReadAnalogData(); break;
        case MulOpt_Test_ReadInternalAlarm:  MulOpt_ReadInternalAlarm(); break;
        case MulOpt_Test_RMRNRSInitialize:   MulOpt_RMRNRSInitialize(); break;
        case MulOpt_Test_WriteHWRev:         MulOpt_WriteHWRevTest(); break;
        case MulOpt_Test_Sector:             MulOpt_SectorTest(); break;
        case MulOpt_Test_MNSPosition:        MulOpt_MNSPositionTest(); break;
        case MulOpt_Test_DACAdjustment:      MulOpt_DACAdjustment(); break;
        case MulOpt_Test_MNSAnalogMaxCheck:  MulOpt_MNSAnalogMaxCheck(); break;
        default: break;
    }
}
