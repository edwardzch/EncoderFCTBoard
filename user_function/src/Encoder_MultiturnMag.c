/****************************************************************************************
 * @file      Encoder_MultiturnMag.c
 * @brief     多圈磁编码器协议实现
 * @author    
 * @date      2026-02-09
 * @note      用于测试多圈磁编
 ****************************************************************************************/
#include "Encoder_MultiturnMag.h"
#include "encoder_driver.h"
#include "modbus_function.h"
#include <string.h>
#include <stdio.h>
#include "uart_config.h"
#include "encoder_modbus.h"
#include "Encoder_MGTMag.h"

// ================= 全局变量 =================
MulMag_t g_MulMag = {0};
MT6835_Addr_t g_MT6835Addr = {0};

static uint8_t s_WriteRegStep = 0;
static uint8_t s_ReadRegStep = 0;


/****************************************************************************************
* 函数名称：MulMag_GetWorkAlarm
* 函数功能：获取工作报警状态
* 输入参量：无
* 输出参量：报警状态值
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t MulMag_GetWorkAlarm(void)
{
    return Work_Alarm;
}

/****************************************************************************************
* 函数名称：MulMag_HasError
* 函数功能：检查是否存在通信错误
* 输入参量：无
* 输出参量：1=有错误，0=无错误
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t MulMag_HasError(void)
{
    return (g_MulMag.Error.bit.DC || g_MulMag.Error.bit.EC);
}

/****************************************************************************************
* 函数名称：MulMag_ClearError
* 函数功能：清除通信错误标志
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_ClearError(void)
{
    g_MulMag.Error.all = 0;
    Work_Alarm = 0;
}

/****************************************************************************************
* 函数名称：MulMag_Init
* 函数功能：初始化多圈磁编码器模块，清零状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_Init(void)
{
    memset(&g_MulMag, 0, sizeof(g_MulMag));
    memset(&g_MotorEncoder, 0, sizeof(g_MotorEncoder));
    memset(&g_MT6835Addr, 0, sizeof(g_MT6835Addr));
    g_MotorEncoder.TestItem = MulMag_Test_Stop;
    Work_Alarm = 0;
}

/****************************************************************************************
* 函数名称：MulMag_CrcError
* 函数功能：编码器返回数据CRC计算错误处理
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_CrcError(void)
{
		g_MulMag.Error.bit.EC = 1;
		Work_Alarm = WORK_ALARM_CRC_ERROR;
		g_MulMag.CrcError = 1;
		g_MulMag.RxDataCnt = 0;
		g_MulMag.TimeoutCnt = 0;
}

/****************************************************************************************
* 函数名称：MulMag_TX
* 函数功能：发送编码器命令，根据 cmd 类型组帧并启动 DMA 发送
* 输入参量：cmd 命令码，addr 地址，data 数据
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_TX(uint8_t cmd, uint16_t addr, uint8_t data)
{
    g_MulMag.TxID = cmd;
    g_MulMag.TxData[0] = cmd;
    
    switch (cmd) {
        case 0x1A:
            EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 1, 11);
            break;
        case 0x02:
        case 0x62:
        case 0x3D:
            EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 1, 6);
				    break;
        case 0x75:
            EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 1, 5);
            break;
        case 0x25:
            EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 1, 10);
            break;
            
        case 0xEA:
            g_MulMag.TxData[1] = (uint8_t)addr;
            g_MulMag.TxData[2] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 2);
            EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 3, 4);
            break;
            
        case 0x32:
            g_MulMag.TxData[1] = (uint8_t)addr;
            g_MulMag.TxData[2] = data;
            g_MulMag.TxData[3] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 3);
            EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 4, 4);
            break;
            
        case 0xAD:
            g_MulMag.TxData[1] = g_MulMag.Eeprom.SetPage;
            g_MulMag.TxData[2] = g_MulMag.Eeprom.SetAddress;
            g_MulMag.TxData[3] = g_MulMag.Eeprom.SetDNum;
            g_MulMag.TxData[4] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 4);
            EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 5, 5 + g_MulMag.Eeprom.SetDNum);
            break;
            
        case 0x35:
						g_MulMag.TxData[1] = g_MulMag.Eeprom.SetPage;
						g_MulMag.TxData[2] = g_MulMag.Eeprom.SetAddress;
						g_MulMag.TxData[3] = g_MulMag.Eeprom.SetDNum;
						uint8_t j;
						for (j = 0; j < g_MulMag.Eeprom.SetDNum; j++) {
								g_MulMag.TxData[4 + j] = g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress + j];
						}
						g_MulMag.TxData[4 + j] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 4 + j);
						EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 5 + g_MulMag.Eeprom.SetDNum, 5 + g_MulMag.Eeprom.SetDNum);
            break;
            
        case 0x6D:
            g_MulMag.IPID = data;
            if (data == Encoder_CED_Test) {
                g_MulMag.TxData[1] = 0x63;
                g_MulMag.TxData[2] = 0x65;
                g_MulMag.TxData[3] = 0x64;
            } else if (data == Encoder_CIP_Test) {
                g_MulMag.TxData[1] = 0x43;
                g_MulMag.TxData[2] = 0x49;
                g_MulMag.TxData[3] = 0x50;
            } else if(data == Encoder_OIP_Test){
                g_MulMag.TxData[1] = 0x4F;
                g_MulMag.TxData[2] = 0x49;
                g_MulMag.TxData[3] = 0x50;
            }
            g_MulMag.TxData[4] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 4);
            EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 5, 6);
            break;
            
        case 0x40:
            if (addr == 0x46) {
                g_MulMag.TxData[1] = 0x46;
                g_MulMag.TxData[2] = data;
                g_MulMag.TxData[3] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 3);
                EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 4, 4);
            } else if (addr == 0x45) {
                g_MulMag.TxData[1] = 0x45;
                g_MulMag.TxData[2] = 0x00;
                g_MulMag.TxData[3] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 2);
                EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 3, 4);
            }
            break;
            
        default:
            break;
    }
}

/****************************************************************************************
* 函数名称：MulMag_TxRegister
* 函数功能：发送寄存器读写命令
* 输入参量：cmd 命令码，status 读写状态，addr 寄存器地址，data 数据
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_TxRegister(uint8_t cmd, uint8_t status, uint8_t addr, uint8_t data)
{
    
    if (status == 0x51) {
        g_MulMag.TxData[0] = cmd;
        g_MulMag.TxData[1] = status;
        g_MulMag.TxData[2] = addr;
        g_MulMag.TxData[3] = 0x00;
        g_MulMag.TxData[4] = data;
        g_MulMag.TxData[5] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 5);
        EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 6, 6);
    } else if (status == 0x52) {
        g_MulMag.TxData[0] = cmd;
        g_MulMag.TxData[1] = status;
        g_MulMag.TxData[2] = addr;
        g_MulMag.TxData[3] = 0x00;
        g_MulMag.TxData[4] = Enc_CRC8((uint8_t *)g_MulMag.TxData, 4);
        EncDrv_SendDMA((uint8_t *)g_MulMag.TxData, 5, 6);
    }
}

/****************************************************************************************
* 函数名称：MulMag_HallCWCheck
* 函数功能：多圈磁编顺时针转动(CW) Hall检测 (FCT PCBA器件朝下)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-02
****************************************************************************************/
void MulMag_HallCWCheck(void)
{
    switch(g_MulMag.Hall.Buffer[0]) {
        case 0x00:
            if (g_MulMag.Hall.Buffer[1] == 3 && g_MulMag.Hall.Buffer[2] == 2 && g_MulMag.Hall.Buffer[3] == 1 && g_MotorEncoder.PitCnt >= 485 && g_MotorEncoder.PitCnt <= 800) {
                g_MulMag.Hall.Result = 1;
                g_MotorEncoder.TestItem = MulMag_Test_Stop;
            } else {
                g_MulMag.Hall.Result = 0;
            }
            break;
        case 0x01:
            if (g_MulMag.Hall.Buffer[1] == 0 && g_MulMag.Hall.Buffer[2] == 3 && g_MulMag.Hall.Buffer[3] == 2 && g_MotorEncoder.PitCnt >= 485 && g_MotorEncoder.PitCnt <= 800) {
                g_MulMag.Hall.Result = 1;
                g_MotorEncoder.TestItem = MulMag_Test_Stop;
            } else {
                g_MulMag.Hall.Result = 0;
            }        
            break;
        case 0x02:
            if (g_MulMag.Hall.Buffer[1] == 1 && g_MulMag.Hall.Buffer[2] == 0 && g_MulMag.Hall.Buffer[3] == 3 && g_MotorEncoder.PitCnt >= 485 && g_MotorEncoder.PitCnt <= 800) {
                g_MulMag.Hall.Result = 1;
                g_MotorEncoder.TestItem = MulMag_Test_Stop;
            } else {
                g_MulMag.Hall.Result = 0;
            }        
            break;
        case 0x03:
            if (g_MulMag.Hall.Buffer[1] == 2 && g_MulMag.Hall.Buffer[2] == 1 && g_MulMag.Hall.Buffer[3] == 0 && g_MotorEncoder.PitCnt >= 485 && g_MotorEncoder.PitCnt <= 800) {
                g_MulMag.Hall.Result = 1;
                g_MotorEncoder.TestItem = MulMag_Test_Stop;
            } else {
                g_MulMag.Hall.Result = 0;
            }        
            break;
    } 
}

/****************************************************************************************
* 函数名称：MulMag_HallCCWCheck
* 函数功能：多圈磁编逆时针转动(CCW) Hall检测 (FCT PCBA器件朝上)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-02
****************************************************************************************/
void MulMag_HallCCWCheck(void)
{
    switch(g_MulMag.Hall.Buffer[0]) {
        case 0x00:
            if (g_MulMag.Hall.Buffer[1] == 1 && g_MulMag.Hall.Buffer[2] == 2 && g_MulMag.Hall.Buffer[3] == 3 && g_MotorEncoder.PitCnt >= 485 && g_MotorEncoder.PitCnt <= 800) {
                g_MulMag.Hall.Result = 1;
                g_MotorEncoder.TestItem = MulMag_Test_Stop;
            } else {
                g_MulMag.Hall.Result = 0;
            }
            break;
        case 0x01:
            if (g_MulMag.Hall.Buffer[1] == 2 && g_MulMag.Hall.Buffer[2] == 3 && g_MulMag.Hall.Buffer[3] == 0 && g_MotorEncoder.PitCnt >= 485 && g_MotorEncoder.PitCnt <= 800) {
                g_MulMag.Hall.Result = 1;
                g_MotorEncoder.TestItem = MulMag_Test_Stop;
            } else {
                g_MulMag.Hall.Result = 0;
            }        
            break;
        case 0x02:
            if (g_MulMag.Hall.Buffer[1] == 3 && g_MulMag.Hall.Buffer[2] == 0 && g_MulMag.Hall.Buffer[3] == 1 && g_MotorEncoder.PitCnt >= 485 && g_MotorEncoder.PitCnt <= 800) {
                g_MulMag.Hall.Result = 1;
                g_MotorEncoder.TestItem = MulMag_Test_Stop;
            } else {
                g_MulMag.Hall.Result = 0;
            }        
            break;
        case 0x03:
            if (g_MulMag.Hall.Buffer[1] == 0 && g_MulMag.Hall.Buffer[2] == 1 && g_MulMag.Hall.Buffer[3] == 2 && g_MotorEncoder.PitCnt >= 485 && g_MotorEncoder.PitCnt <= 800) {
                g_MulMag.Hall.Result = 1;
                g_MotorEncoder.TestItem = MulMag_Test_Stop;
            } else {
                g_MulMag.Hall.Result = 0;
            }        
            break;
    } 
}

/****************************************************************************************
* 函数名称：MulMag_HallCheck
* 函数功能：多圈磁编 Hall 检测主入口，根据 EEPROM 配置决定检测 CW 还是 CCW
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-02
****************************************************************************************/
void MulMag_HallCheck(void)
{
    if (g_MulMag.Hall.Check) {
        if (g_MulMag.Eeprom.DataBuffer[3][20] == 3) {
            MulMag_HallCWCheck();
        } else if (g_MulMag.Eeprom.DataBuffer[3][20] == 2) {
            MulMag_HallCCWCheck();
        }
    } else {
        g_MotorEncoder.PitCnt++;
        if (g_MulMag.Hall.Cnt > 0) {
            if (g_MulMag.Hall.Sector != g_MulMag.Hall.Buffer[g_MulMag.Hall.Cnt - 1]) {
                g_MulMag.Hall.Buffer[g_MulMag.Hall.Cnt] = g_MulMag.Hall.Sector;
                g_MulMag.Hall.Cnt++;           
                if (g_MulMag.Hall.Cnt == 4) {
                    g_MulMag.Hall.Check = 1;
                }
            }  
        } else {
            g_MulMag.Hall.Buffer[g_MulMag.Hall.Cnt] = g_MulMag.Hall.Sector;
            g_MulMag.Hall.Cnt++;      
        }    
    }  
}

/****************************************************************************************
* 函数名称：MulMag_RxComplete
* 函数功能：接收完成处理，解析响应帧并更新编码器数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_RxComplete(void)
{
    if (g_MulMag.RxDataCnt < 1) return;
    
    uint8_t cmd = g_MulMag.RxData[0];
    
    switch (cmd) {
        case 0x1A:
            if (g_MulMag.RxDataCnt >= 11) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 10);
                if (g_MulMag.CrcData != g_MulMag.RxData[10]) {
										MulMag_CrcError();
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Status.all = g_MulMag.RxData[1];
                g_MulMag.SingleTurnPos = ((uint32_t)g_MulMag.RxData[4] << 16) | 
                                         ((uint32_t)g_MulMag.RxData[3] << 8) | 
                                         g_MulMag.RxData[2];
                g_MulMag.ResolutionID = g_MulMag.RxData[5];
                g_MulMag.MultiTurnPos = ((uint16_t)g_MulMag.RxData[7] << 8) | g_MulMag.RxData[6];
                g_MulMag.Alarm.all = g_MulMag.RxData[9];
                g_MulMag.CrcError = 0;
            }
            break;
            
        case 0x02:
        case 0x62:
            if (g_MulMag.RxDataCnt >= 6) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 5);
                if (g_MulMag.CrcData != g_MulMag.RxData[5]) {
                    MulMag_CrcError();
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Status.all = g_MulMag.RxData[1];
                g_MulMag.SingleTurnPos = ((uint32_t)g_MulMag.RxData[4] << 16) | 
                                         ((uint32_t)g_MulMag.RxData[3] << 8) | 
                                         g_MulMag.RxData[2];
                g_MulMag.CrcError = 0;
            }
            break;
						
            
        case 0x25:
            if (g_MulMag.RxDataCnt >= 10) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 9);
                if (g_MulMag.CrcData != g_MulMag.RxData[9]) {
                    MulMag_CrcError();
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.SingleTurnPos = ((uint32_t)g_MulMag.RxData[3] << 16) | 
                                         ((uint32_t)g_MulMag.RxData[2] << 8) | 
                                         g_MulMag.RxData[1];
								g_MulMag.SingleTurnPos = g_MulMag.SingleTurnPos >> 6;
								g_MulMag.Hall.Sector = (g_MulMag.RxData[5] & 0x30) >> 4;
                g_MulMag.CrcError = 0;
                
                // 执行转速和 Hall 的检测任务
                if (g_MotorEncoder.TestItem == MulMag_Test_Hall && g_MulMag.Cnt.CF25 == 100) {//一定要转速问题，前面100个周期不需要
                    MulMag_HallCheck();
                }
            }
            break;
            
        case 0x75:
            if (g_MulMag.RxDataCnt >= 5) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 4);
                if (g_MulMag.CrcData != g_MulMag.RxData[4]) {
                    MulMag_CrcError();
                    return;
                }							
                g_MulMag.TimeoutCnt = 0;
                
                g_MulMag.RxID = cmd;
                g_MulMag.InternalAlarm1 = g_MulMag.RxData[1];
                g_MulMag.InternalAlarm2 = g_MulMag.RxData[2];
                g_MulMag.InternalAlarm3 = g_MulMag.RxData[3];
                
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 4);

								g_MulMag.CrcError = 0;
            }
            break;
            
        case 0x3D:
            if (g_MulMag.RxDataCnt >= 6) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 5);
                if (g_MulMag.CrcData != g_MulMag.RxData[5]) {
                    MulMag_CrcError();
                    MulMag_CrcError();
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
								g_MulMag.Temperature = g_MulMag.RxData[3];
                g_MulMag.BatteryVoltage = g_MulMag.RxData[4];
                g_MulMag.CrcError = 0;
            }
            break;
            
        case 0x6D:
            if (g_MulMag.RxDataCnt >= 6) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 5);
                if (g_MulMag.CrcData != g_MulMag.RxData[5]) {
                    MulMag_CrcError();
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Status0x6D = g_MulMag.RxData[4];
								if(g_MulMag.Cnt.CFOIP == 5){
									
								}
								if(g_MulMag.Cnt.CFCED){
									
								}
                g_MulMag.CrcError = 0;
            }
            break;
            
        case 0xAD:
            if (g_MulMag.RxDataCnt >= (g_MulMag.Eeprom.SetDNum + 5)) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, (g_MulMag.Eeprom.SetDNum + 5) - 1);
                if (g_MulMag.CrcData != g_MulMag.RxData[(g_MulMag.Eeprom.SetDNum + 5) - 1]) {
                    MulMag_CrcError();
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Eeprom.ReturnPage = g_MulMag.RxData[1];
                g_MulMag.Eeprom.ReturnAddress = g_MulMag.RxData[2];
                g_MulMag.Eeprom.ReturnDNum = g_MulMag.RxData[3];

                for (uint8_t i = 0; i < g_MulMag.Eeprom.ReturnDNum; i++) {
                    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.ReturnPage][g_MulMag.Eeprom.ReturnAddress + i] = g_MulMag.RxData[4 + i];
                }
                g_MulMag.CrcError = 0;
            }
            break;

        case 0x35:
            if (g_MulMag.RxDataCnt >= (g_MulMag.Eeprom.SetDNum + 5)) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, (g_MulMag.Eeprom.SetDNum + 5) - 1);
								if (g_MulMag.CrcData != g_MulMag.RxData[(g_MulMag.Eeprom.SetDNum + 5) - 1]) {
                    MulMag_CrcError();
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Eeprom.ReturnPage = g_MulMag.RxData[1];
                g_MulMag.Eeprom.ReturnAddress = g_MulMag.RxData[2];
                g_MulMag.Eeprom.ReturnDNum = g_MulMag.RxData[3];

                for (uint8_t i = 0; i < g_MulMag.Eeprom.ReturnDNum; i++) {
                    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.ReturnPage][g_MulMag.Eeprom.ReturnAddress + i] = g_MulMag.RxData[4 + i];
                }
                g_MulMag.CrcError = 0;
            }
            break;

        case 0xEA:
            if (g_MulMag.RxDataCnt == 4) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 3);
                if (g_MulMag.CrcData != g_MulMag.RxData[3]) {
                    MulMag_CrcError();
                    return;
                }
                
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Eeprom.ReturnAddress = g_MulMag.RxData[1] & 0x7F;
                
                if (g_MulMag.Eeprom.ReturnAddress == MULENC_PNS_ADDR) {
                    g_MulMag.Eeprom.ReturnPage = g_MulMag.RxData[2];
                    g_MulMag.Eeprom.Status |= ((g_MulMag.RxData[1] >> 7) & 0x01); // Busy bit mapping equivalent
                    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.ReturnPage][g_MulMag.Eeprom.ReturnAddress] = g_MulMag.RxData[2];
                } else if (g_MulMag.Eeprom.ReturnAddress < MULENC_PNS_ADDR) {
                    g_MulMag.Eeprom.Status |= ((g_MulMag.RxData[1] >> 7) & 0x01);
                    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.ReturnPage][g_MulMag.Eeprom.ReturnAddress] = g_MulMag.RxData[2];
                }
                g_MulMag.CrcError = 0;
            }
            break;
            
        case 0x32:
            if (g_MulMag.RxDataCnt == 4) {
                g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 3);
                if (g_MulMag.CrcData != g_MulMag.RxData[3]) {
                    MulMag_CrcError();
                    return;
                }
                
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Eeprom.ReturnAddress = g_MulMag.RxData[1] & 0x7F;
                
                if (g_MulMag.Eeprom.ReturnAddress == MULENC_PNS_ADDR) {
                    g_MulMag.Eeprom.ReturnPage = g_MulMag.RxData[2];
                    g_MulMag.Eeprom.Status |= ((g_MulMag.RxData[1] >> 7) & 0x01); 
                    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.ReturnPage][g_MulMag.Eeprom.ReturnAddress] = g_MulMag.RxData[2];
                } else if (g_MulMag.Eeprom.ReturnAddress < MULENC_PNS_ADDR) {
                    g_MulMag.Eeprom.Status |= ((g_MulMag.RxData[1] >> 7) & 0x01);
                    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.ReturnPage][g_MulMag.Eeprom.ReturnAddress] = g_MulMag.RxData[2];
                }
                g_MulMag.CrcError = 0;
            }
            break;
            
        case 0x40:
            if (g_MulMag.RxDataCnt >= 3) {
                switch (g_MulMag.RxData[1]) {
                    case 0x45:
                        if (g_MulMag.RxDataCnt == 4) {
                            g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 3);
                            if (g_MulMag.CrcData != g_MulMag.RxData[3]) {
                                MulMag_CrcError();
																return;
                            }													
                            g_MulMag.TimeoutCnt = 0;
                            g_MulMag.RxDataCnt = 0;                            
                            g_MulMag.CrcError = 0;
														
														g_MulMag.ResolutionID = g_MulMag.RxData[2];
                        }
                        break;
                    case 0x46:
                        if (g_MulMag.RxDataCnt == 4) {
                            g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 3);
                            if (g_MulMag.CrcData != g_MulMag.RxData[3]) {
                                MulMag_CrcError();
																return;
                            }	
														
                            g_MulMag.TimeoutCnt = 0;
                            g_MulMag.RxDataCnt = 0;
                            g_MulMag.CrcError = 0;
												
                            if (g_MulMag.RxData[2] != 0xF0) {
                                g_MulMag.Error.bit.F0 = 1;
                                Work_Alarm = WORK_ALARM_F0_ERROR;
                            } else {
                                g_MulMag.Error.bit.F0 = 0;
                            }
                        }
                        break;
                    case 0x51:
                        if (g_MulMag.RxDataCnt == 6) {
                            g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 5);
                            if (g_MulMag.CrcData != g_MulMag.RxData[5]) {
                                MulMag_CrcError();
																return;
                            }														
                            g_MulMag.TimeoutCnt = 0;
                            g_MulMag.RxDataCnt = 0;
														g_MulMag.CrcError = 0;

                        }
                        break;
                    case 0x52:
                        if (g_MulMag.RxDataCnt == 6) {
                            g_MulMag.CrcData = Enc_CRC8((uint8_t *)g_MulMag.RxData, 5);
                            if (g_MulMag.CrcData != g_MulMag.RxData[5]) {
                                MulMag_CrcError();
																return;
                            }													
                            g_MulMag.TimeoutCnt = 0;
                            g_MulMag.RxDataCnt = 0;

														g_MulMag.CrcError = 0;
														if (g_MulMag.RxData[2] == 0x11) {
																g_MT6835Addr.Reg0x11 = g_MulMag.RxData[4];
																if (g_MT6835Addr.Reg0x11 == g_MT6835Addr.Reg0x11_True) {
																		g_MT6835Addr.RegBit.bit.Bit11 = 1;
																}
														} else if (g_MulMag.RxData[2] == 0xDA) {
																g_MT6835Addr.Reg0xDA = g_MulMag.RxData[4];
																if (g_MT6835Addr.Reg0xDA == g_MT6835Addr.Reg0xDA_True) {
																		g_MT6835Addr.RegBit.bit.BitDA = 1;
																}
														} else if (g_MulMag.RxData[2] == 0xEA) {
																g_MT6835Addr.Reg0xEA = g_MulMag.RxData[4];
																if (g_MT6835Addr.Reg0xEA == g_MT6835Addr.Reg0xEA_True) {
																		g_MT6835Addr.RegBit.bit.BitEA = 1;
																}
														} else if (g_MulMag.RxData[2] == 0xEC) {
																g_MT6835Addr.Reg0xEC = g_MulMag.RxData[4];
																if (g_MT6835Addr.Reg0xEC == g_MT6835Addr.Reg0xEC_True) {
																		g_MT6835Addr.RegBit.bit.BitEC = 1;
																}
														} else if (g_MulMag.RxData[2] == 0x12) {
																g_MT6835Addr.Reg0x12 = g_MulMag.RxData[4];
																if (g_MT6835Addr.Reg0x12 == g_MT6835Addr.Reg0x12_True) {
																		g_MT6835Addr.RegBit.bit.Bit12 = 1;
																}
														}

                        }
                        break;
                }
            }
            break;
            
        default:
						g_MulMag.RxDataCnt = 0;
            break;
    }
    
    
}

/****************************************************************************************
* 函数名称：MulMag_MultiturnReset
* 函数功能：多圈复位操作
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_MultiturnReset(void)
{
    if (g_MulMag.Cnt.CF62 < 100) {
        MulMag_TX(0x62, 0x00, 0x00);
        g_MulMag.Cnt.CF62++;
        if (g_MulMag.Cnt.CF62 == 100) {
            g_MotorEncoder.TestItem = MulMag_Test_Stop;
        }
    }
}

/****************************************************************************************
* 函数名称：MulMag_GetAllData
* 函数功能：获取编码器全部数据（单圈+多圈+状态）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_GetAllData(void)
{
    static uint8_t toggle = 0;
    if (toggle < 10) {
        MulMag_TX(0x1A, 0x00, 0x00);
        toggle ++;
    } else {
        g_MotorEncoder.TestItem = MulMag_Test_Stop;
        MulMag_TX(0x75, 0x00, 0x00);
        toggle = 0;
    }
}

/****************************************************************************************
* 函数名称：MulMag_ReadHWRev
* 函数功能：从EEPROM主缓存解析硬件版本号
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-2
****************************************************************************************/
void MulMag_ReadHWRev(void)
{
    snprintf(g_MotorEncoder.HWRevBuffer, sizeof(g_MotorEncoder.HWRevBuffer), 
             "%c.%c.%d.%d", 
             g_MulMag.Eeprom.DataBuffer[2][0x70],
             g_MulMag.Eeprom.DataBuffer[2][0x71],
             g_MulMag.Eeprom.DataBuffer[2][0x72],
             g_MulMag.Eeprom.DataBuffer[2][0x73]);
}

/****************************************************************************************
* 函数名称：MulMag_ReadFWRev
* 函数功能：从EEPROM主缓存解析软件版本号
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-2
****************************************************************************************/
void MulMag_ReadFWRev(void)
{
    snprintf(g_MotorEncoder.FWRevBuffer, sizeof(g_MotorEncoder.FWRevBuffer), 
             "%d.%d.%d.%c", 
             g_MulMag.Eeprom.DataBuffer[3][20],
             g_MulMag.Eeprom.DataBuffer[3][21],
             g_MulMag.Eeprom.DataBuffer[3][22],
             g_MulMag.Eeprom.DataBuffer[3][23]);
}

/****************************************************************************************
* 函数名称：MulMag_ReadAllEeprom
* 函数功能：读取编码器全部 EEPROM 数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_ReadAllEeprom(void)
{
    if (g_MulMag.Eeprom.SetPage < MULENC_PAGE_NUMBER) {
        g_MulMag.Eeprom.SetDNum = 0x80;
        g_MulMag.Eeprom.SetAddress = 0x00;
        MulMag_TX(0xAD, 0x00, 0x00);
        g_MulMag.Eeprom.SetPage++;
    } else {
        g_MotorEncoder.TestItem = MulMag_Test_Stop;
        MulMag_ReadHWRev();
        MulMag_ReadFWRev();
        g_MulMag.ReadResolutionID = g_MulMag.Eeprom.DataBuffer[2][0x65];
    }
}

/****************************************************************************************
* 函数名称：MulMag_Initialize
* 函数功能：编码器初始化流程（ced）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_Initialize(void)
{
    if (g_MulMag.Cnt.CFCED < 5) {
        // 当 CFCED == 4 时，说明即将进行的是第 5 次发送
        if(g_MulMag.Cnt.CFCED == 4) {
            EncDrv_SetRxWaitDelay(1, ENC_UNIT_S); // 为即将发送的第5次数据配置长超时
        }
        
        MulMag_TX(0x6D, 0x00, Encoder_CED_Test);
        g_MulMag.Cnt.CFCED++;
    } else {
        g_MulMag.Cnt.CFCED = 0;
        g_MotorEncoder.TestItem = MulMag_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MulMag_HallRead
* 函数功能：读取 Hall 位置数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_HallRead(void)
{
    MulMag_TX(0x25, 0x00, 0x00);
    if (g_MulMag.Cnt.CF25 < 100) {
        g_MulMag.Cnt.CF25++;
    }
}

/****************************************************************************************
* 函数名称：MulMag_WriteResult
* 函数功能：写入测试结果到编码器
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_WriteResult(void)
{
    g_MulMag.Eeprom.SetPage = 0x02;
    g_MulMag.Eeprom.SetAddress = 0x00;
    g_MulMag.Eeprom.SetDNum = 0x01;
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress] = 0x09;
    MulMag_TX(0x35, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulMag_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulMag_TB
* 函数功能：读取电池电压/温度数据
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_TB(void)
{
    MulMag_TX(0x3D, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulMag_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulMag_OpenInternalProtocol
* 函数功能：打开编码器内部协议模式
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_OpenInternalProtocol(void)
{
    if (g_MulMag.Cnt.CFOIP < 5) {
        MulMag_TX(0x6D, 0x00, Encoder_OIP_Test);
        g_MulMag.Cnt.CFOIP++;
    } else {
        g_MulMag.Cnt.CFOIP = 0;
        g_MotorEncoder.TestItem = MulMag_Test_Stop;
    }
}

/****************************************************************************************
* 函数名称：MulMag_CloseInternalProtocol
* 函数功能：关闭编码器内部协议模式
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_CloseInternalProtocol(void)
{
    MulMag_TX(0x6D, 0x00, Encoder_CIP_Test);
    g_MotorEncoder.TestItem = MulMag_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulMag_WriteDate
* 函数功能：写入日期到编码器 EEPROM
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_WriteDate(void)
{
    g_MulMag.Eeprom.SetPage = 0x02;
    g_MulMag.Eeprom.SetAddress = 0x68;
    g_MulMag.Eeprom.SetDNum = 0x04;
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress] = g_MotorEncoder.TestYear;
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress + 1] = g_MotorEncoder.TestMoon;
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress + 2] = g_MotorEncoder.TestDay;
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress + 3] = g_MotorEncoder.TestHour;
    MulMag_TX(0x35, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulMag_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulMag_SetResolution
* 函数功能：设置编码器分辨率
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_SetResolution(void)
{
    MulMag_TX(0x40, 0x46, g_MotorEncoder.SetResolutionID);
    g_MotorEncoder.TestItem = MulMag_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulMag_ReadResolution
* 函数功能：读取编码器分辨率
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_ReadResolution(void)
{
    MulMag_TX(0x40, 0x45, 0x00);
    g_MotorEncoder.TestItem = MulMag_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulMag_WriteHWRev
* 函数功能：写入硬件版本到编码器 EEPROM
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_WriteHWRev(void)
{
    g_MulMag.Eeprom.SetPage = 0x02;
    g_MulMag.Eeprom.SetAddress = 0x70;
    g_MulMag.Eeprom.SetDNum = 0x04;
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress] = 'H';
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress + 1] = 'M';
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress + 2] = 1;
    g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress + 3] = g_MotorEncoder.HWRevData;
    MulMag_TX(0x35, 0x00, 0x00);
    g_MotorEncoder.TestItem = MulMag_Test_Stop;
}

/****************************************************************************************
* 函数名称：MulMag_WriteRegister
* 函数功能：写入寄存器（按步骤写入多个寄存器）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_WriteRegister(void)
{
    switch (s_WriteRegStep) {
        case 0:
            g_MT6835Addr.Reg0x11_True = 0x06;
            MulMag_TxRegister(0x40, 0x51, 0x11, g_MT6835Addr.Reg0x11_True);
            s_WriteRegStep = 1;
            break;
        case 1:
            g_MT6835Addr.Reg0xDA_True = 0x01;
            MulMag_TxRegister(0x40, 0x51, 0xDA, g_MT6835Addr.Reg0xDA_True);
            s_WriteRegStep = 2;
            break;
        case 2:
            g_MT6835Addr.Reg0xEA_True = 0x01;
            MulMag_TxRegister(0x40, 0x51, 0xEA, g_MT6835Addr.Reg0xEA_True);
            s_WriteRegStep = 3;
            break;
        case 3:
            g_MT6835Addr.Reg0xEC_True = 0x02;
            MulMag_TxRegister(0x40, 0x51, 0xEC, g_MT6835Addr.Reg0xEC_True);
            s_WriteRegStep = 4;
            break;
        case 4:
            g_MT6835Addr.Reg0x12_True = 0xA0;
            MulMag_TxRegister(0x40, 0x51, 0x12, g_MT6835Addr.Reg0x12_True);
            g_MotorEncoder.TestItem = MulMag_Test_ReadRegister;
            s_WriteRegStep = 0;
            break;
    }
}

/****************************************************************************************
* 函数名称：MulMag_ReadRegister
* 函数功能：读取寄存器（按步骤读取多个寄存器）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_ReadRegister(void)
{
    switch (s_ReadRegStep) {
        case 0:
            MulMag_TxRegister(0x40, 0x52, 0x11, 0x06);
            s_ReadRegStep = 1;
            break;
        case 1:
            MulMag_TxRegister(0x40, 0x52, 0xDA, 0x01);
            s_ReadRegStep = 2;
            break;
        case 2:
            MulMag_TxRegister(0x40, 0x52, 0xEA, 0x01);
            s_ReadRegStep = 3;
            break;
        case 3:
            MulMag_TxRegister(0x40, 0x52, 0xEC, 0x02);
            s_ReadRegStep = 4;
            break;
        case 4:
            MulMag_TxRegister(0x40, 0x52, 0x12, 0xA0);
            s_ReadRegStep = 5;
            break;
        case 5:
            if (g_MT6835Addr.RegBit.all == 0x1F) {
                g_MT6835Addr.TestRegData = 1;
            } else {
                g_MT6835Addr.TestRegData = 2;
            }
            g_MotorEncoder.TestItem = MulMag_Test_Stop;
            s_ReadRegStep = 0;
            break;
    }
}

/****************************************************************************************
* 函数名称：MulMag_Modbus_IsMyAddr
* 函数功能：判断 Modbus 地址是否属于多圈磁编码器模块
* 输入参量：addr Modbus 寄存器地址
* 输出参量：1=属于本模块，0=不属于
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t MulMag_Modbus_IsMyAddr(uint16_t addr)
{
    return (addr >= 0x0200 && addr <= 0x02FF);
}

/****************************************************************************************
* 函数名称：MulMag_Modbus_Read
* 函数功能：Modbus 03H 读取处理，返回对应地址的数据
* 输入参量：addr Modbus 寄存器地址
* 输出参量：对应地址的数据值
* 编写日期：2026-2-24
****************************************************************************************/
uint16_t MulMag_Modbus_Read(uint16_t addr)
{
    switch (addr) {
        case 0x0200: return g_MulMag.Hall.Result;
        case 0x0201: return g_MotorEncoder.ActualSpeed;
        case 0x0202: return g_MulMag.Status0x6D;
        case 0x0203: return g_MulMag.Eeprom.DataBuffer[2][0];
        case 0x0204: return g_MulMag.Status.all;
        case 0x0205: return g_MulMag.Alarm.all;
        case 0x0206: return g_MulMag.Eeprom.DataBuffer[3][20];
        case 0x0207: return g_MulMag.Eeprom.DataBuffer[3][21];
        case 0x0208: return g_MulMag.Eeprom.DataBuffer[3][22];
        case 0x0209: return g_MulMag.Eeprom.DataBuffer[3][23];
        case 0x020A: return g_MulMag.BatteryVoltage;
        case 0x020B: return g_MulMag.Eeprom.DataBuffer[0x02][0x68];
        case 0x020C: return g_MulMag.Eeprom.DataBuffer[0x02][0x69];
        case 0x020D: return g_MulMag.Eeprom.DataBuffer[0x02][0x6A];
        case 0x020E: return g_MulMag.Eeprom.DataBuffer[0x02][0x6B];
        case 0x020F: return g_MulMag.ReadResolutionID;
        case 0x0210: return g_MT6835Addr.TestRegData;
        case 0x0211: return g_MotorEncoder.PitCnt;
        case 0x0212: return (g_MulMag.Hall.Buffer[3]<<12) | 
                           (g_MulMag.Hall.Buffer[2]<<8) | 
                           (g_MulMag.Hall.Buffer[1]<<4) | 
                           g_MulMag.Hall.Buffer[0];
        case 0x0213: return g_MulMag.Eeprom.DataBuffer[0x03][0x0A];
        default:
            Communication_Address_Error();
            return 0xFFFF;
    }
}

/****************************************************************************************
* 函数名称：MulMag_Modbus_Write
* 函数功能：Modbus 06H 写入处理，写入指定地址的数据
* 输入参量：addr Modbus 寄存器地址，value 写入值
* 输出参量：0=成功，非0=错误
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t MulMag_Modbus_Write(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case 0x0200:  // 读取 EEPROM
            if (value == 1) {
							g_MulMag.Eeprom.SetPage = 0;//从0开始，以防g_MulMag.Eeprom.SetPage此时不是0
                g_MotorEncoder.TestItem = MulMag_Test_ReadAllEeprom;
            }
            return 0;
            
        case 0x0201:  // 编码器初始化
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_Initialize;
            }
            return 0;
            
        case 0x0202:  // Hall 测试
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_Hall;
            }
            return 0;
            
        case 0x0203:  // 写入测试结果
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_WriteResult;
            }
            return 0;
            
        case 0x0204:  // 获取全部数据
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_GetAllData;
            }
            return 0;
            
        case 0x0205:  // 多圈复位
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_Multiturn;
            }
            return 0;
            
        case 0x0206:  // 电池电压/温度
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_TB;
            }
            return 0;
            
        case 0x0207:  // 打开内部协议
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_OIP;
            }
            return 0;
            
        case 0x0208:  // 关闭内部协议
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_CIP;
            }
            return 0;
            
        case 0x0209:  // 设置分辨率 (value 为分辨率 ID)
            g_MotorEncoder.SetResolutionID = (uint8_t)value;
            g_MotorEncoder.TestItem = MulMag_Test_SetResolution;
            return 0;
            
        case 0x020A:  // 读取分辨率
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_ReadResolution;
            }
            return 0;

        case 0x020B:  // 写入硬件版本
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_WriteHWRev;
            }
            return 0;

        case 0x020C:  // 写入 MT6835 寄存器内容
            if (value == 1) {
                g_MotorEncoder.TestItem = MulMag_Test_WriteRegister;
            }
            return 0;

        default:
            Communication_Address_Error();
            return 1;  // 无效地址
    }
}

/****************************************************************************************
* 函数名称：MulMag_Test
* 函数功能：测试函数主入口，根据测试项目执行对应操作
* 输入参量：testItem 测试项目编号
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_Test(uint8_t testItem)
{
    switch (testItem) {
        case MulMag_Test_Stop:               break;
        case MulMag_Test_Multiturn:          MulMag_MultiturnReset(); break;
        case MulMag_Test_GetAllData:         MulMag_GetAllData(); break;
        case MulMag_Test_ReadAllEeprom:      MulMag_ReadAllEeprom(); break;
        case MulMag_Test_Initialize:         MulMag_Initialize(); break;
        case MulMag_Test_Hall:               MulMag_HallRead(); break;
        case MulMag_Test_WriteResult:        MulMag_WriteResult(); break;
        case MulMag_Test_TB:                 MulMag_TB(); break;
        case MulMag_Test_OIP:                MulMag_OpenInternalProtocol(); break;
        case MulMag_Test_CIP:                MulMag_CloseInternalProtocol(); break;
        case MulMag_Test_WriteDate:          MulMag_WriteDate(); break;
        case MulMag_Test_SetResolution:      MulMag_SetResolution(); break;
        case MulMag_Test_ReadResolution:     MulMag_ReadResolution(); break;
        case MulMag_Test_WriteHWRev:         MulMag_WriteHWRev(); break;
        case MulMag_Test_WriteRegister:      MulMag_WriteRegister(); break;
        case MulMag_Test_ReadRegister:       MulMag_ReadRegister(); break;
        default: break;
    }
}
