/****************************************************************************************
 * @file      Encoder_MultiturnMag.c
 * @brief     多圈磁编码器协议实现 (完整移植自 MultiturnMagneticEncoder.c)
 * @author    Gemini (移植优化)
 * @date      2026-02-09
 * @note      保持原有寄存器地址 (0x0200-0x0213) 和测试逻辑完全一致
 ****************************************************************************************/
#include "Encoder_MultiturnMag.h"
#include "encoder_driver.h"
#include "modbus_function.h"
#include <string.h>

// ================= 全局变量 =================
MulMag_t g_MulMag = {0};
MotorEncoder_t g_MotorEncoder = {0};
MT6835_Addr_t g_MT6835Addr = {0};
volatile uint8_t Work_Alarm = 0;

static uint8_t s_WriteRegStep = 0;
static uint8_t s_ReadRegStep = 0;

/****************************************************************************************
* 函数名称：Enc_CRC8
* 函数功能：计算 XOR 校验值（与编码器端算法一致）
* 输入参量：data 数据指针，len 数据长度
* 输出参量：校验值
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t Enc_CRC8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
    }
    return crc;
}

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
* 函数名称：MulMag_TX
* 函数功能：发送编码器命令，根据 cmd 类型组帧并启动 DMA 发送
* 输入参量：cmd 命令码，addr 地址，data 数据
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_TX(uint8_t cmd, uint16_t addr, uint8_t data)
{
    g_MulMag.TimeoutCnt++;
    g_MulMag.TxID = cmd;
    g_MulMag.TxData[0] = cmd;
    
    switch (cmd) {
        case CMD_GET_ALLDATA:
            EncDrv_SendDMA(g_MulMag.TxData, 1, 11);
            break;
        case CMD_GET_SINGLETURN:
        case CMD_GET_MULTITURN:
        case CMD_READ_BATTERY:
            EncDrv_SendDMA(g_MulMag.TxData, 1, 6);
            break;
        case CMD_MULTITURN_RESET:
        case CMD_READ_ALARM:
            EncDrv_SendDMA(g_MulMag.TxData, 1, 4);
            break;
        case CMD_READ_HALL_POS:
            EncDrv_SendDMA(g_MulMag.TxData, 1, 8);
            break;
            
        case CMD_READ_EEPROM:
            g_MulMag.TxData[0] = cmd;
            g_MulMag.TxData[1] = (uint8_t)addr;
            g_MulMag.TxData[2] = Enc_CRC8(g_MulMag.TxData, 2);
            EncDrv_SendDMA(g_MulMag.TxData, 3, 4);
            break;
            
        case CMD_WRITE_EEPROM:
            g_MulMag.TxData[0] = cmd;
            g_MulMag.TxData[1] = (uint8_t)addr;
            g_MulMag.TxData[2] = data;
            g_MulMag.TxData[3] = Enc_CRC8(g_MulMag.TxData, 3);
            EncDrv_SendDMA(g_MulMag.TxData, 4, 4);
            break;
            
        case CMD_READ_BATCH_EEPROM:
            g_MulMag.TxData[0] = cmd;
            g_MulMag.TxData[1] = g_MulMag.Eeprom.SetPage;
            g_MulMag.TxData[2] = g_MulMag.Eeprom.SetAddress;
            g_MulMag.TxData[3] = g_MulMag.Eeprom.SetDNum;
            g_MulMag.TxData[4] = Enc_CRC8(g_MulMag.TxData, 4);
            EncDrv_SendDMA(g_MulMag.TxData, 5, 5 + g_MulMag.Eeprom.SetDNum);
            break;
            
        case CMD_WRITE_BATCH_EEPROM:
            {
                g_MulMag.TxData[0] = cmd;
                g_MulMag.TxData[1] = g_MulMag.Eeprom.SetPage;
                g_MulMag.TxData[2] = g_MulMag.Eeprom.SetAddress;
                g_MulMag.TxData[3] = g_MulMag.Eeprom.SetDNum;
                uint8_t j;
                for (j = 0; j < g_MulMag.Eeprom.SetDNum; j++) {
                    g_MulMag.TxData[4 + j] = g_MulMag.Eeprom.DataBuffer[g_MulMag.Eeprom.SetPage][g_MulMag.Eeprom.SetAddress + j];
                }
                g_MulMag.TxData[4 + j] = Enc_CRC8(g_MulMag.TxData, 4 + j);
                EncDrv_SendDMA(g_MulMag.TxData, 5 + j, 4);
            }
            break;
            
        case CMD_INIT_RESULT:
            g_MulMag.IPID = data;
            g_MulMag.TxData[0] = cmd;
            if (data == Encoder_ced_Test) {
                g_MulMag.TxData[1] = 0x63;
                g_MulMag.TxData[2] = 0x65;
                g_MulMag.TxData[3] = 0x64;
            } else if (data == Encoder_CIP_Test) {
                g_MulMag.TxData[1] = 0x43;
                g_MulMag.TxData[2] = 0x49;
                g_MulMag.TxData[3] = 0x50;
            } else {
                g_MulMag.TxData[1] = 0x4F;
                g_MulMag.TxData[2] = 0x49;
                g_MulMag.TxData[3] = 0x50;
            }
            g_MulMag.TxData[4] = Enc_CRC8(g_MulMag.TxData, 4);
            EncDrv_SendDMA(g_MulMag.TxData, 5, 6);
            break;
            
        case CMD_SET_RESOLUTION:
            if (addr == 0x46) {
                g_MulMag.TxData[0] = cmd;
                g_MulMag.TxData[1] = 0x46;
                g_MulMag.TxData[2] = data;
                g_MulMag.TxData[3] = Enc_CRC8(g_MulMag.TxData, 3);
                EncDrv_SendDMA(g_MulMag.TxData, 4, 4);
            } else if (addr == 0x45) {
                g_MulMag.TxData[0] = cmd;
                g_MulMag.TxData[1] = 0x45;
                g_MulMag.TxData[2] = 0x00;
                g_MulMag.TxData[3] = Enc_CRC8(g_MulMag.TxData, 3);
                EncDrv_SendDMA(g_MulMag.TxData, 4, 4);
            }
            break;
            
        default:
            break;
    }
    
    // 超时检测
    if (g_MulMag.TimeoutCnt > 5) {
        g_MulMag.Error.bit.DC = 1;
        Work_Alarm = WORK_ALARM_TIMEOUT;
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
    g_MulMag.TimeoutCnt++;
    
    if (status == 0x51) {
        g_MulMag.TxData[0] = cmd;
        g_MulMag.TxData[1] = status;
        g_MulMag.TxData[2] = addr;
        g_MulMag.TxData[3] = 0x00;
        g_MulMag.TxData[4] = data;
        g_MulMag.TxData[5] = Enc_CRC8(g_MulMag.TxData, 5);
        EncDrv_SendDMA(g_MulMag.TxData, 6, 6);
    } else if (status == 0x52) {
        g_MulMag.TxData[0] = cmd;
        g_MulMag.TxData[1] = status;
        g_MulMag.TxData[2] = addr;
        g_MulMag.TxData[3] = 0x00;
        g_MulMag.TxData[4] = Enc_CRC8(g_MulMag.TxData, 4);
        EncDrv_SendDMA(g_MulMag.TxData, 5, 6);
    }
    
    // 超时检测
    if (g_MulMag.TimeoutCnt > 5) {
        g_MulMag.Error.bit.DC = 1;
        Work_Alarm = WORK_ALARM_TIMEOUT;
    }
}

/****************************************************************************************
* 函数名称：MulMag_RxHandler
* 函数功能：接收处理（逐字节），存储接收数据
* 输入参量：byte 接收到的字节
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_RxHandler(uint8_t byte)
{
    g_MulMag.RxData[g_MulMag.RxDataCnt++] = byte;
    if (g_MulMag.RxDataCnt >= MULENC_RX_SIZE) {
        g_MulMag.RxDataCnt = 0;
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
    uint8_t crc_ok = 0;
    
    switch (cmd) {
        case 0x1A:
            if (g_MulMag.RxDataCnt >= 11) {
                g_MulMag.CrcData = Enc_CRC8(g_MulMag.RxData, 10);
                crc_ok = (g_MulMag.CrcData == g_MulMag.RxData[10]);
                if (!crc_ok) {
                    g_MulMag.Error.bit.EC = 1;
                    Work_Alarm = WORK_ALARM_CRC_ERROR;
                    g_MulMag.CrcError = 1;
                    g_MulMag.RxDataCnt = 0;
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
                g_MulMag.CrcData = Enc_CRC8(g_MulMag.RxData, 5);
                crc_ok = (g_MulMag.CrcData == g_MulMag.RxData[5]);
                if (!crc_ok) {
                    g_MulMag.Error.bit.EC = 1;
                    Work_Alarm = WORK_ALARM_CRC_ERROR;
                    g_MulMag.CrcError = 1;
                    g_MulMag.RxDataCnt = 0;
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
            if (g_MulMag.RxDataCnt >= 8) {
                g_MulMag.CrcData = Enc_CRC8(g_MulMag.RxData, 7);
                crc_ok = (g_MulMag.CrcData == g_MulMag.RxData[7]);
                if (!crc_ok) {
                    g_MulMag.Error.bit.EC = 1;
                    Work_Alarm = WORK_ALARM_CRC_ERROR;
                    g_MulMag.CrcError = 1;
                    g_MulMag.RxDataCnt = 0;
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Status.all = g_MulMag.RxData[1];
                g_MulMag.SingleTurnPos = ((uint32_t)g_MulMag.RxData[4] << 16) | 
                                         ((uint32_t)g_MulMag.RxData[3] << 8) | 
                                         g_MulMag.RxData[2];
                g_MulMag.Hall.Buffer[0] = (g_MulMag.RxData[5] >> 4) & 0x0F;
                g_MulMag.Hall.Buffer[1] = g_MulMag.RxData[5] & 0x0F;
                g_MulMag.Hall.Buffer[2] = (g_MulMag.RxData[6] >> 4) & 0x0F;
                g_MulMag.Hall.Buffer[3] = g_MulMag.RxData[6] & 0x0F;
                g_MulMag.CrcError = 0;
            }
            break;
            
        case 0x75:
            if (g_MulMag.RxDataCnt >= 5) {
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxDataCnt = 0;
                
                g_MulMag.RxID = g_MulMag.RxData[0];
                g_MulMag.InternalAlarm1 = g_MulMag.RxData[1];
                g_MulMag.InternalAlarm2 = g_MulMag.RxData[2];
                g_MulMag.InternalAlarm3 = g_MulMag.RxData[3];
                
                g_MulMag.CrcData = Enc_CRC8(g_MulMag.RxData, 4);
                if (g_MulMag.CrcData != g_MulMag.RxData[4]) {
                    g_MulMag.XorCrcError = 1;
                    ModBus.Error.bit.EC = 1; 
                    Work_Alarm = WORK_ALARM_CRC_ERROR;
                } else {
                    g_MulMag.XorCrcError = 0;
                }
            }
            break;
            
        case 0x3D:
            if (g_MulMag.RxDataCnt >= 6) {
                g_MulMag.CrcData = Enc_CRC8(g_MulMag.RxData, 5);
                crc_ok = (g_MulMag.CrcData == g_MulMag.RxData[5]);
                if (!crc_ok) {
                    g_MulMag.Error.bit.EC = 1;
                    Work_Alarm = WORK_ALARM_CRC_ERROR;
                    g_MulMag.CrcError = 1;
                    g_MulMag.RxDataCnt = 0;
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.BatteryVoltage = ((uint16_t)g_MulMag.RxData[2] << 8) | g_MulMag.RxData[1];
                g_MulMag.Temperature = ((uint16_t)g_MulMag.RxData[4] << 8) | g_MulMag.RxData[3];
                g_MulMag.CrcError = 0;
            }
            break;
            
        case 0x6D:
            if (g_MulMag.RxDataCnt >= 6) {
                g_MulMag.CrcData = Enc_CRC8(g_MulMag.RxData, 5);
                crc_ok = (g_MulMag.CrcData == g_MulMag.RxData[5]);
                if (!crc_ok) {
                    g_MulMag.Error.bit.EC = 1;
                    Work_Alarm = WORK_ALARM_CRC_ERROR;
                    g_MulMag.CrcError = 1;
                    g_MulMag.RxDataCnt = 0;
                    return;
                }
                g_MulMag.TimeoutCnt = 0;
                g_MulMag.RxID = cmd;
                g_MulMag.Status0x6D = g_MulMag.RxData[4];
                g_MulMag.CrcError = 0;
            }
            break;
            
        case 0xAD:
            {
                uint8_t expected_len = 4 + g_MulMag.Eeprom.SetDNum + 1;
                if (g_MulMag.RxDataCnt >= expected_len) {
                    g_MulMag.CrcData = Enc_CRC8(g_MulMag.RxData, expected_len - 1);
                    crc_ok = (g_MulMag.CrcData == g_MulMag.RxData[expected_len - 1]);
                    if (!crc_ok) {
                        g_MulMag.Error.bit.EC = 1;
                        Work_Alarm = WORK_ALARM_CRC_ERROR;
                        g_MulMag.CrcError = 1;
                        g_MulMag.RxDataCnt = 0;
                        return;
                    }
                    g_MulMag.TimeoutCnt = 0;
                    g_MulMag.RxID = cmd;
                    g_MulMag.Eeprom.ReturnPage = g_MulMag.RxData[1];
                    g_MulMag.Eeprom.ReturnAddress = g_MulMag.RxData[2];
                    uint8_t page = g_MulMag.Eeprom.ReturnPage;
                    if (page < MULENC_EEPROM_PAGES) {
                        for (uint8_t i = 0; i < g_MulMag.Eeprom.SetDNum && i < MULENC_EEPROM_ADDRS; i++) {
                            g_MulMag.Eeprom.DataBuffer[page][i] = g_MulMag.RxData[4 + i];
                        }
                    }
                    g_MulMag.CrcError = 0;
                }
            }
            break;
            
        default:
            break;
    }
    
    g_MulMag.RxDataCnt = 0;
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
        MulMag_TX(CMD_GET_MULTITURN, 0x00, 0x00);
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
    if (toggle == 0) {
        MulMag_TX(CMD_GET_ALLDATA, 0x00, 0x00);
        toggle = 0;
    } else {
        g_MotorEncoder.TestItem = MulMag_Test_Stop;
        MulMag_TX(CMD_READ_ALARM, 0x00, 0x00);
        toggle = 0;
    }
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
        MulMag_TX(CMD_READ_BATCH_EEPROM, 0x00, 0x00);
        g_MulMag.Eeprom.SetPage++;
    } else {
        g_MotorEncoder.TestItem = MulMag_Test_Stop;
        g_MulMag.ReadResolutionID = g_MulMag.Eeprom.DataBuffer[2][0x65];
    }
}

/****************************************************************************************
* 函数名称：MulMag_Initialize
* 函数功能：编码器初始化流程（ced/CIP/OIP）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void MulMag_Initialize(void)
{
    if (g_MulMag.Cnt.CFced < 5) {
        MulMag_TX(CMD_INIT_RESULT, 0x00, Encoder_ced_Test);
        g_MulMag.Cnt.CFced++;
    } else {
        g_MulMag.Cnt.CFced = 0;
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
    MulMag_TX(CMD_READ_HALL_POS, 0x00, 0x00);
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
    MulMag_TX(CMD_WRITE_BATCH_EEPROM, 0x00, 0x00);
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
    MulMag_TX(CMD_READ_BATTERY, 0x00, 0x00);
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
        MulMag_TX(CMD_INIT_RESULT, 0x00, Encoder_OIP_Test);
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
    MulMag_TX(CMD_INIT_RESULT, 0x00, Encoder_CIP_Test);
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
    MulMag_TX(CMD_WRITE_BATCH_EEPROM, 0x00, 0x00);
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
    MulMag_TX(CMD_SET_RESOLUTION, 0x46, g_MotorEncoder.SetResolutionID);
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
    MulMag_TX(CMD_SET_RESOLUTION, 0x45, 0x00);
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
    MulMag_TX(CMD_WRITE_BATCH_EEPROM, 0x00, 0x00);
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
        default: return 0xFFFF;
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
            
        default:
            return 1;  // 无效地址
    }
}
