/****************************************************************************************
  * @file      modbus_function.c
  * @brief     Modbus 通信协议实现
  * ****************************************************************************************/
#include "modbus_function.h"
#include "uart_config.h"
#include "relay_control.h"
#include "usart.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "iap_function.h"

volatile strModBus ModBus = {0};

/****************************************************************************************
* 函数名称：Modbus_CRC16
* 函数功能：计算 Modbus RTU 帧的 CRC16 校验码
* 输入参量：
* - Data：指向需要计算 CRC 的数据缓冲区的指针
* - Len：数据缓冲区的长度（字节数）
* 输出参量：
* - uint16_t：计算出的 16 位 CRC 校验码
* 编写日期：2025-8-27
****************************************************************************************/
uint16_t Modbus_CRC16(volatile uint8_t * Data, uint8_t Len)
{
    uint16_t uCRC = 0xFFFF; // CRC 初始化值为 0xFFFF
    uint8_t i = 0;          // 循环计数器

    // 遍历数据缓冲区中的每个字节
    while (Len--) {
        uCRC ^= *Data++;    // 将当前字节与 CRC 寄存器的低字节进行异或操作

        // 对当前字节的每一位进行处理（共 8 位）
        for (i = 0; i < 8; i++) {
            if (uCRC & 0x01) { // 检查 CRC 寄存器的最低位是否为 1
                uCRC = (uCRC >> 1) ^ 0xA001; // 如果最低位为 1，右移 1 位并与多项式 0xA001 异或
            } else {
                uCRC = (uCRC >> 1); // 如果最低位为 0，仅右移 1 位
            }
        }
    }

    return uCRC; // 返回最终计算出的 16 位 CRC 校验码
}

/****************************************************************************************
* 函数名称：ModBus_Slave_SendErrorResponse
* 函数功能：发送 Modbus 异常响应帧
* 输入参量：exception_code 异常码
* 输出参量：无
****************************************************************************************/
void ModBus_Slave_SendErrorResponse(uint8_t exception_code)
{   
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80; // 功能码最高位置 1 表示错误响应
    Usart1.TxData[2] = exception_code;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)(crc >> 8);
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx03DataCollation
* 函数功能：解析 Modbus 03H 命令接收的数据，包括数据地址、数量和 CRC 校验
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveRx03DataCollation(void)
{
    ModBus.Slave.Rx.DataAddrHigh = Usart1.RxData[2];
    ModBus.Slave.Rx.DataAddrLow = Usart1.RxData[3];
    ModBus.Slave.Rx.DataAddr = ((ModBus.Slave.Rx.DataAddrHigh << 8) | ModBus.Slave.Rx.DataAddrLow) & 0xFFFF;
    ModBus.Slave.Rx.DataCountHigh = Usart1.RxData[4];
    ModBus.Slave.Rx.DataCountLow = Usart1.RxData[5];
    ModBus.Slave.Rx.DataSize = ((ModBus.Slave.Rx.DataCountHigh << 8) | ModBus.Slave.Rx.DataCountLow) & 0xFFFF;
    
    if(ModBus.Slave.Rx.DataSize >= 29){
        return;
    }
    ModBus.Slave.Rx.CRCLow = (uint8_t)((Modbus_CRC16((uint8_t *)Usart1.RxData, 6)) & 0xFF);
    ModBus.Slave.Rx.CRCHigh = (uint8_t)((Modbus_CRC16((uint8_t *)Usart1.RxData, 6)) >> 8) & 0xFF;    
}

/****************************************************************************************
* 函数名称：ModBus_SlaveReturnTx03
* 函数功能：根据 Modbus 03H 命令返回寄存器数据
* 输入参量：
* - ReturnDataStart：起始寄存器地址
* - ReturnDataLen：返回的寄存器数量
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveReturnTx03(uint16_t ReturnDataStart, uint16_t ReturnDataLen)
{
    uint16_t i;
    uint8_t data_bytes = ReturnDataLen * 2;
    uint8_t frame_len_no_crc = 3 + data_bytes;

    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD;
    Usart1.TxData[2] = data_bytes;

    for (i = 0; i < ReturnDataLen; i++) {
        if ((ReturnDataStart + i) < MODBUS_REGISTER_COUNT) {
            uint16_t regValue = ModBus.Slave.DisplayRegisters[ReturnDataStart + i];
            Usart1.TxData[3 + i * 2] = (uint8_t)(regValue >> 8);
            Usart1.TxData[4 + i * 2] = (uint8_t)(regValue & 0xFF);
        }
    }
    
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, frame_len_no_crc);
    Usart1.TxData[frame_len_no_crc] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[frame_len_no_crc + 1] = (uint8_t)(crc >> 8);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = frame_len_no_crc + 2;
    Usart1TransmitterDMA(&Usart1.Tx);
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx03
* 函数功能：处理 Modbus 03H 命令，包括数据解析、校验和返回寄存器数据
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveRx03(void)
{
    if(Usart1.DataCnt == 8){
        ModBus_SlaveRx03DataCollation();
        if(Usart1.RxData[6] != ModBus.Slave.Rx.CRCLow || Usart1.RxData[7] != ModBus.Slave.Rx.CRCHigh){
            ModBus_Slave_SendErrorResponse(0x01); // CRC 校验错误
        }else{
            if ((ModBus.Slave.Rx.DataAddr + ModBus.Slave.Rx.DataSize) <= MODBUS_REGISTER_COUNT) {
                 ModBus_SlaveReturnTx03(ModBus.Slave.Rx.DataAddr, ModBus.Slave.Rx.DataSize);
            } else {
                 ModBus_Slave_SendErrorResponse(0x02); // 非法数据地址
            } 
        }
    }else{
        ModBus_Slave_SendErrorResponse(0x03); // 非法数据值 (用于长度错误)
    }
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx04DataCollation
* 函数功能：解析 Modbus 04H 请求帧格式（与 03H 相同）
* 输入参量：无
* 输出参量：无
****************************************************************************************/
void ModBus_SlaveRx04DataCollation(void)
{
    ModBus.Slave.Rx.DataAddrHigh = Usart1.RxData[2];
    ModBus.Slave.Rx.DataAddrLow = Usart1.RxData[3];
    ModBus.Slave.Rx.DataAddr = ((uint16_t)ModBus.Slave.Rx.DataAddrHigh << 8) | ModBus.Slave.Rx.DataAddrLow;
    ModBus.Slave.Rx.DataCountHigh = Usart1.RxData[4];
    ModBus.Slave.Rx.DataCountLow = Usart1.RxData[5];
    ModBus.Slave.Rx.DataSize = ((uint16_t)ModBus.Slave.Rx.DataCountHigh << 8) | ModBus.Slave.Rx.DataCountLow;

    uint16_t crc_calc = Modbus_CRC16((uint8_t *)Usart1.RxData, 6);
    ModBus.Slave.Rx.CRCLow = (uint8_t)(crc_calc & 0xFF);
    ModBus.Slave.Rx.CRCHigh = (uint8_t)(crc_calc >> 8);
}

/****************************************************************************************
* 函数名称：ModBus_SlaveReturnTx04
* 函数功能：Modbus 04H 功能码响应，返回输入寄存器数据
* 输入参量：SourceDataStart, ReturnDataLen
* 输出参量：无
****************************************************************************************/
void ModBus_SlaveReturnTx04(uint16_t SourceDataStart, uint16_t ReturnDataLen)
{
    uint16_t i;
    uint8_t data_bytes = ReturnDataLen * 2;
    uint8_t frame_len_no_crc = 3 + data_bytes;

    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = 0x04; // 功能码是 04
    Usart1.TxData[2] = data_bytes;

    for (i = 0; i < ReturnDataLen; i++) {
        if ((SourceDataStart + i) < MODBUS_REGISTER_COUNT) {
            // 注意：数据源是主机读取 ADC 模块返回的数据
            uint16_t regValue = ModBus.Master.DisplayRegisters[SourceDataStart + i];
            Usart1.TxData[3 + i * 2] = (uint8_t)(regValue >> 8);
            Usart1.TxData[4 + i * 2] = (uint8_t)(regValue & 0xFF);
        }
    }
    
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, frame_len_no_crc);
    Usart1.TxData[frame_len_no_crc] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[frame_len_no_crc + 1] = (uint8_t)(crc >> 8);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = frame_len_no_crc + 2;
    Usart1TransmitterDMA(&Usart1.Tx);
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx04
* 函数功能：处理 Modbus 04H 功能码（读取输入寄存器）
****************************************************************************************/
void ModBus_SlaveRx04(void)
{
    uint16_t i;
    
    if (Usart1.DataCnt == 8) {
        ModBus_SlaveRx04DataCollation();
        if (Usart1.RxData[6] != ModBus.Slave.Rx.CRCLow || Usart1.RxData[7] != ModBus.Slave.Rx.CRCHigh) {
            ModBus_Slave_SendErrorResponse(0x01); // CRC 校验错误
        } else {
            if ((ModBus.Slave.Rx.DataAddr + ModBus.Slave.Rx.DataSize) <= MODBUS_REGISTER_COUNT) {                            
                for (i = 0; i < ModBus.Slave.Rx.DataSize; i++) {
                    if ((ModBus.Slave.Rx.DataSize + i) < MODBUS_REGISTER_COUNT) {
                        ModBus.Master.DisplayRegisters[ModBus.Slave.Rx.DataAddr + i] = Relay_GetStatus(ModBus.Slave.Rx.DataAddr + i + 1);
                    }
                }                            
                ModBus_SlaveReturnTx04(ModBus.Slave.Rx.DataAddr, ModBus.Slave.Rx.DataSize);
            } else {
                 ModBus_Slave_SendErrorResponse(0x02); // 非法数据地址
            }                   
        }
    } else {
        ModBus_Slave_SendErrorResponse(0x03); // 非法数据值 (用于长度错误)
    }
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx06DataCollation
* 函数功能：解析 Modbus 06H 命令接收的数据，包括地址、数值和 CRC 校验
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveRx06DataCollation(void)
{
    ModBus.Slave.Rx.DataAddrHigh = Usart1.RxData[2];
    ModBus.Slave.Rx.DataAddrLow = Usart1.RxData[3];
    ModBus.Slave.Rx.DataAddr = ((ModBus.Slave.Rx.DataAddrHigh << 8) | ModBus.Slave.Rx.DataAddrLow) & 0xFFFF;
    ModBus.Slave.Rx.DataHigh[0] = Usart1.RxData[4];
    ModBus.Slave.Rx.DataLow[0] = Usart1.RxData[5];
    ModBus.Slave.Rx.Data[0] = ((ModBus.Slave.Rx.DataHigh[0] << 8) | ModBus.Slave.Rx.DataLow[0]) & 0xFFFF;
    ModBus.Slave.Rx.CRCLow = (uint8_t)((Modbus_CRC16((uint8_t *)Usart1.RxData, 6)) & 0xFF);
    ModBus.Slave.Rx.CRCHigh = (uint8_t)((Modbus_CRC16((uint8_t *)Usart1.RxData, 6)) >> 8) & 0xFF; 
}

/****************************************************************************************
* 函数名称：ModBus_SlaveReturnTx06
* 函数功能：Modbus 06H 成功接收后的原样返回确认
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveReturnTx06(void)
{
    memcpy((void*)Usart1.TxData, (void*)Usart1.RxData, 8);
    Usart1.Tx.Data = (uint8_t *)&Usart1.TxData[0];
    Usart1.Tx.DataSize = 8;
    Usart1TransmitterDMA(&Usart1.Tx);
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx06
* 函数功能：处理 Modbus 06H 命令，解析数据、校验并控制继电器状态
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveRx06(void)
{
    if(Usart1.DataCnt == 8){
        ModBus_SlaveRx06DataCollation();
        if(Usart1.RxData[6] != ModBus.Slave.Rx.CRCLow || Usart1.RxData[7] != ModBus.Slave.Rx.CRCHigh){
            ModBus_Slave_SendErrorResponse(0x01); // CRC 校验错误
        }else{
            // 先判断特殊命令地址
            switch(ModBus.Slave.Rx.DataAddr){
                case 0x0000:  // 全部关闭
                    Relay_AllOff();
                    ModBus_SlaveReturnTx06();
                    break;
                case 0x0001:  // 继电器1-8
                case 0x0002:
                case 0x0003:
                case 0x0004:
                case 0x0005:
                case 0x0006:
                case 0x0007:
                case 0x0008:
                    if(ModBus.Slave.Rx.Data[0])
                        Relay_On(ModBus.Slave.Rx.DataAddr);
                    else
                        Relay_Off(ModBus.Slave.Rx.DataAddr);
                    ModBus_SlaveReturnTx06();
                    break;
                case 0x00FF:  // 全部打开
                    Relay_AllOn();
                    ModBus_SlaveReturnTx06();                       
                    break;
                default:
                    // 普通寄存器写入，需要检查范围
                    if ((ModBus.Slave.Rx.DataAddr + 1) <= MODBUS_REGISTER_COUNT) {
                        ModBus.Slave.DisplayRegisters[ModBus.Slave.Rx.DataAddr] = ModBus.Slave.Rx.Data[0];
                        ModBus_SlaveReturnTx06();
                    } else {
                        ModBus_Slave_SendErrorResponse(0x02); // 非法数据地址
                    }
                    break;                          
            }
        }
    }else{
        ModBus_Slave_SendErrorResponse(0x03); // 非法数据值 (用于长度错误)
    }
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx10DataCollation
* 函数功能：解析 Modbus 10H 命令接收的数据，处理多寄存器写入
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveRx10DataCollation(void)
{
    uint16_t i;

    ModBus.Slave.Rx.DataAddrHigh = Usart1.RxData[2];
    ModBus.Slave.Rx.DataAddrLow = Usart1.RxData[3];
    ModBus.Slave.Rx.DataAddr = ((ModBus.Slave.Rx.DataAddrHigh << 8) | ModBus.Slave.Rx.DataAddrLow) & 0xFFFF;

    ModBus.Slave.Rx.DataCountHigh = Usart1.RxData[4];
    ModBus.Slave.Rx.DataCountLow = Usart1.RxData[5];
    uint16_t dataCount = ((ModBus.Slave.Rx.DataCountHigh << 8) | ModBus.Slave.Rx.DataCountLow) & 0xFFFF;

    ModBus.Slave.Rx.DataSize = Usart1.RxData[6];

    // 检查数据个数是否超过最大支持范围
    if (dataCount >= 29) {
        return;
    }

    // 解析具体数据
    for (i = 0; i < dataCount; i++) {
        ModBus.Slave.Rx.DataHigh[i] = Usart1.RxData[7 + i * 2];     // 高字节
        ModBus.Slave.Rx.DataLow[i] = Usart1.RxData[8 + i * 2];      // 低字节
        ModBus.Slave.Rx.Data[i] = ((ModBus.Slave.Rx.DataHigh[i] << 8) | ModBus.Slave.Rx.DataLow[i]) & 0xFFFF; // 合并为 16 位数据
    }

    ModBus.Slave.Rx.CRCLow = (uint8_t)((Modbus_CRC16((uint8_t *)Usart1.RxData, 7 + dataCount * 2)) & 0xFF);
    ModBus.Slave.Rx.CRCHigh = (uint8_t)((Modbus_CRC16((uint8_t *)Usart1.RxData, 7 + dataCount * 2)) >> 8) & 0xFF;
}

/****************************************************************************************
* 函数名称：ModBus_SlaveReturnTx10
* 函数功能：Modbus 10H 命令成功接收后的确认返回帧
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveReturnTx10(void)
{
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD;
    Usart1.TxData[2] = ModBus.Slave.Rx.DataAddrHigh;
    Usart1.TxData[3] = ModBus.Slave.Rx.DataAddrLow;
    Usart1.TxData[4] = ModBus.Slave.Rx.DataCountHigh;
    Usart1.TxData[5] = ModBus.Slave.Rx.DataCountLow;
    Usart1.TxData[6] = (uint8_t)((Modbus_CRC16((uint8_t *)Usart1.TxData, 6)) & 0xFF);
    Usart1.TxData[7] = (uint8_t)((Modbus_CRC16((uint8_t *)Usart1.TxData, 6)) >> 8) & 0xFF;
    Usart1.Tx.Data = (uint8_t *)&Usart1.TxData[0];
    Usart1.Tx.DataSize = 8;
    Usart1TransmitterDMA(&Usart1.Tx);
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx10
* 函数功能：处理 Modbus 10H 命令，解析多寄存器写入、校验及数据存储
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveRx10(void)
{
    uint8_t byte_count = Usart1.RxData[6];
    uint16_t expected_len = 9 + byte_count;

    if (Usart1.DataCnt == expected_len) {
        uint16_t crc_received = ((uint16_t)Usart1.RxData[expected_len - 1] << 8) | Usart1.RxData[expected_len - 2];
        uint16_t crc_calc = Modbus_CRC16((uint8_t *)Usart1.RxData, expected_len - 2);

        if (crc_calc != crc_received) {
            ModBus_Slave_SendErrorResponse(0x01); // CRC 校验错误
        } else {
            ModBus_SlaveRx10DataCollation();
            uint16_t reg_count = ((uint16_t)ModBus.Slave.Rx.DataCountHigh << 8) | ModBus.Slave.Rx.DataCountLow;
            
            if ((ModBus.Slave.Rx.DataAddr + reg_count) <= MODBUS_REGISTER_COUNT) {
                for (uint16_t i = 0; i < reg_count; i++) {
                    ModBus.Slave.DisplayRegisters[ModBus.Slave.Rx.DataAddr + i] = ModBus.Slave.Rx.Data[i];
                }
                ModBus_SlaveReturnTx10();
            } else {
                ModBus_Slave_SendErrorResponse(0x02); // 非法地址
            }               
        }
    } else {
        ModBus_Slave_SendErrorResponse(0x03); // 长度错误
    }
}

/****************************************************************************************
* 函数名称：ModBus_SlaveRx
* 函数功能：根据接收到的 Modbus 帧解析命令并调用对应的处理函数
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void ModBus_SlaveRx(void)
{
    DisableUARTReceive(&huart1);
    ModBus.Slave.ADDR = Usart1.RxData[0];
    ModBus.Slave.CMD = Usart1.RxData[1];
    
    if(ModBus.Slave.ADDR == 3 ){ // 站地址检查
        switch(ModBus.Slave.CMD){
            case 0x03:
                ModBus_SlaveRx03();
            break;
            case 0x04:
                ModBus_SlaveRx04();
            break;
            case 0x06:
                ModBus_SlaveRx06();
            break;
            case 0x10:
                ModBus_SlaveRx10();
            break;
            default:
                ModBus_Slave_SendErrorResponse(0x05); // 功能码不支持
            break;
        }       
    }else{
        EnableUARTReceive(&huart1);
    }
}

/****************************************************************************************
* 函数名称：Usart1_ReceiveStringHandler
* 函数功能：处理通过串口接收到的字符串数据
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void Usart1_ReceiveStringHandler(void)
{
    DisableUARTReceive(&huart1);
    // 确保字符串以 '\0' 结尾
    Usart1.RxData[Usart1.DataCnt - 2] = '\0';  // 替换 '\r' 为字符串结束符
    Usart1.RxData[Usart1.DataCnt - 1] = '\0';  // 替换 '\n' 为字符串结束符
    Usart1.StringFlag = 1;
}

/****************************************************************************************
* 函数名称：Usart1_SendStringHandler
* 函数功能：根据接收到的字符串命令进行逻辑处理与响应
* 输入参量：无
* 输出参量：无
* 编写日期：2025-8-27
****************************************************************************************/
void Usart1_SendStringHandler(void)
{
    if(strcmp((char *)Usart1.RxData, "Board Status") == 0){
        Usart1_Print("Relay: K1:%s K2:%s K3:%s K4:%s K5:%s K6:%s K7:%s K8:%s\n",
                     Relay_GetStatus(1) ? "ON" : "OFF",
                     Relay_GetStatus(2) ? "ON" : "OFF",
                     Relay_GetStatus(3) ? "ON" : "OFF",
                     Relay_GetStatus(4) ? "ON" : "OFF",
                     Relay_GetStatus(5) ? "ON" : "OFF",
                     Relay_GetStatus(6) ? "ON" : "OFF",
                     Relay_GetStatus(7) ? "ON" : "OFF",
                     Relay_GetStatus(8) ? "ON" : "OFF");
    }else if(strcmp((char *)Usart1.RxData, "Board Info") == 0){
        Usart1_Print("MCU: STM32G491CCU6\n");
        Usart1_Print("FW: V2.0\n");
        Usart1_Print("HW: Encoder FCT Board V1.0\n");
        Usart1_Print("K1-K8 -> PA0-PA7\n");
    }else if(strcmp((char *)Usart1.RxData, "Firmware Update") == 0){
        IAP_RequestUpdate();
    }else if(strcmp((char *)Usart1.RxData, "Firmware version") == 0){
        Usart1_Print("V2.0\r\n");
    }else if(strcmp((char *)Usart1.RxData, "Relay AllOn") == 0){
        Relay_AllOn();
        Usart1_Print("OK\r\n");
    }else if(strcmp((char *)Usart1.RxData, "Relay AllOff") == 0){
        Relay_AllOff();
        Usart1_Print("OK\r\n");
    }else{
        EnableUARTReceive(&huart1);
    }
    
    memset((char *)Usart1.RxData, 0, sizeof(Usart1.RxData)); 
}

