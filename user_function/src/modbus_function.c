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
#include "delay_function.h"
#include "encoder_modbus.h"  // 编码器测试模块
#include "encoder_driver.h"   // TIM1_Stop_Direct
#include "Encoder_MultiturnMag.h"  // g_MotorEncoder
#include "Parameter_Module.h"      // PA_Buffer
#include "blackbox.h"


extern UART_HandleTypeDef huart1;

volatile strModBus ModBus = {0};
volatile uint8_t Need_Reset_Board = 0, Need_MCU_Reset = 0;
volatile uint8_t Work_Alarm = 0;
/****************************************************************************************
* 函数名称：Modbus_ApplyConfig
* 函数功能：根据 PA 参数重新配置 Modbus 从机地址、波特率和校验位
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-26
****************************************************************************************/
void Modbus_ApplyConfig(void)
{
    // 1. 从机地址: PA000 (1-255)
    uint8_t addr = (uint8_t)PA_Buffer[0];
    if (addr < 1) addr = 5;  // 兜底默认值
    ModBus.Slave.ADDR = addr;
    
    // 2. 波特率: PA001 * 100
    uint32_t baud = (uint32_t)PA_Buffer[1] * 100;
    if (baud < 1200 || baud > 115200) baud = 57600;  // 兜底
    
    // 3. 校验位: PA002 (0=none, 1=odd, 2=even)
    uint32_t parity;
    uint32_t wordlen;
    switch (PA_Buffer[2]) {
        case 0:  // None
            parity = UART_PARITY_NONE;
            wordlen = UART_WORDLENGTH_8B;
            break;
        case 2:  // Even
            parity = UART_PARITY_EVEN;
            wordlen = UART_WORDLENGTH_9B;
            break;
        default: // Odd (默认)
            parity = UART_PARITY_ODD;
            wordlen = UART_WORDLENGTH_9B;
            break;
    }
    
    // 4. 重新配置 USART1
    huart1.Init.BaudRate = baud;
    huart1.Init.Parity = parity;
    huart1.Init.WordLength = wordlen;
    HAL_UART_Init(&huart1);
    
    // 5. 继电器控制: PA003(K1-K4) + PA004(K5-K8)
    uint8_t relay_mask = ((uint8_t)(PA_Buffer[4] & 0x0F) << 4) | ((uint8_t)(PA_Buffer[3] & 0x0F));
    Relay_SetByMask(relay_mask);
}

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
/*
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
*/

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
    BlackBox_Log(BB_a, (uint8_t *)Usart1.TxData, (uint8_t)Usart1.Tx.DataSize);
    Usart1TransmitterDMA(&Usart1.Tx);
}

/****************************************************************************************
* 函数名称：ModBus_EncoderReturnTx03
* 函数功能：处理编码器模块 03H 命令 16-bit 循环读取
* 输入参量：DataAddr 起始地址, DataSize 寄存器数量
* 编写日期：2026-03-06
****************************************************************************************/
void ModBus_EncoderReturnTx03(uint16_t DataAddr, uint16_t DataSize)
{
    uint16_t i;
    uint8_t data_bytes = DataSize * 2;
    uint8_t frame_len_no_crc = 3 + data_bytes;
    
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD;
    Usart1.TxData[2] = data_bytes;
    
    uint8_t has_error = 0;
    for (i = 0; i < DataSize; i++) {
        uint16_t regValue = EncoderModbus_ReadReg(DataAddr + i);
        if (regValue == 0xFFFF) {
            has_error = 1;
            break; // 获取失败
        }
        Usart1.TxData[3 + i * 2] = (uint8_t)(regValue >> 8);
        Usart1.TxData[4 + i * 2] = (uint8_t)(regValue & 0xFF);
    }
    
    if (!has_error) {
        uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, frame_len_no_crc);
        Usart1.TxData[frame_len_no_crc] = (uint8_t)(crc & 0xFF);
        Usart1.TxData[frame_len_no_crc + 1] = (uint8_t)(crc >> 8);
        
        Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
        Usart1.Tx.DataSize = frame_len_no_crc + 2;
        BlackBox_Log(BB_a, (uint8_t *)Usart1.TxData, (uint8_t)Usart1.Tx.DataSize);
        Usart1TransmitterDMA(&Usart1.Tx);
    }
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
            ModBus_Crc_Error(); // CRC 校验错误
        }else{
            // 检查是否为编码器模块地址 (0x01xx - 0x06xx)
            if (EncoderModbus_IsMyAddress(ModBus.Slave.Rx.DataAddr)) {
                // 常规的编码器 16-bit 循环读取
                ModBus_EncoderReturnTx03(ModBus.Slave.Rx.DataAddr, ModBus.Slave.Rx.DataSize);
                
            } else if (ModBus.Slave.Rx.DataAddr >= MODBUS_REG_BASE_ADDR && 
                       (ModBus.Slave.Rx.DataAddr - MODBUS_REG_BASE_ADDR + ModBus.Slave.Rx.DataSize) <= MODBUS_REGISTER_COUNT) {
                 ModBus_SlaveReturnTx03(ModBus.Slave.Rx.DataAddr - MODBUS_REG_BASE_ADDR, ModBus.Slave.Rx.DataSize);
            } else {
                 Communication_Address_Error(); // 非法数据地址
            } 
        }
    }else{
        Communication_Length_Error(); // 长度错误
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
            ModBus_Crc_Error(); // CRC 校验错误
        } else {
            if ((ModBus.Slave.Rx.DataAddr + ModBus.Slave.Rx.DataSize) <= MODBUS_REGISTER_COUNT) {                            
                for (i = 0; i < ModBus.Slave.Rx.DataSize; i++) {
                    if ((ModBus.Slave.Rx.DataSize + i) < MODBUS_REGISTER_COUNT) {
                        ModBus.Master.DisplayRegisters[ModBus.Slave.Rx.DataAddr + i] = Relay_GetStatus(ModBus.Slave.Rx.DataAddr + i + 1);
                    }
                }                            
                ModBus_SlaveReturnTx04(ModBus.Slave.Rx.DataAddr, ModBus.Slave.Rx.DataSize);
            } else {
                 Communication_Address_Error(); // 非法数据地址
            }                   
        }
    } else {
        Communication_Length_Error(); // 长度错误
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
            ModBus_Crc_Error(); // CRC 校验错误
        }else{
            // 检查是否为编码器模块地址 (0x00xx-0x06xx)
            if (EncoderModbus_IsMyAddress(ModBus.Slave.Rx.DataAddr)) {
                if (EncoderModbus_WriteReg(ModBus.Slave.Rx.DataAddr, ModBus.Slave.Rx.Data[0]) == 0) {
                    ModBus_SlaveReturnTx06();
                }
            }
            // 普通寄存器地址 (0x1000+)
            else if (ModBus.Slave.Rx.DataAddr >= MODBUS_REG_BASE_ADDR &&
                     (ModBus.Slave.Rx.DataAddr - MODBUS_REG_BASE_ADDR + 1) <= MODBUS_REGISTER_COUNT) {
                uint16_t idx = ModBus.Slave.Rx.DataAddr - MODBUS_REG_BASE_ADDR;
                ModBus.Slave.DisplayRegisters[idx] = ModBus.Slave.Rx.Data[0];
                
                // 继电器控制 (地址 0x1000-0x1009 触发硬件动作)
                switch (ModBus.Slave.Rx.DataAddr) {
                    case 0x1000:  // 按位掩码控制继电器 (bit0=K1 ... bit7=K8)
                        Relay_SetByMask((uint8_t)(ModBus.Slave.Rx.Data[0] & 0xFF));
                        break;
                    case 0x1001:  // 继电器1-8
                    case 0x1002:
                    case 0x1003:
                    case 0x1004:
                    case 0x1005:
                    case 0x1006:
                    case 0x1007:
                    case 0x1008:
                        if (ModBus.Slave.Rx.Data[0])
                            Relay_On(idx);
                        else
                            Relay_Off(idx);
                        break;
                    case 0x1009:  // 继电器逐个测试 (K1-K8 依次吸合1s后断开)
                        g_RelayTestActive = 1;
                        break;
                    case 0x100A:
                        Need_Reset_Board = 1;         // 第一步：只打标记
                        ModBus_SlaveReturnTx06();     // 第二步：触发 DMA 发送
                        return;
                    case 0x1010:
												if(ModBus.Slave.Rx.Data[0] == 0x5AA5) {
													Need_MCU_Reset = 1;         // 第一步：只打标记
													ModBus_SlaveReturnTx06();     // 第二步：触发 DMA 发送													
												}

                        return;										
                    default:
                        break;  // 其他地址只写寄存器，无硬件动作
                }
                ModBus_SlaveReturnTx06();
            } else {
                Communication_Address_Error(); // 非法数据地址
            }
        }
    }else{
        Communication_Length_Error(); // 长度错误
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
            ModBus_Crc_Error(); // CRC 校验错误
        } else {
            ModBus_SlaveRx10DataCollation();
            uint16_t reg_count = ((uint16_t)ModBus.Slave.Rx.DataCountHigh << 8) | ModBus.Slave.Rx.DataCountLow;
            
            if (ModBus.Slave.Rx.DataAddr >= MODBUS_REG_BASE_ADDR &&
                (ModBus.Slave.Rx.DataAddr - MODBUS_REG_BASE_ADDR + reg_count) <= MODBUS_REGISTER_COUNT) {
                uint16_t base_idx = ModBus.Slave.Rx.DataAddr - MODBUS_REG_BASE_ADDR;
                for (uint16_t i = 0; i < reg_count; i++) {
                    ModBus.Slave.DisplayRegisters[base_idx + i] = ModBus.Slave.Rx.Data[i];
                }
                ModBus_SlaveReturnTx10();
            } else {
                Communication_Length_Error(); // 长度错误
            }               
        }
    } else {
        Communication_Length_Error(); // 长度错误
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
	
    // 判断是否为本机地址 (Modbus)
    if (Usart1.RxData[0] == (uint8_t)PA_Buffer[0]) {	
				// 同步当前选中编码器模块的通讯底层错误标志
				EncoderModbus_UpdateErrors();

				// ================= 0. 白名单检查 =================
				uint8_t is_whitelist = 0;
				if (Usart1.DataCnt >= 4) { // 确保接收的数据长度足够提取地址
						uint8_t cmd = Usart1.RxData[1];
						uint16_t data_addr = ((uint16_t)Usart1.RxData[2] << 8) | Usart1.RxData[3];
						
						if (Usart1.RxData[0] == (uint8_t)PA_Buffer[0]) {
								// === 在此添加不受编码器通讯错误阻塞的指令白名单 ===
								if (cmd == MODBUS_FUNC_WRITE_SINGLE_REGISTER && data_addr == 0x100A) {
										is_whitelist = 1; // 100A: 重启板卡指令
								} else if (cmd == MODBUS_FUNC_WRITE_SINGLE_REGISTER && data_addr == 0x1000) {
										is_whitelist = 1; // 1000: 按位控制继电器 (如强行复位电源等)
								}
								// 可以继续使用 else if 增加其他白名单寄存器 
						}
				}

				// ================= 1. 优先检查错误标志 (用户逻辑) =================
				// 对于非白名单指令，DC/EC 错误优先级最高, 拦截指令
				if (!is_whitelist) {
						if (ModBus.Error.bit.DC) {
						ModBus.Error.bit.DC = 0;
								Encoder_Timeout();
								goto RX_END;
						} 
						if (ModBus.Error.bit.EC) {
						ModBus.Error.bit.EC = 0;
								Encoder_CrcError();
								goto RX_END;
						}
				
						if (ModBus.Error.bit.SN) {
						ModBus.Error.bit.SN = 0; // 清除 SN 错误
								SNDigitsAreIncorrect();
								goto RX_END;
						}
				} else {
						// 白名单放行，但也清空当前的错误位，防止堆积影响主流程，底层更新函数会再次获取最新状态
						ModBus.Error.bit.DC = 0;
						ModBus.Error.bit.EC = 0;
						ModBus.Error.bit.SN = 0;
				}

				// ================= 2. 正常接收处理 =================
				// 记录 FCT 上位机发给测试板的帧 (BB_A通道)
				BlackBox_Log(BB_A, (uint8_t *)Usart1.RxData, (uint8_t)Usart1.DataCnt);
				
				ModBus.Slave.ADDR = Usart1.RxData[0];
				ModBus.Slave.CMD = Usart1.RxData[1];

        switch (ModBus.Slave.CMD) {
            case MODBUS_FUNC_READ_HOLDING_REGISTERS: // 03H
                ModBus_SlaveRx03();
                break;
            case MODBUS_FUNC_WRITE_SINGLE_REGISTER:   // 06H
                ModBus_SlaveRx06();
                break;
            case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS: // 10H
                ModBus_SlaveRx10();
                break;
            default:
                Communication_FunctionCode_Error(); // 功能码不支持
                break;
        }
    }

RX_END:
    // 注意: 当前 ModBus_SlaveRx 在 USART1 IDLE 中断中同步执行。
    // 如果触发了任何响应帧 (如 SNDigitsAreIncorrect)，发送是 DMA 异步的，此时尚未发完。
    // 因此这里绝对不能调用 `EnableUARTReceive(&huart1)` 提前修改串口状态，
    // 也不必清空 RxData。
    // 1. DataCnt 的清零将在外层的 IDLE 中断处理末尾完成。
    // 2. RS485 接收使能 (EnableUARTReceive 和 PA8 切换) 将在 USART1 TX TC中断中处理。
    return;
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
	  Usart1.StringDataCnt = Usart1.DataCnt;
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
    // 错误检查已在 ModBus_SlaveRx 中完成

    // ================= 板卡通用命令 =================
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
        Usart1_Print("FW: V%0.1f\n", FirmwareVersion);
        Usart1_Print("HW: Encoder FCT Board V%0.1f\n", HardwareVersion);
        Usart1_Print("K1-K8 -> PA0-PA7\n");
    }else if(strcmp((char *)Usart1.RxData, "Firmware Update") == 0){
        IAP_RequestUpdate();
    }else if(strcmp((char *)Usart1.RxData, "Firmware version") == 0){
        Usart1_Print("STM32G4_APP_FCT_%0.1f\r\n", FirmwareVersion);
    }else if(strcmp((char *)Usart1.RxData, "Relay AllOn") == 0){
        Relay_AllOn();
        Usart1_Print("OK\r\n");
    }else if(strcmp((char *)Usart1.RxData, "Relay AllOff") == 0){
        Relay_AllOff();
        Usart1_Print("OK\r\n");
    }
    // ================= 3. 编码器 ASCII 命令 =================
    else if(strcmp((char *)Usart1.RxData, "EncoderPowerOff") == 0){
        TIM1_Stop_Direct(); 
        Usart1_Print("OK\r\n");
    }else if(strncmp((char *)Usart1.RxData, "SN:DL", 5) == 0){
        if(Usart1.StringDataCnt == 19){  
            g_MotorEncoder.HWRevData = Usart1.RxData[5] - '0';
            ModBus.Error.bit.SN = 0; // 清除 SN 错误
        }else{
            ModBus.Error.bit.SN = 1; // 设置 SN 错误
        }
				Usart1.StringDataCnt = 0;
				EnableUARTReceive(&huart1);
    }else if(strcmp((char *)Usart1.RxData, "GetHWRev") == 0){
        Usart1_Print("%s", g_MotorEncoder.HWRevBuffer);
    }else if(strcmp((char *)Usart1.RxData, "GetFWRev") == 0){
        Usart1_Print("%s", g_MotorEncoder.FWRevBuffer);
    }else if(strcmp((char *)Usart1.RxData, "GetTestContent") == 0){
        BlackBox_SendAll();
    }else{
        EnableUARTReceive(&huart1);
    }
    
    memset((char *)Usart1.RxData, 0, sizeof(Usart1.RxData)); 
}

// ================= 错误处理函数 (移植自旧项目) =================
// 注意: RTU_CRC 替换为 Modbus_CRC16, 串口相关使用了 Usart1TransmitterDMA

void Encoder_Timeout(void)
{   
    Work_Alarm = 1;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void Encoder_CrcError(void)
{
    Work_Alarm = 2;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void Communication_Address_Error(void)
{
    Work_Alarm = 3;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void ModBus_Crc_Error(void)
{
    Work_Alarm = 4;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void NotSetEncoderType(void)
{
    Work_Alarm = 5;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void SNDigitsAreIncorrect(void)
{
    Work_Alarm = 6;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void Communication_Length_Error(void)
{
    Work_Alarm = 0x07;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void Communication_FunctionCode_Error(void)
{
    Work_Alarm = 8; 
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void Communication_SlaveAddress_Error(void)
{
    Work_Alarm = 9;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}

void Encoder_F0Error(void)
{
    Work_Alarm = 10;
    Usart1.TxData[0] = ModBus.Slave.ADDR;
    Usart1.TxData[1] = ModBus.Slave.CMD | 0x80;            
    Usart1.TxData[2] = Work_Alarm | 0x50;
    uint16_t crc = Modbus_CRC16((uint8_t *)Usart1.TxData, 3);
    Usart1.TxData[3] = (uint8_t)(crc & 0xFF);
    Usart1.TxData[4] = (uint8_t)((crc >> 8) & 0xFF);
    
    Usart1.Tx.Data = (uint8_t *)Usart1.TxData;
    Usart1.Tx.DataSize = 5;
    Usart1TransmitterDMA(&Usart1.Tx);
}
