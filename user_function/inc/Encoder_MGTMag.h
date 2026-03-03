/****************************************************************************************
 * @file      Encoder_MGTMag.h
 * @brief     MGT 磁编码器协议框架 (移植自 MGTMagneticEncoder.c)
 ****************************************************************************************/
#ifndef __ENCODER_MGTMAG_H
#define __ENCODER_MGTMAG_H

#include "main.h"

// ================= 测试项目枚举 =================
typedef enum {
    MGT_Test_Stop = 0,
    MGT_Test_MultiturnReset,
    MGT_Test_GetAllData,
    MGT_Test_ReadAllEeprom,
    MGT_Test_Initialize,
    MGT_Test_WriteResult,
    MGT_Test_Hall,
    MGT_Test_Speed,
    MGT_Test_TB,
    MGT_Test_OIP,
    MGT_Test_CIP,
    MGT_Test_WriteDate,
    MGT_Test_WriteHWRev,
    MGT_Test_FirmwareVersion,
    MGT_Test_WriteAllEeprom,
    MGT_Test_Command0x32,
    MGT_Test_Command0xEA,
    MGT_Test_Command0x7A,
    MGT_Test_Command0xA2,
    MGT_Test_Firmware,
    MGT_Test_SetResolution,
    MGT_Test_ReadResolution,
    MGT_Test_ReadHWRev,
    MGT_Test_Timeout,
    MGT_Test_Count
} MGT_TestItem_t;

// ================= 常量定义 =================
#define MGT_PAGE_NUMBER    4
#define MGT_EEPROM_ADDR    0x80
#define MGT_TX_SIZE        5
#define MGT_RX_SIZE        12

// ================= 错误标志位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t DC:1;    // 通讯超时
        uint8_t EC:1;    // CRC 错误
        uint8_t reserved:6;
    } bit;
} MGT_Error_t;

// ================= 编码器主结构体 =================
typedef struct {
    uint8_t TxID;
    uint8_t RxID;
    uint8_t Status;
    uint8_t ResolutionID;
    uint8_t Alarm;
    MGT_Error_t Error;
    uint8_t CrcData;
    uint8_t CrcError;
    uint32_t SingleTurnPosition;
    uint16_t MultiTurnPosition;
    uint8_t TxData[MGT_TX_SIZE];
    uint8_t RxData[MGT_RX_SIZE];
    uint8_t RxDataCnt;
    uint16_t TimeoutCnt;
  
    uint8_t Status0x6D;
    
    uint8_t EepromPage;
    uint8_t EepromAddr;
    uint8_t EepromData[MGT_PAGE_NUMBER][MGT_EEPROM_ADDR];
    
    struct {
        uint8_t SetPage;
        uint8_t SetAddress;
        uint8_t ReturnPage;
        uint8_t ReturnAddress;
        uint8_t Data;
        uint8_t Address;
    } EepromParams;
    
    uint16_t FW;
    uint32_t Firmware;
    
    uint16_t CF62;
    uint16_t CFCED;
    uint16_t CFOIP;
    uint16_t CF25;
} MGT_t;

// ================= 全局变量声明 =================
extern MGT_t g_MGT;

// ================= 接口函数 =================
void MGT_Init(void);
void MGT_TX(uint8_t cmd, uint16_t addr, uint8_t data);
void MGT_RxHandler(uint8_t byte);
void MGT_RxComplete(void);
void MGT_Test(uint8_t testItem);

// ================= Modbus 接口 (0x0400-0x04FF) =================
uint8_t  MGT_Modbus_IsMyAddr(uint16_t addr);
uint16_t MGT_Modbus_Read(uint16_t addr);
uint8_t  MGT_Modbus_Write(uint16_t addr, uint16_t value);

#endif /* __ENCODER_MGTMAG_H */
