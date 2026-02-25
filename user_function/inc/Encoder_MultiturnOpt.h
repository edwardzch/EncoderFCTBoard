/****************************************************************************************
 * @file      Encoder_MultiturnOpt.h
 * @brief     多圈光编码器协议框架 (移植自 MultiturnOpticalEncoder.c, 1530行)
 ****************************************************************************************/
#ifndef __ENCODER_MULTITURN_OPT_H
#define __ENCODER_MULTITURN_OPT_H

#include "main.h"

// ================= 测试项目枚举 =================
typedef enum {
    MulOpt_Test_Stop = 0,
    MulOpt_Test_MultiturnReset,
    MulOpt_Test_GetAllData,
    MulOpt_Test_ReadAllEeprom,
    MulOpt_Test_Initialize,
    MulOpt_Test_WriteResult,
    MulOpt_Test_TB,
    MulOpt_Test_DAC,
    MulOpt_Test_LightIntensity,
    MulOpt_Test_Speed,
    MulOpt_Test_ReadHWRev,
    MulOpt_Test_ReadFWRev,
    MulOpt_Test_Hall,
    MulOpt_Test_MTAB,
    MulOpt_Test_MNSAnalog,
    MulOpt_Test_WriteDate,
    MulOpt_Test_WriteHWRev,
    MulOpt_Test_OIP,
    MulOpt_Test_CIP,
    MulOpt_Test_Count
} MulOpt_TestItem_t;

// ================= 常量定义 =================
#define MULOPT_PAGE_NUMBER    4
#define MULOPT_EEPROM_ADDR    0x80
#define MULOPT_TX_SIZE        8
#define MULOPT_RX_SIZE        20

// ================= SF 状态位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t dd0:1;
        uint8_t dd1:1;
        uint8_t dd2:1;
        uint8_t dd3:1;
        uint8_t ea0:1;
        uint8_t ea1:1;
        uint8_t ca0:1;
        uint8_t ca1:1;
    } bit;
} MulOpt_Status_t;

// ================= 编码器主结构体 =================
typedef struct {
    uint8_t TxID;
    uint8_t RxID;
    MulOpt_Status_t Status;
    uint8_t ResolutionID;
    uint8_t Error;
    uint8_t CrcData;
    uint8_t CrcError;
    uint32_t SingleTurnPosition;
    uint16_t MultiTurnPosition;
    uint8_t TxData[MULOPT_TX_SIZE];
    uint8_t RxData[MULOPT_RX_SIZE];
    uint8_t RxDataCnt;
    uint16_t TimeoutCnt;
    
    // MNS 模拟量
    uint16_t MSinMax, MSinMin, MSin;
    uint16_t MCosMax, MCosMin, MCos;
    uint16_t NSinMax, NSinMin, NSin;
    uint16_t NCosMax, NCosMin, NCos;
    uint16_t SSinMax, SSinMin, SSin;
    uint16_t SCosMax, SCosMin, SCos;
    
    // Hall/MTAB
    uint8_t Hall;
    uint8_t MTAB;
    uint8_t HallResult;
    
    // 光强/DAC
    uint16_t LightIntensity;
    uint16_t DACValue;
    
    // 电池/温度
    uint16_t Battery;
    uint16_t Temperature;
    
    // EEPROM
    uint8_t EepromPage;
    uint8_t EepromAddr;
    uint8_t EepromData[MULOPT_PAGE_NUMBER][MULOPT_EEPROM_ADDR];
    
    // 计数器
    uint16_t CF62;
    uint16_t CFced;
    uint16_t CFOIP;
} MulOpt_t;

// ================= 全局变量声明 =================
extern MulOpt_t g_MulOpt;

// ================= 接口函数 =================
void MulOpt_Init(void);
void MulOpt_TX(uint8_t cmd, uint8_t addr, uint8_t data);
void MulOpt_RxHandler(uint8_t byte);
void MulOpt_RxComplete(void);
void MulOpt_Test(uint8_t testItem);

// ================= Modbus 接口 (0x0300-0x03FF) =================
uint8_t  MulOpt_Modbus_IsMyAddr(uint16_t addr);
uint16_t MulOpt_Modbus_Read(uint16_t addr);
uint8_t  MulOpt_Modbus_Write(uint16_t addr, uint16_t value);

#endif /* __ENCODER_MULTITURN_OPT_H */
