/****************************************************************************************
 * @file      Encoder_SensAR.h
 * @brief     SensAR 编码器协议实现 (ASCII 协议)
 ****************************************************************************************/
#ifndef __ENCODER_SENSAR_H
#define __ENCODER_SENSAR_H

#include "main.h"

// ================= 测试项目枚举 =================
typedef enum {
    SensAR_Test_Stop = 0,
    SensAR_Test_Enter,
    SensAR_Test_GetPrdInfo,
    SensAR_Test_GetHallValue1,
    SensAR_Test_GetHallValue2,
    SensAR_Test_GetHallValue3,
    SensAR_Test_GetHallValue4,
    SensAR_Test_GetHallValue5,
    SensAR_Test_GetHallValue6,
    SensAR_Test_GetHallValue7,
    SensAR_Test_SetPrdInfo,
    SensAR_Test_Version,
    SensAR_Test_Count
} SensAR_TestItem_t;

// ================= 常量定义 =================
#define SENSAR_TX_SIZE       128
#define SENSAR_RX_SIZE       256

// ================= 标志位 =================
typedef union {
    uint16_t all;
    struct {
        uint8_t Enter:1;
        uint8_t getprdinfo:1;
        uint8_t gethallvalue1:1;
        uint8_t gethallvalue2:1;
        uint8_t gethallvalue3:1;
        uint8_t gethallvalue4:1;
        uint8_t gethallvalue5:1;
        uint8_t gethallvalue6:1;
        uint8_t gethallvalue7:1;
        uint8_t setprdinfo:1;
        uint8_t ver:1;
        uint8_t res:5;
    } bit;
} SensAR_Flag_t;

// ================= 编码器主结构体 =================
typedef struct {
    char TxData[SENSAR_TX_SIZE];
    uint8_t RxData[SENSAR_RX_SIZE];
    uint8_t RxDataCnt;
    uint8_t TxDataCnt;
    uint8_t CrcData;
    uint8_t CycleFlag;
    SensAR_Flag_t Flag;
    uint8_t TestContent;
    
    // Hall 值
    uint16_t Hall_1;
    uint16_t Hall_2;
    uint16_t Hall_3;
    uint16_t Hall_4;
    uint16_t Hall_5;
    uint16_t Hall_6;
    uint16_t Hall_7;
    
    // 产品信息
    char PrdInfo[64];
    char Version[32];
} SensAR_t;

// ================= 全局变量声明 =================
extern SensAR_t g_SensAR;

// ================= 接口函数 =================
void SensAR_Init(void);
uint8_t SensAR_Crc(const char *data, uint8_t len);
void SensAR_TX(const char *cmd);
void SensAR_RxHandler(uint8_t byte);
void SensAR_RxComplete(void);
void SensAR_Test(uint8_t testItem);

// ================= Modbus 接口 (0x0700-0x07FF) =================
uint8_t  SensAR_Modbus_IsMyAddr(uint16_t addr);
uint16_t SensAR_Modbus_Read(uint16_t addr);
uint8_t  SensAR_Modbus_Write(uint16_t addr, uint16_t value);

#endif /* __ENCODER_SENSAR_H */
