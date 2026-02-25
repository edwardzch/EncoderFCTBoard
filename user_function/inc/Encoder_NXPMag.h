/****************************************************************************************
 * @file      Encoder_NXPMag.h
 * @brief     NXP 磁编码器协议实现 (移植自 NXPMagneticEncoder.h/c)
 ****************************************************************************************/
#ifndef __ENCODER_NXPMAG_H
#define __ENCODER_NXPMAG_H

#include "main.h"

// ================= 测试项目枚举 =================
typedef enum {
    NXP_Test_Stop = 0,
    NXP_Test_MultiturnReset,
    NXP_Test_GetAllData,
    NXP_Test_ReadAllEeprom,
    NXP_Test_Initialize,
    NXP_Test_Speed,
    NXP_Test_FirmwareVersion,
    NXP_Test_Count
} NXP_TestItem_t;

// ================= 常量定义 =================
#define NXP_PAGE_NUMBER      4
#define NXP_EEPROM_ADDR      0x80
#define NXP_TX_SIZE          4
#define NXP_RX_SIZE          16

// ================= SF 状态位 =================
typedef struct {
    uint8_t dd0:1;
    uint8_t dd1:1;
    uint8_t dd2:1;
    uint8_t dd3:1;
    uint8_t ea0:1;
    uint8_t ea1:1;
    uint8_t ca0:1;
    uint8_t ca1:1;
} NXP_SF_Bits_t;

typedef union {
    uint8_t all;
    NXP_SF_Bits_t bit;
} NXP_Status_t;

// ================= ALMC 报警位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t OS:1;
        uint8_t FS:1;
        uint8_t CE:1;
        uint8_t OF:1;
        uint8_t OH:1;
        uint8_t ME:1;
        uint8_t BE:1;
        uint8_t BA:1;
    } bit;
} NXP_Alarm_t;

// ================= EEPROM 结构 =================
typedef struct {
    uint8_t SetAddress;
    uint8_t ReturnAddress;
    uint8_t SetPage;
    uint8_t ReturnPage;
    uint8_t Busy;
    uint8_t DataBuffer[NXP_PAGE_NUMBER][NXP_EEPROM_ADDR];
} NXP_Eeprom_t;

// ================= 计数器结构 =================
typedef struct {
    uint16_t CF62;
    uint16_t CFDA;
    uint16_t CFEA;
} NXP_Cnt_t;

// ================= 编码器主结构体 =================
typedef struct {
    uint8_t TxID;
    uint8_t RxID;
    NXP_Status_t Status;
    uint8_t ResolutionID;
    NXP_Alarm_t Error;
    uint8_t XorCrcData;
    uint8_t XorCrcError;
    uint32_t SingleTurnPosition;
    uint16_t MultiTurnPosition;
    uint8_t TxData[NXP_TX_SIZE];
    uint8_t RxData[NXP_RX_SIZE];
    uint8_t RxDataCnt;
    uint16_t TimeoutCnt;
    NXP_Eeprom_t Eeprom;
    NXP_Cnt_t Cnt;
    uint16_t FWVersion;
} NXP_t;

// ================= 全局变量声明 =================
extern NXP_t g_NXP;

// ================= 接口函数 =================
void NXP_Init(void);
void NXP_TX(uint8_t cmd, uint16_t addr, uint8_t data);
void NXP_RxHandler(uint8_t byte);
void NXP_RxComplete(void);
void NXP_Test(uint8_t testItem);

// ================= Modbus 接口 (0x0500-0x05FF) =================
uint8_t  NXP_Modbus_IsMyAddr(uint16_t addr);
uint16_t NXP_Modbus_Read(uint16_t addr);
uint8_t  NXP_Modbus_Write(uint16_t addr, uint16_t value);

#endif /* __ENCODER_NXPMAG_H */
