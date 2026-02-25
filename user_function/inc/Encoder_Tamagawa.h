/****************************************************************************************
 * @file      Encoder_Tamagawa.h
 * @brief     多摩川编码器协议实现 (移植自 TamagawaEncoder.h/c)
 * @author    Gemini (移植优化)
 * @date      2026-02-09
 ****************************************************************************************/
#ifndef __ENCODER_TAMAGAWA_H
#define __ENCODER_TAMAGAWA_H

#include "main.h"

// ================= 测试项目枚举 =================
typedef enum {
    Tmgw_Test_Stop = 0,
    Tmgw_Test_MultiturnReset,
    Tmgw_Test_GetAllData,
    Tmgw_Test_ReadMTP,
    Tmgw_Test_SingleTurnReset,
    Tmgw_Test_ReadMTPDefine,
    Tmgw_Test_Count
} Tmgw_TestItem_t;

// ================= 常量定义 =================
#define TMGW_PAGE_NUMBER      6
#define TMGW_EEPROM_ADDR      0x80
#define TMGW_TX_SIZE          4
#define TMGW_RX_SIZE          16
#define TMGW_PNS_ADDR         0x7F

// ================= SF 状态位 =================
typedef struct {
    uint8_t dd0:1;   // bit0: fixed "0"
    uint8_t dd1:1;   // bit1: fixed "0"
    uint8_t dd2:1;   // bit2: fixed "0"
    uint8_t dd3:1;   // bit3: fixed "0"
    uint8_t ea0:1;   // bit4: CE
    uint8_t ea1:1;   // bit5: Logic-OR of OH, ME, BE, BA
    uint8_t ca0:1;   // bit6: Parity error
    uint8_t ca1:1;   // bit7: Delimiter error
} strTmgwSF;

typedef union {
    uint8_t all;
    strTmgwSF bit;
} Tmgw_Status_t;

// ================= ALMC 报警位 =================
typedef struct {
    uint8_t OS:1;    // bit0: Over speed
    uint8_t FS:1;    // bit1: Full absolute status
    uint8_t CE:1;    // bit2: Counting error
    uint8_t OF:1;    // bit3: Counter overflow
    uint8_t OH:1;    // bit4: Over heat
    uint8_t ME:1;    // bit5: Multi-turn error
    uint8_t BE:1;    // bit6: Battery error
    uint8_t BA:1;    // bit7: Battery alarm
} strTmgwALMC;

typedef union {
    uint8_t all;
    strTmgwALMC bit;
} Tmgw_Alarm_t;

// ================= EEPROM 结构 =================
typedef struct {
    uint8_t SetAddress;
    uint8_t ReturnAddress;
    uint8_t SetPage;
    uint8_t ReturnPage;
    uint8_t Status;
    uint8_t DataBuffer[TMGW_PAGE_NUMBER][TMGW_EEPROM_ADDR];
} Tmgw_Eeprom_t;

// ================= 计数器结构 =================
typedef struct {
    uint16_t CF02;
    uint16_t CF1A;
    uint16_t CF32;
    uint16_t CFEA;
    uint16_t CFBA;
    uint16_t CFC2;
    uint16_t CF62;
} Tmgw_Cnt_t;

// ================= 编码器主结构体 =================
typedef struct {
    uint8_t TxID;
    uint8_t RxID;
    Tmgw_Status_t Status;
    uint8_t ResolutionID;
    Tmgw_Alarm_t Error;
    uint8_t XorCrcData;
    uint8_t XorCrcError;
    uint32_t SingleTurnPosition;
    uint16_t MultiTurnPosition;
    uint8_t TxData[TMGW_TX_SIZE];
    uint8_t RxData[TMGW_RX_SIZE];
    uint8_t RxDataCnt;
    uint16_t TimeoutCnt;
    Tmgw_Eeprom_t Eeprom;
    Tmgw_Cnt_t Cnt;
    uint32_t Resolution;
    uint32_t Phase;
    uint32_t SinglePoleResolution;
    uint16_t PhaseAngle;
} Tmgw_t;

// ================= MTP 电机参数结构 =================
typedef struct {
    uint8_t EPSDriverType;
    uint8_t EPSMotorType;
    uint8_t EPSEncoderType;
    uint8_t MotorType;
    uint8_t EncoderType;
    uint8_t VoltageType;
    uint16_t MotorPower;
    uint16_t MultiTurnPosition;
    uint16_t RatedSpeed;
    uint16_t MaximumSpeed;
    uint8_t NumberOfPolePairs;
    uint8_t OverSpeedLevel;
    float RatedTorque;
    uint16_t MaximumTorque;
    float RatedPeakCurrent;
    float InstantaneousMaximumPeakCurrent;
    float EMFConstant;
    uint16_t RotorInertia;
    float ArmatureWindingResistance;
    float ArmatureInductance;
    uint16_t PhaseLow;
    uint16_t PhaseHigh;
    uint16_t CrcData;
    uint16_t CrcDataCalculate;
} Tmgw_MTP_t;

// ================= 全局变量声明 =================
extern Tmgw_t g_Tmgw;
extern Tmgw_MTP_t g_TmgwMTP;

// ================= 接口函数 =================
void Tmgw_Init(void);
void Tmgw_TX(uint8_t cmd, uint16_t addr, uint8_t data);
void Tmgw_RxHandler(uint8_t byte);
void Tmgw_RxComplete(void);
void Tmgw_Test(uint8_t testItem);

// ================= 各测试项 =================
void Tmgw_MultiturnReset(void);
void Tmgw_SingleTurnReset(void);
void Tmgw_GetAllData(void);
void Tmgw_ReadMTP(void);
void Tmgw_ReadMTPDefine(void);

// ================= Modbus 接口 =================
uint8_t  Tmgw_Modbus_IsMyAddr(uint16_t addr);
uint16_t Tmgw_Modbus_Read(uint16_t addr);
uint8_t  Tmgw_Modbus_Write(uint16_t addr, uint16_t value);

#endif /* __ENCODER_TAMAGAWA_H */
