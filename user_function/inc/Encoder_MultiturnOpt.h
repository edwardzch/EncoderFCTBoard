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
    MulOpt_Test_AnalogData,           // 0x0302: 6路模拟信号采集
    MulOpt_Test_ReadInternalAlarm,    // 0x030B: 读取内部报警
    MulOpt_Test_RMRNRSInitialize,     // 0x030C: RMRNRS初始化
    MulOpt_Test_Sector,               // 0x030F: MTAB/HALL扇区测试
    MulOpt_Test_MNSPosition,          // 0x0310: MNS位置读取
    MulOpt_Test_MNSAnalogMaxCheck,    // 0x0312: MNS模拟最大值检测
    MulOpt_Test_ReadAnalogData,       // 0x0313: 读取模拟数据
    MulOpt_Test_Count
} MulOpt_TestItem_t;

// ================= 常量定义 =================
#define MULOPT_PAGE_NUMBER    4
#define MULOPT_EEPROM_ADDR    0x80
#define MULOPT_TX_SIZE        10
#define MULOPT_RX_SIZE        135

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

// ================= 报警标志位 =================
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
} MulOpt_Alarm_t;

// ================= 错误标志位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t DC:1;    // 通讯超时
        uint8_t EC:1;    // CRC 错误
        uint8_t reserved:6;
    } bit;
} MulOpt_Error_t;

// ================= 编码器主结构体 =================
typedef struct {
    uint8_t TxID;
    uint8_t RxID;
    MulOpt_Status_t Status;
	  MulOpt_Alarm_t Alarm;
    uint8_t ResolutionID;
    MulOpt_Error_t Error;
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
    uint8_t Status0x6D;
    uint8_t TestResult;
    
    // Hall/MTAB 扩展 (移植原项目)
    uint8_t HallBuffer[4];
    uint16_t HallSector;
    uint8_t MTABBuffer[4];
    uint16_t MTABSector;
    uint8_t MTABResult;
    uint8_t MTABCheck;
    uint16_t MTABGlitchCnt;
    uint16_t MTABJitterCnt;
    uint16_t MTABCountErrCnt;
    uint8_t MTABHALLFlag;
    uint8_t MPositionFlag;
    
    // 内部报警
    uint8_t InternalAlarm1;
    uint8_t InternalAlarm2;
    uint8_t InternalAlarm3;
    
    // 光强/DAC
    uint16_t LightIntensity;
    uint16_t DACValue;
    uint16_t LEDTestDAC;
    uint16_t LEDTestDACResult;
    uint16_t MFeedbackSpeed;
    uint16_t MNSAnalogSyncResult;
    
    // 电池/温度
    uint16_t Battery;
    uint16_t Temperature;
    
    // EEPROM
    uint8_t EepromPage;
    uint8_t EepromAddr;
    uint8_t EepromData[MULOPT_PAGE_NUMBER][MULOPT_EEPROM_ADDR];
    
    // 计数器
    uint16_t CF62;
    uint16_t CFCED;
    uint16_t CFOIP;
    
    // 标志位
    union {
        uint8_t all;
        struct {
            uint8_t AMD:2;       // 模拟数据模式
            uint8_t LedDac:1;    // LED DAC标志
            uint8_t MTABHALL:2;  // MTAB/HALL标志
            uint8_t res:3;
        } bit;
    } Flag;
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
