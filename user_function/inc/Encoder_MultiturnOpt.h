#ifndef __ENCODER_MULTITURN_OPT_H
#define __ENCODER_MULTITURN_OPT_H

#include "main.h"
#include <stdint.h>

// ================= 测试项目枚举 =================
typedef enum {
    MulOpt_Test_Stop = 0,
    MulOpt_Test_MultiturnReset,
    MulOpt_Test_GetAllData,
    MulOpt_Test_ReadAllEeprom,
    MulOpt_Test_Initialize,
		MulOpt_Test_ReadAnalogDataTest,
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
    MulOpt_Test_AnalogData,
    MulOpt_Test_ReadInternalAlarm,
    MulOpt_Test_RMRNRSInitialize,
    MulOpt_Test_Sector,
    MulOpt_Test_MNSPosition,
    MulOpt_Test_MNSAnalogMaxCheck,
    MulOpt_Test_ReadAnalogData,
		MulOpt_Test_DACAdjustment,
    MulOpt_Test_Count
} MulOpt_TestItem_t;

// ================= 常量定义 =================
#define MULOPT_PAGE_NUMBER    4
#define MULOPT_EEPROM_ADDR    0x80
#define MULOPT_TX_SIZE        10
#define MULOPT_RX_SIZE        135
#define MULOPT_PNS_ADDRESS    0x7F

// ================= 状态码定义 =================
#define ENC_STATUS_TESTING     0
#define ENC_STATUS_PASS        1
#define ENC_STATUS_ERR_GLITCH  2
#define ENC_STATUS_ERR_JITTER  3
#define ENC_STATUS_ERR_COUNT   4

// ================= 物理->逻辑 映射表 =================
static const uint8_t MTAB_Map[4] = {3, 0, 2, 1}; // 物理 1,3,2,0 -> 逻辑 0,1,2,3
static const uint8_t HALL_Map[4] = {0, 3, 2, 1}; // 物理 0,3,2,1 -> 逻辑 0,1,2,3

// ================= SF 状态位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t dd0:1;  // bit0: fixed to "0"
        uint8_t dd1:1;  // bit1: fixed to "0"
        uint8_t dd2:1;  // bit2: fixed to "0"
        uint8_t dd3:1;  // bit3: fixed to "0"
        uint8_t ea0:1;  // bit4: CE
        uint8_t ea1:1;  // bit5: Logic-OR of OH, ME, BE, BA
        uint8_t ca0:1;  // bit6: Parity error
        uint8_t ca1:1;  // bit7: Delimiter error
    } bit;
} MulOpt_Status_t;

// ================= 报警标志位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t OS:1;  // bit0: Over speed (SA)
        uint8_t FS:1;  // bit1: Full absolute status (SA/SI)
        uint8_t CE:1;  // bit2: Counting error (SA/SI)
        uint8_t OF:1;  // bit3: Counter overflow (SA)
        uint8_t OH:1;  // bit4: Over heat (SA)
        uint8_t ME:1;  // bit5: Multi-turn error (SA)
        uint8_t BE:1;  // bit6: Battery error (SA)
        uint8_t BA:1;  // bit7: Battery alarm (SA)
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

// ================= EEPROM状态 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t Busy:1;
        uint8_t Write:1;
        uint8_t Read:1;
        uint8_t Res:5;
    } bit;
} MulOpt_EepromStatus_t;

// ================= M路信号数据 =================
typedef struct {
    uint16_t SinData;
    uint16_t CosData;
    uint16_t Absolute;
    uint16_t AbsoluteBuffer[2];
    int16_t AbsoluteDifference;
    int16_t Speed;
    int16_t FeedbackSpeed;
    uint8_t SpeedResult;
    uint8_t SpeedFlag;
    uint16_t SinDataMax;
    uint16_t SinDataMin;
    uint16_t CosDataMax;
    uint16_t CosDataMin;
    uint8_t PositionFlag;
    uint16_t MarkPositionBuffer[2];
    uint16_t MarkPositionDifference;
} MulOpt_EncM_t;

// ================= N路信号数据 =================
typedef struct {
    uint16_t SinData;
    uint16_t CosData;
    uint16_t Absolute;
    uint16_t AbsoluteBuffer[2];
    int16_t AbsoluteDifference;
    int16_t Speed;
    int16_t FeedbackSpeed;
    uint8_t SpeedResult;
    uint8_t SpeedFlag;
    uint16_t SinDataMax;
    uint16_t SinDataMin;
    uint16_t CosDataMax;
    uint16_t CosDataMin;
} MulOpt_EncN_t;

// ================= S路信号数据 =================
typedef struct {
    uint16_t SinData;
    uint16_t CosData;
    uint16_t Absolute;
    uint16_t AbsoluteBuffer[2];
    int16_t AbsoluteDifference;
    int16_t Speed;
    int16_t FeedbackSpeed;
    uint8_t SpeedResult;
    uint8_t SpeedFlag;
    uint16_t SinDataMax;
    uint16_t SinDataMin;
    uint16_t CosDataMax;
    uint16_t CosDataMin;
} MulOpt_EncS_t;

// ================= PNH2612 主结构 =================
typedef struct {
    uint8_t MTABSector;
    uint8_t MTABSectorraw;
    uint8_t HallSector;
    uint16_t MNdelta;
    uint8_t Q4D;
    uint16_t Q14D;
    uint32_t AbsolutePosition;
    
    // LED相关
    uint16_t LEDReadDAC;
    uint16_t LEDTestDAC;
    uint8_t LEDTestDACCnt;
    uint8_t LEDTestDACFlag;
    uint8_t LEDTestDACResult;
    uint16_t LEDAdjustDAC;
    uint8_t LEDAdjustDACCnt;
    int8_t LEDAdjustDACData;
    
    // 模拟量结果
    uint8_t MNSAnalogResult;
    uint8_t MNSAnalogSynchronousCheckResult;
    uint8_t MNSAnalogWaitCnt;
    uint16_t MNSAnalogChekcCnt;
    
    // 数据缓冲区
    uint16_t MSinDataBuffer[16];
    uint8_t LEDDAC0;
    uint8_t LEDDAC1;
    
    // 三路信号
    MulOpt_EncM_t M;
    MulOpt_EncN_t N;
    MulOpt_EncS_t S;
    
    // 其他
    uint32_t SamplingCnt;
    uint16_t MNSSinDataMaxBuffer[30];
    uint16_t MNSCosDataMaxBuffer[30];
		
		uint16_t MarkPositionBuffer[2];  // 添加位置标记缓冲区
    int16_t MarkPositionDifference;  // 添加位置差值
} MulOpt_PNH2612_t;

// ================= EEPROM结构 =================
typedef struct {
    uint8_t SetAddress;
    uint8_t ReturnAddress;
    uint8_t Address;
    uint8_t Data;
    uint8_t SetPage;
    uint8_t ReturnPage;
    uint8_t Page;
    uint8_t SetDNum;
    uint8_t ReturnDNum;
    MulOpt_EepromStatus_t Status;
    uint8_t DataBuffer[MULOPT_PAGE_NUMBER][MULOPT_EEPROM_ADDR];
} MulOpt_Eeprom_t;

// ================= HALL结构 =================
typedef struct {
    uint16_t Sector;
    uint8_t Buffer[4];
    uint8_t Result;  // 0:失败, 1:成功
    uint8_t Cnt;     // 记录实际走的步数
    uint8_t Check;   // 状态码
    uint16_t GlitchCnt;
    uint16_t JitterCnt;
    uint8_t CountErrCnt;
} MulOpt_Hall_t;

// ================= MTAB结构 =================
typedef struct {
    uint16_t Sector;
    uint8_t Buffer[4];
    uint8_t Result;
    uint8_t Cnt;
    uint8_t Check;
    uint16_t GlitchCnt;
    uint16_t JitterCnt;
    uint8_t CountErrCnt;
} MulOpt_MTAB_t;

// ================= 标志位 =================
typedef union {
    uint16_t all;
    struct {
        uint16_t CF02:1;
        uint16_t CF08:1;
        uint16_t CF92:1;
        uint16_t CF1A:1;
        uint16_t CF32:1;
        uint16_t CFEA:1;
        uint16_t CFBA:1;
        uint16_t CFC2:1;
        uint16_t CF62:1;
        uint16_t LedDacAdjust:1;
        uint16_t AMD:2;  // AnalogMaxData
        uint16_t LedDac:1;
        uint16_t MTABHALL:3;
        uint16_t MNSAMDOneLapCheck:2;
    } bit;
} MulOpt_Flag_t;

// ================= 计数器 =================
typedef struct {
    uint8_t CF02;
    uint8_t CF1A;
    uint8_t CF32;
    uint8_t CFEA;
    uint8_t CFBA;
    uint8_t CFC2;
    uint8_t CF62;
    uint8_t CFOIP;
    uint8_t CFCIP;
    uint8_t CFced;
} MulOpt_Counter_t;

// ================= 编码器主结构体 =================
typedef struct {
    // 通讯相关
    uint8_t TxID;
    uint8_t RxID;
    uint8_t IPID;
    
    // 状态信息
    MulOpt_Status_t Status;
    uint8_t OIPStatus;
    uint8_t ResolutionID;
    MulOpt_Alarm_t Alarm;
    MulOpt_Error_t Error;
    
    // 数据校验
    uint8_t XorCrcData;
    uint8_t XorCrcError;
    
    // 位置数据
    uint32_t SingleTurnPosition;
    uint16_t MultiTurnPosition;
    
    // 数据缓冲区
    uint8_t TxData[MULOPT_TX_SIZE];
    uint8_t RxData[MULOPT_RX_SIZE];
    uint8_t RxDataCnt;
    uint16_t TimeoutCnt;
    
    // EEPROM
    MulOpt_Eeprom_t Eeprom;
    
    // 标志位
    MulOpt_Flag_t Flag;
    
    // 计数器
    MulOpt_Counter_t Cnt;
    
    // 测试相关
    uint32_t Data;
    int16_t Speed;
    uint8_t Status0x6D;
    
    // 环境数据
    int8_t Temperature;
    uint8_t BatteryVoltage;
    
    // 报警信息
    uint8_t InternalAlarm1;
    uint8_t InternalAlarm2;
    uint8_t InternalAlarm3;
    
    // 测试结果
    uint8_t TestResult;
    
    // HALL/MTAB
    MulOpt_Hall_t Hall;
    MulOpt_MTAB_t MTAB;
} MulOpt_t;

// ================= 全局变量声明 =================
extern MulOpt_t g_MulOpt;
extern MulOpt_PNH2612_t g_PNH2612;

// ================= 接口函数 =================
void MulOpt_Init(void);
void MulOpt_TX(uint8_t cmd, uint8_t addr, uint8_t data);
void MulOpt_RxHandler(uint8_t byte);
void MulOpt_RxComplete(void);
void MulOpt_Test(uint8_t testItem);

// ================= Modbus 接口 =================
uint8_t  MulOpt_Modbus_IsMyAddr(uint16_t addr);
uint16_t MulOpt_Modbus_Read(uint16_t addr);
uint8_t  MulOpt_Modbus_Write(uint16_t addr, uint16_t value);

static void MultiturnOpticalEncoderHallCheck(void);
static void MultiturnOpticalEncoderMTABCheck(void);
static void MultiturnOpticalEncoderMNSSpeedCheck(void);
static void MultiturnOpticalEncoderMPositionMark(void);
static void MultiturnOpticalEncoderMPositionMark1(void);
static void MultiturnOpticalEncoderDACAdjustment(void);
static void MultiturnOpticalEncoderMNSAnalogMaxCheck(void);

#endif /* __ENCODER_MULTITURN_OPT_H */
