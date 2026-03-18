/****************************************************************************************
 * @file      Encoder_MultiturnMag.h
 * @brief     多圈磁编码器协议实现 (完整移植自 MultiturnMagneticEncoder)
 * @author    Gemini (移植优化)
 * @date      2026-02-09
 * @note      保持原有寄存器地址 (0x0200-0x0213) 和测试逻辑完全一致
 ****************************************************************************************/
#ifndef __ENCODER_MULTITURN_MAG_H
#define __ENCODER_MULTITURN_MAG_H

#include "main.h"



// ================= 常量定义 =================
#define MULENC_TX_SIZE          10
#define MULENC_RX_SIZE          135
#define MULENC_EEPROM_PAGES     6
#define MULENC_EEPROM_ADDRS     0x80
#define MULENC_PAGE_NUMBER      4
#define MULENC_PNS_ADDR         127

// ================= 测试项目枚举 =================
typedef enum {
    MulMag_Test_Stop = 0,
    MulMag_Test_ReadAllEeprom,      // 0x0200
    MulMag_Test_Initialize,         // 0x0201
    MulMag_Test_Hall,               // 0x0202
    MulMag_Test_WriteResult,        // 0x0203
    MulMag_Test_GetAllData,         // 0x0204
    MulMag_Test_Multiturn,          // 0x0205
    MulMag_Test_TB,                 // 0x0206
    MulMag_Test_OIP,                // 0x0207
    MulMag_Test_CIP,                // 0x0208
    MulMag_Test_SetResolution,      // 0x0209
    MulMag_Test_ReadResolution,     // 0x020A
    MulMag_Test_WriteDate,
    MulMag_Test_WriteHWRev,
    MulMag_Test_WriteRegister,
    MulMag_Test_ReadRegister,
    MulMag_Test_Speed,
    MulMag_Test_FirmwareVersion,
    MulMag_Test_Count
} MulMag_TestItem_t;

// ================= SF 状态位 (原有定义) =================
typedef struct {
    uint8_t dd0:1;   // bit0: fixed to "0"
    uint8_t dd1:1;   // bit1: fixed to "0"
    uint8_t dd2:1;   // bit2: fixed to "0"
    uint8_t dd3:1;   // bit3: fixed to "0"
    uint8_t ea0:1;   // bit4: CE
    uint8_t ea1:1;   // bit5: Logic-OR of OH, ME, BE, BA
    uint8_t ca0:1;   // bit6: Parity error
    uint8_t ca1:1;   // bit7: Delimiter error
} strMULMGTSF;

typedef union {
    uint8_t all;
    strMULMGTSF bit;
} MulMag_Status_t;

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
} MulMag_Alarm_t;

// ================= 错误标志位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t DC:1;    // 通讯超时
        uint8_t EC:1;    // CRC 错误
        uint8_t F0:1;    // 需要返回0xF0,实际没有			
        uint8_t reserved:5;
    } bit;
} MulMag_Error_t;

// ================= 计数器结构 =================
typedef struct {
    uint8_t CF62;
    uint8_t CF25;
    uint8_t CFCED;
    uint8_t CFOIP;
} MulMag_Cnt_t;

// ================= Hall 结构 =================
typedef struct {
  uint8_t   Sector;
  uint8_t   Buffer[4];
  uint8_t   Result; 
  uint8_t   Cnt;
  uint8_t   Check;
} MulMag_Hall_t;

// ================= EEPROM 管理结构 =================
typedef struct {
    uint8_t SetAddress;
    uint8_t ReturnAddress;
    uint8_t SetPage;
    uint8_t ReturnPage;
    uint8_t SetDNum;
    uint8_t Status;
		uint8_t ReturnDNum;
    uint8_t DataBuffer[MULENC_EEPROM_PAGES][MULENC_EEPROM_ADDRS];
} MulMag_Eeprom_t;

// ================= MT6835 寄存器结构 =================
typedef struct {
    uint8_t Reg0x11;
    uint8_t Reg0xDA;
    uint8_t Reg0xEA;
    uint8_t Reg0xEC;
    uint8_t Reg0x12;

    uint8_t Reg0x11_True;
    uint8_t Reg0xDA_True;
    uint8_t Reg0xEA_True;
    uint8_t Reg0xEC_True;
    uint8_t Reg0x12_True;
    union {
        uint8_t all;
        struct {
            uint8_t Bit11:1;
            uint8_t BitDA:1;
            uint8_t BitEA:1;
            uint8_t BitEC:1;
            uint8_t Bit12:1;
            uint8_t res:3;
        } bit;
    } RegBit;
    uint8_t TestRegData;
} MT6835_Addr_t;

// ================= 编码器主结构体 =================
typedef struct {
    volatile uint8_t TxData[MULENC_TX_SIZE];
    volatile uint8_t RxData[MULENC_RX_SIZE];
    volatile uint8_t RxDataCnt;
    volatile uint8_t TxID;
    volatile uint8_t RxID;
    
    uint32_t SingleTurnPos;
    uint16_t MultiTurnPos;
    uint8_t ResolutionID;
    uint8_t ReadResolutionID;
    
    volatile MulMag_Status_t Status;
    volatile MulMag_Alarm_t Alarm;
    volatile uint8_t InternalAlarm1;
    volatile uint8_t InternalAlarm2;
    volatile uint8_t InternalAlarm3;
    
    volatile MulMag_Error_t Error;
    volatile uint8_t Status0x6D;
    
    volatile uint8_t CrcData;
    volatile uint8_t CrcError;
    volatile uint16_t TimeoutCnt;
    volatile uint16_t TimeoutCnt1;    
    MulMag_Eeprom_t Eeprom;
    uint8_t IPID;
    MulMag_Hall_t Hall;
    MulMag_Cnt_t Cnt;
    
    uint16_t BatteryVoltage;
    uint16_t Temperature;
} MulMag_t;


// ================= 全局变量声明 =================
extern MulMag_t g_MulMag;

extern MT6835_Addr_t g_MT6835Addr;

// ================= 接口函数 =================
void MulMag_Init(void);
void MulMag_TX(uint8_t cmd, uint16_t addr, uint8_t data);
void MulMag_TxRegister(uint8_t cmd, uint8_t status, uint8_t addr, uint8_t data);
void MulMag_RxHandler(uint8_t byte);
void MulMag_RxComplete(void);  // RX 完成后调用，做 CRC 校验和错误处理
void MulMag_Test(uint8_t testItem);

// ================= 各测试项实现 =================
void MulMag_MultiturnReset(void);
void MulMag_GetAllData(void);
void MulMag_ReadAllEeprom(void);
void MulMag_Initialize(void);
void MulMag_HallRead(void);
void MulMag_WriteResult(void);
void MulMag_TB(void);
void MulMag_OpenInternalProtocol(void);
void MulMag_CloseInternalProtocol(void);
void MulMag_WriteDate(void);
void MulMag_SetResolution(void);
void MulMag_ReadResolution(void);
void MulMag_WriteHWRev(void);
void MulMag_WriteRegister(void);
void MulMag_ReadRegister(void);

// ================= Modbus 接口 (返回值用于判断是否成功) =================
uint8_t  MulMag_Modbus_IsMyAddr(uint16_t addr);
uint16_t MulMag_Modbus_Read(uint16_t addr);
uint8_t  MulMag_Modbus_Write(uint16_t addr, uint16_t value);  // 返回0成功，非0错误

// ================= 错误处理 (返回报警码给上位机) =================
uint8_t MulMag_GetWorkAlarm(void);
uint8_t MulMag_HasError(void);
void MulMag_ClearError(void);

// ================= Hall及转速检测 =================
void MulMag_HallCWCheck(void);
void MulMag_HallCCWCheck(void);
void MulMag_HallCheck(void);

#endif /* __ENCODER_MULTITURN_MAG_H */
