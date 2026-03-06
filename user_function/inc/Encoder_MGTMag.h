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
    MGT_Test_MultiturnReset,        // 1  多圈复位 (0x62)
    MGT_Test_GetAllData,            // 2  获取全部数据 (0x1A)
    MGT_Test_ReadAllEeprom,         // 3  读所有EEPROM
    MGT_Test_Initialize,            // 4  初始化 (0xDA)
    MGT_Test_Speed,                 // 5  速度测试
    MGT_Test_FirmwareVersion,       // 6  固件版本号 (0xA2@0x0305)
    MGT_Test_WriteAllEeprom,        // 7  写所有EEPROM
    MGT_Test_Command0x32,           // 8  EEPROM单字节写 (0x32)
    MGT_Test_Command0xEA,           // 9  EEPROM单字节读 (0xEA)
    MGT_Test_OIP,                   // 10 OpenInternalProtocol
    MGT_Test_CIP,                   // 11 CloseInternalProtocol
    MGT_Test_Command0x7A,           // 12 EEPROM 16位地址写 (0x7A)
    MGT_Test_Command0xA2,           // 13 EEPROM 16位地址读 (0xA2)
    MGT_Test_Firmware,              // 14 读固件信息 (0x40/0x10)
    MGT_Test_WriteDate,             // 15 写日期
    MGT_Test_SetResolution,         // 16 设置分辨率 (0x40/0x46)
    MGT_Test_ReadResolution,        // 17 读取分辨率 (0x40/0x45)
    MGT_Test_WriteHWRev,            // 18 写硬件版本
    MGT_Test_ReadHWRev,             // 19 读硬件版本
    MGT_Test_Timeout,               // 20 超时测试 (0x02)
    MGT_Test_Count
} MGT_TestItem_t;

// ================= 常量定义 =================
#define MGT_PAGE_NUMBER         6       // 原始为 6 页
#define MGT_EEPROM_ADDR         0x80    // 每页 128 字节
#define MGT_PNS_ADDRESS         0x7F    // PageNumberStorageAddress
#define MGT_TX_SIZE             5
#define MGT_RX_SIZE             12

// ================= 状态标志位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t dd0:1;      // bit0: fixed to 0
        uint8_t dd1:1;      // bit1: fixed to 0
        uint8_t dd2:1;      // bit2: fixed to 0
        uint8_t dd3:1;      // bit3: fixed to 0
        uint8_t ea0:1;      // bit4: CE
        uint8_t ea1:1;      // bit5: Logic-OR of OH, ME, BE, BA
        uint8_t ca0:1;      // bit6: Parity error
        uint8_t ca1:1;      // bit7: Delimiter error
    } bit;
} MGT_Status_t;

// ================= 告警标志位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t OS:1;   // bit0: Over speed
        uint8_t FS:1;   // bit1: Full absolute status
        uint8_t CE:1;   // bit2: Counting error
        uint8_t OF:1;   // bit3: Counter overflow
        uint8_t OH:1;   // bit4: Over heat
        uint8_t ME:1;   // bit5: Multi-turn error
        uint8_t BE:1;   // bit6: Battery error
        uint8_t BA:1;   // bit7: Battery alarm
    } bit;
} MGT_Alarm_t;

// ================= 错误标志位 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t DC:1;    // 通讯超时
        uint8_t EC:1;    // CRC 错误
        uint8_t reserved:6;
    } bit;
} MGT_Error_t;

// ================= EEPROM 状态位域 =================
typedef union {
    uint8_t all;
    struct {
        uint8_t Busy:1;
        uint8_t Write:1;
        uint8_t Read:1;
        uint8_t Res:5;
    } bit;
} MGT_EepromStatus_t;

// ================= EEPROM 管理结构 =================
typedef struct {
    uint8_t         SetAddress;
    uint8_t         ReturnAddress;
    uint16_t        Address;           // 16位地址 (0x7A/0xA2 命令使用)
    uint8_t         Data;
    uint8_t         SetPage;
    uint8_t         ReturnPage;
    MGT_EepromStatus_t Status;         // 含 Busy/Write/Read 位域
    uint8_t         DataBuffer[MGT_PAGE_NUMBER][MGT_EEPROM_ADDR];
} MGT_Eeprom_t;

// ================= 计数器结构 =================
typedef struct {
    uint8_t CF02;
    uint8_t CF1A;
    uint8_t CF32;
    uint8_t CFEA;
    uint8_t CFBA;
    uint8_t CFC2;
    uint8_t CF62;
    uint8_t CFDA;
    uint8_t CFOIP;
    uint8_t CFCIP;
} MGT_Cnt_t;

// ================= 编码器主结构体 =================
typedef struct {
    uint8_t         TxID;
    uint8_t         RxID;
    MGT_Status_t    Status;
    uint8_t         ResolutionID;
    MGT_Alarm_t     Alarm;
    uint8_t         XorCrcData;
    uint32_t        SingleTurnPosition;
    uint16_t        MultiTurnPosition;
    uint8_t         TxData[MGT_TX_SIZE];
    uint8_t         RxData[MGT_RX_SIZE];
    uint8_t         RxDataCnt;
    uint8_t         TimeoutCnt;
    uint8_t         XorCrcError;
    uint8_t         CrcData;
    
    MGT_Eeprom_t    Eeprom;
    MGT_Cnt_t       Cnt;
    
    uint32_t        Data;
    uint8_t         FW;    // 0xA2@0x0305 返回
    uint8_t         OIPStatus;          // 0x6D 返回的 OIP 状态
    uint32_t        Firmware;           // 0x40/0x10 返回的固件信息
    uint8_t         TimeOutFlag;
    
    MGT_Error_t     Error;              // DC/EC 错误标志
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
uint32_t MGT_Modbus_Read(uint16_t addr);
uint8_t  MGT_Modbus_Write(uint16_t addr, uint16_t value);

#endif /* __ENCODER_MGTMAG_H */
