/****************************************************************************************
 * @file      encoder_modbus.h
 * @brief     编码器测试 Modbus 寄存器处理模块
 * @author    Gemini
 * @date      2026-02-09
 ****************************************************************************************/
#ifndef __ENCODER_MODBUS_H
#define __ENCODER_MODBUS_H

#include "main.h"

// ================= 初始化测试类型 =================
#define Encoder_CED_Test        1
#define Encoder_CIP_Test        2
#define Encoder_OIP_Test        3

// ================= 编码器类型定义 (对应原 Motor.EncoderType) =================
typedef enum {
    ENC_TYPE_NONE = 0,
    ENC_TYPE_MULTITURN_MAG,         // MULTITURNMagneticEncoder
    ENC_TYPE_MULTITURN_OPT,         // MULTITURNOpticalEncoder
    ENC_TYPE_MGT_MAG,               // MGTMagneticEncoder
    ENC_TYPE_TAMAGAWA,              // TAMAGAWAEncoder
    ENC_TYPE_COUNT
} EncoderType_t;

// ================= 报警码定义 (对应 ModBus_SlaveRx 中的错误响应) =================
#define WORK_ALARM_TIMEOUT      0x01
#define WORK_ALARM_CRC_ERROR    0x02
#define WORK_ALARM_ADDR_ERROR   0x03
#define WORK_ALARM_MODBUS_CRC   0x04
#define WORK_ALARM_NO_ENC_TYPE  0x05
#define WORK_ALARM_SN_ERROR     0x06
#define WORK_ALARM_Length_ERROR 0x07
#define WORK_ALARM_FUNC_ERROR   0x08
#define WORK_ALARM_SLAVE_ERROR  0x09
#define WORK_ALARM_F0_ERROR     0x0A

// ================= 测试项目定义 =================
typedef enum {
    TEST_ITEM_STOP = 0,             // 停止测试
    TEST_ITEM_READ_ANGLE,           // 读取角度
    TEST_ITEM_READ_MULTITURN,       // 读取多圈
    TEST_ITEM_READ_ALL,             // 读取全部数据
    TEST_ITEM_READ_MTP,             // 读取铭牌 (MTP)
    TEST_ITEM_RESET_MULTITURN,      // 多圈复位
    TEST_ITEM_RESET_SINGLETURN,     // 单圈复位
    TEST_ITEM_COUNT
} TestItem_t;

// ================= 编码器配置结构 =================
typedef struct {
    EncoderType_t Type;             // 编码器类型
    uint32_t BaudRate;              // 波特率 (2500000, 4000000, etc.)
    float CommCycle_us;             // 通信周期 (us), 默认 62.5 (16kHz), 支持小数
    TestItem_t CurrentTest;         // 当前测试项目
} EncoderConfig_t;

// ================= 编码器数据结构 =================
typedef struct {
    uint32_t SingleTurnPos;         // 单圈位置
    uint16_t MultiTurnPos;          // 多圈位置
    uint32_t RawData;               // 原始数据
    uint8_t  Status;                // 状态字节 (SF)
    uint8_t  Alarm;                 // 报警字节 (ALMC)
    uint8_t  CrcOK;                 // CRC 校验结果
    uint8_t  Timeout;               // 超时标志
    uint8_t  CommError;             // 通信错误标志
} EncoderData_t;

// ================= 寄存器地址分区定义 =================
// 0x0000 - 0x00FF: 默认配置区
#define REG_CONFIG_START        0x0000
#define REG_CONFIG_END          0x00FF

// 0x0200 - 0x02FF: MultiturnMagneticEncoder 测试
#define REG_MULTITURN_MAG_START 0x0200
#define REG_MULTITURN_MAG_END   0x02FF

// 0x0300 - 0x03FF: MultiturnOpticalEncoder 测试
#define REG_MULTITURN_OPT_START 0x0300
#define REG_MULTITURN_OPT_END   0x03FF

// 0x0400 - 0x04FF: MGTMagneticEncoder 测试
#define REG_MGT_MAG_START       0x0400
#define REG_MGT_MAG_END         0x04FF

// 0x0600 - 0x06FF: TamagawaEncoder 测试
#define REG_TAMAGAWA_START      0x0600
#define REG_TAMAGAWA_END        0x06FF

// ================= 配置区寄存器 - 03H 读取 (0x00xx) =================
#define REG_FW_VERSION          0x0000  // 软件版本
#define REG_HW_VERSION          0x0001  // 硬件版本
#define REG_ENCODER_ALARM       0x0002  // 编码器报警
#define REG_ENCODER_SPEED       0x0003  // 编码器转速
#define REG_COMM_CYCLE_READ     0x0004  // 通信周期 (读取)

// ================= 配置区寄存器 - 06H 写入 (0x00xx) =================
#define REG_ENCODER_POWER       0x0000  // 编码器断电/上电 (预留)
#define REG_ENCODER_POWER_ON    0x0001  // 编码器上电 (预留)
#define REG_STOP_COMM           0x0002  // 停止通讯
#define REG_SET_COMM_CYCLE      0x0003  // 设置通讯周期并启动
#define REG_SET_ENCODER_TYPE    0x0004  // 设置编码器类型
#define REG_SPEED_TEST          0x0005  // 准备检测速度波动
#define REG_WRITE_YEAR          0x0006  // 写入测试年
#define REG_WRITE_MONTH         0x0007  // 写入测试月
#define REG_WRITE_DAY           0x0008  // 写入测试日
#define REG_WRITE_HOUR          0x0009  // 写入测试时
#define REG_SET_HWREV           0x000A  // 设置硬件版本号
#define REG_SET_RESOLUTION      0x000B  // 设置分辨率

// ================= 外部接口声明 =================
void EncoderModbus_Init(void);

// 03H 读取处理
// 03H 读取处理
uint16_t EncoderModbus_ReadReg(uint16_t addr);

// 06H 写入处理 (返回 0=成功, 1=失败)
uint8_t EncoderModbus_WriteReg(uint16_t addr, uint16_t value);

// 同步各编码器的通讯错误 (DC/EC) 到 ModBus 结构体
void EncoderModbus_UpdateErrors(void);

// 检查地址是否属于编码器模块
uint8_t EncoderModbus_IsMyAddress(uint16_t addr);

// 全局数据访问
extern EncoderConfig_t g_EncoderConfig;
extern EncoderData_t g_EncoderData;

#endif /* __ENCODER_MODBUS_H */
