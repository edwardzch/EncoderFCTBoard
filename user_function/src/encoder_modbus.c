/****************************************************************************************
 * @file      encoder_modbus.c
 * @brief     编码器测试 Modbus 统一路由模块
 * @author    Gemini
 * @date      2026-02-09
 * @note      路由编码器地址到各个独立模块:
 *            0x0200-0x02FF -> MultiturnMag
 *            0x0300-0x03FF -> MultiturnOpt
 *            0x0400-0x04FF -> MGTMag
 *            0x0500-0x05FF -> NXPMag
 *            0x0600-0x06FF -> Tamagawa
 *            0x0700-0x07FF -> SensAR
 ****************************************************************************************/
#include "encoder_modbus.h"
#include "modbus_function.h"
#include "encoder_driver.h"
#include "encoder_speed.h"
#include "Encoder_MultiturnMag.h"
#include "Encoder_MultiturnOpt.h"
#include "Encoder_MGTMag.h"
#include "Encoder_Tamagawa.h"
#include "blackbox.h"

// 引入 modbus_function.h 中的 ModBus 变量用于同步错误标志
extern volatile strModBus ModBus;

// ================= 全局变量 =================
EncoderConfig_t g_EncoderConfig = {
    .Type = ENC_TYPE_NONE,
    .BaudRate = 2500000,
    .CommCycle_us = 62.5f,
    .CurrentTest = TEST_ITEM_STOP
};

EncoderData_t g_EncoderData = {0};


// ================= 前向声明 =================
static void EncoderModbus_ApplyConfig(void);

/****************************************************************************************
* 函数名称：Enc_CRC8
* 函数功能：计算 XOR 校验值（与编码器端算法一致）
* 输入参量：data 数据指针，len 数据长度
* 输出参量：校验值
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t Enc_CRC8(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
    }
    return crc;
}

/****************************************************************************************
* 函数名称：EncoderModbus_Init
* 函数功能：初始化编码器 Modbus 模块，调用各编码器初始化和转速计算初始化
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncoderModbus_Init(void)
{
    g_EncoderConfig.Type = ENC_TYPE_NONE;
    g_EncoderConfig.BaudRate = 2500000;
    g_EncoderConfig.CommCycle_us = 62.5f;
    g_EncoderConfig.CurrentTest = TEST_ITEM_STOP;
    
    g_EncoderData.SingleTurnPos = 0;
    g_EncoderData.MultiTurnPos = 0;
    g_EncoderData.RawData = 0;
    g_EncoderData.Status = 0;
    g_EncoderData.Alarm = 0;
    g_EncoderData.CrcOK = 0;
    g_EncoderData.Timeout = 0;
    g_EncoderData.CommError = 0;
    
    // 初始化各编码器模块
    MulMag_Init();
    MulOpt_Init();
    MGT_Init();
    Tmgw_Init();
}

/****************************************************************************************
* 函数名称：EncoderModbus_IsMyAddress
* 函数功能：判断 Modbus 地址是否属于编码器模块
* 输入参量：addr Modbus 寄存器地址
* 输出参量：1=属于，0=不属于
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t EncoderModbus_IsMyAddress(uint16_t addr)
{
    // 配置区 0x0000-0x00FF
    if (addr <= REG_CONFIG_END) return 1;
    
    if (addr >= REG_MULTITURN_MAG_START && addr <= REG_TAMAGAWA_END) return 1;
    
    return 0;
}

/****************************************************************************************
* 函数名称：BasicReadings
* 函数功能：基础配置区读取 (0x0000-0x00FF)
* 输入参量：addr Modbus 寄存器地址
* 输出参量：对应地址的数据值
* 编写日期：2026-2-24
****************************************************************************************/
static uint16_t BasicReadings(uint16_t addr)
{
    switch (addr) {
        case REG_FW_VERSION:        return (uint16_t)(FirmwareVersion * 10);  // 1.0 -> 10
        case REG_HW_VERSION:        return (uint16_t)(HardwareVersion * 10);
        case REG_ENCODER_ALARM:     return MulMag_GetWorkAlarm();
        case REG_ENCODER_SPEED:     return g_MotorEncoder.ActualSpeed;
        case REG_COMM_CYCLE_READ:   return g_EncoderConfig.CommCycle_us;
        default: return 0xFFFF;
    }
}

/****************************************************************************************
* 函数名称：BasicSettings
* 函数功能：基础配置区写入 (0x0000-0x00FF)，处理配置参数设置
* 输入参量：addr Modbus 寄存器地址，value 写入值
* 输出参量：0=成功，非0=错误
* 编写日期：2026-2-24
****************************************************************************************/
static uint8_t BasicSettings(uint16_t addr, uint16_t value)
{
    switch (addr) {
        case REG_ENCODER_POWER:     // 0x0000: 编码器断电 (预留, 当前版本无控制引脚)
            // TODO: 加入 EncoderPowerOff()
            if (value == 1 || value == 2) {
                TIM1_Stop_Direct();
            }
            return 0;
            
        case REG_ENCODER_POWER_ON:  // 0x0001: 编码器上电 (预留)
            // TODO: 加入 EncoderPowerOn()
            if (value == 1 || value == 3) {
                TIM1_Start_Direct();
            }
            return 0;
            
        case REG_STOP_COMM:         // 0x0002: 停止通讯
            if (value == 1) {
                TIM1_Stop_Direct();
            }
            return 0;
            
        case REG_SET_COMM_CYCLE:    // 0x0003: 设置通讯周期并启动 (单位: us, 整数)
            g_EncoderConfig.CommCycle_us = (float)value;
            EncoderModbus_ApplyConfig();
            SpeedCalc_SetPeriod(&g_SpeedCalc, g_EncoderConfig.CommCycle_us);
            return 0;
            
        case REG_SET_ENCODER_TYPE:  // 0x0004: 设置编码器类型
            if (value > 0 && value < ENC_TYPE_COUNT) {
                g_EncoderConfig.Type = (EncoderType_t)value;
                // 根据类型设置波特率、默认周期和分辨率
                uint8_t res_bits = 17;  // 默认 17 位
                switch (value) {
                    case ENC_TYPE_MULTITURN_MAG:  // 1 - 17位
                        g_EncoderConfig.BaudRate = 2500000;
                        g_EncoderConfig.CommCycle_us = 1000;
                        res_bits = 17;
                        break;
                    case ENC_TYPE_MULTITURN_OPT:  // 2 - 23位
                        g_EncoderConfig.BaudRate = 2500000;
                        g_EncoderConfig.CommCycle_us = 500;
                        res_bits = 23;
                        break;
                    case ENC_TYPE_MGT_MAG:        // 3 - 17位
                        g_EncoderConfig.BaudRate = 2500000;
                        g_EncoderConfig.CommCycle_us = 125;
                        res_bits = 17;
                        break;
                    case ENC_TYPE_TAMAGAWA:       // 4 - 17位
                        g_EncoderConfig.BaudRate = 2500000;
                        g_EncoderConfig.CommCycle_us = 125;
                        res_bits = 17;
                        break;
										case ENC_TYPE_INTEGRAL_OPT:   // 8 - 23位
                        g_EncoderConfig.BaudRate = 2500000;
                        g_EncoderConfig.CommCycle_us = 125;
                        res_bits = 23;											
												break;
                    default:
												Communication_Address_Error();
                        break;
                }
                EncoderModbus_ApplyConfig();
                // 初始化速度计算器
                if (res_bits > 0) {
                    SpeedCalc_Init(&g_SpeedCalc, res_bits, g_EncoderConfig.CommCycle_us);
                }
            }
            return 0;
        
        case 0x0020:                // 0x0020: 黑匣子控制
            if (value == 0x5AA5) {   // 0x5AA5: 开启 (清空+归零+开始录制)
                BlackBox_Enable();
            } else if (value == 0xA55A) { // 0xA55A: 关闭 (停止录制, 缓冲区保留可读)
                BlackBox_Disable();
            }
            return 0;
            
        case REG_WRITE_YEAR:        // 0x0006: 写入测试年
            g_MotorEncoder.TestYear = (uint8_t)value;
            if (g_EncoderConfig.Type == ENC_TYPE_MGT_MAG) {
                g_MotorEncoder.TestDate = g_MotorEncoder.TestYear;
                g_MotorEncoder.TestDateCnt++;
                g_MotorEncoder.TestItem = MGT_Test_WriteDate;
            }
            return 0;
            
        case REG_WRITE_MONTH:       // 0x0007: 写入测试月
            g_MotorEncoder.TestMoon = (uint8_t)value;
            if (g_EncoderConfig.Type == ENC_TYPE_MGT_MAG) {
                g_MotorEncoder.TestDate = g_MotorEncoder.TestMoon;
                g_MotorEncoder.TestDateCnt++;
                g_MotorEncoder.TestItem = MGT_Test_WriteDate;
            }
            return 0;
            
        case REG_WRITE_DAY:         // 0x0008: 写入测试日
            g_MotorEncoder.TestDay = (uint8_t)value;
            if (g_EncoderConfig.Type == ENC_TYPE_MGT_MAG) {
                g_MotorEncoder.TestDate = g_MotorEncoder.TestDay;
                g_MotorEncoder.TestDateCnt++;
                g_MotorEncoder.TestItem = MGT_Test_WriteDate;
            }
            return 0;
            
        case REG_WRITE_HOUR:        // 0x0009: 写入测试时 (写入后触发日期写入测试)
            g_MotorEncoder.TestHour = (uint8_t)value;
            switch (g_EncoderConfig.Type) {
                case ENC_TYPE_MULTITURN_MAG:						
                    g_MotorEncoder.TestItem = MulMag_Test_WriteDate;
                    break;
                case ENC_TYPE_MULTITURN_OPT:
								case ENC_TYPE_INTEGRAL_OPT:
                    g_MotorEncoder.TestItem = MulOpt_Test_WriteDate;
                    break;
                case ENC_TYPE_MGT_MAG:
										g_MotorEncoder.TestDate = g_MotorEncoder.TestHour;
										g_MotorEncoder.TestDateCnt++;												
                    g_MotorEncoder.TestItem = MGT_Test_WriteDate;
                    break;
                default:
                    break;
            }
            return 0;
            
        case REG_SET_HWREV:         // 0x000A: 设置硬件版本号
            g_MotorEncoder.HWRevData = (uint8_t)value;
            g_MotorEncoder.TestItem = MulMag_Test_WriteHWRev;
            return 0;
            
        case REG_SET_RESOLUTION:    // 0x000B: 设置分辨率
            g_MotorEncoder.SetResolutionID = (uint8_t)value;
            g_MotorEncoder.TestItem = MulMag_Test_SetResolution;
            return 0;
            
        default:
            return 1;
    }
}

/****************************************************************************************
* 函数名称：EncoderModbus_ReadReg
* 函数功能：Modbus 03H 读取处理，路由到对应编码器模块
* 输入参量：addr Modbus 寄存器地址
* 输出参量：对应地址的数据值
* 编写日期：2026-2-24
****************************************************************************************/
uint16_t EncoderModbus_ReadReg(uint16_t addr)
{
    // 基础配置区 (0x0000-0x00FF)
    if (addr <= REG_CONFIG_END) {
        return BasicReadings(addr);
    }
    
    // 路由到各编码器模块
    if (addr >= REG_MULTITURN_MAG_START && addr <= REG_MULTITURN_MAG_END) {
        return MulMag_Modbus_Read(addr);
    }
    if (addr >= REG_MULTITURN_OPT_START && addr <= REG_MULTITURN_OPT_END) {
        return MulOpt_Modbus_Read(addr);
    }
    if (addr >= REG_MGT_MAG_START && addr <= REG_MGT_MAG_END) {
        return MGT_Modbus_Read(addr);
    }
    if (addr >= REG_TAMAGAWA_START && addr <= REG_TAMAGAWA_END) {
        return Tmgw_Modbus_Read(addr);
    }
    
    return 0xFFFF; // 未知地址
}

/****************************************************************************************
* 函数名称：EncoderModbus_WriteReg
* 函数功能：Modbus 06H 写入处理，路由到对应编码器模块
* 输入参量：addr Modbus 寄存器地址，value 写入值
* 输出参量：0=成功，1=失败
* 编写日期：2026-2-24
****************************************************************************************/
uint8_t EncoderModbus_WriteReg(uint16_t addr, uint16_t value)
{
    // 基础配置区 (0x0000-0x00FF)
    if (addr <= REG_CONFIG_END) {
        return BasicSettings(addr, value);
    }
    
    // 路由到各编码器模块
    if (addr >= REG_MULTITURN_MAG_START && addr <= REG_MULTITURN_MAG_END) {
        return MulMag_Modbus_Write(addr, value);
    }
    if (addr >= REG_MULTITURN_OPT_START && addr <= REG_MULTITURN_OPT_END) {
        return MulOpt_Modbus_Write(addr, value);
    }
    if (addr >= REG_MGT_MAG_START && addr <= REG_MGT_MAG_END) {
        return MGT_Modbus_Write(addr, value);
    }
    if (addr >= REG_TAMAGAWA_START && addr <= REG_TAMAGAWA_END) {
        return Tmgw_Modbus_Write(addr, value);
    }
    
    return 1;
}

/****************************************************************************************
* 函数名称：EncoderModbus_UpdateErrors
* 函数功能：将当前选中编码器模块的通讯和校验错误同步到 ModBus 全局结构体
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-28
****************************************************************************************/
void EncoderModbus_UpdateErrors(void)
{
    switch (g_EncoderConfig.Type) {
        case ENC_TYPE_MULTITURN_MAG:
            ModBus.Error.bit.DC = g_MulMag.Error.bit.DC;
            ModBus.Error.bit.EC = g_MulMag.Error.bit.EC;
            break;
            
        case ENC_TYPE_MULTITURN_OPT:
				case ENC_TYPE_INTEGRAL_OPT:
            ModBus.Error.bit.DC = g_MulOpt.Error.bit.DC;
            ModBus.Error.bit.EC = g_MulOpt.Error.bit.EC;
            break;
            
        case ENC_TYPE_MGT_MAG:
            ModBus.Error.bit.DC = g_MGT.Error.bit.DC;
            ModBus.Error.bit.EC = g_MGT.Error.bit.EC;
            break;
            
        case ENC_TYPE_TAMAGAWA:
            ModBus.Error.bit.DC = g_Tmgw.Error.bit.DC;
            ModBus.Error.bit.EC = g_Tmgw.Error.bit.EC;
            break;
            
        default:
            break;
    }
}

/****************************************************************************************
* 函数名称：EncoderModbus_ApplyConfig
* 函数功能：应用编码器配置，初始化驱动波特率和定时器周期
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void EncoderModbus_ApplyConfig(void)
{
    EncDrv_SetBaudRate(g_EncoderConfig.BaudRate);
    TIM1_SetPeriod_Direct(g_EncoderConfig.CommCycle_us);
}

