/****************************************************************************************
 * @file      encoder_dispatcher.c
 * @brief     编码器统一调度 (测试函数和RX数据分发)
 * @author    Gemini
 * @date      2026-02-09
 * @note      根据编码器类型调度到对应模块
 ****************************************************************************************/
#include "encoder_dispatcher.h"
#include "encoder_modbus.h"
#include "encoder_speed.h"
#include "Encoder_MultiturnMag.h"
#include "Encoder_MultiturnOpt.h"
#include "Encoder_MGTMag.h"
#include "Encoder_Tamagawa.h"
#include "encoder_driver.h"
#include "tim.h"
#include <string.h>

/****************************************************************************************
* 函数名称：Encoder_DispatchTest
* 函数功能：编码器测试调度，由 TIM1 中断调用，根据编码器类型分发测试
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Encoder_DispatchTest(void)
{
    switch (g_EncoderConfig.Type) {
        case ENC_TYPE_MULTITURN_MAG:
            MulMag_Test(g_MotorEncoder.TestItem);
            break;
            
        case ENC_TYPE_MULTITURN_OPT:
            MulOpt_Test(g_MotorEncoder.TestItem);
            break;
            
        case ENC_TYPE_MGT_MAG:
            MGT_Test(g_MotorEncoder.TestItem);
            break;
            
        case ENC_TYPE_TAMAGAWA:
            Tmgw_Test(g_MotorEncoder.TestItem);
            break;
            
        default:
            break;
    }
    g_EncoderConfig.CurrentTest = (TestItem_t)g_MotorEncoder.TestItem;
}

/****************************************************************************************
* 函数名称：Encoder_DispatchRx
* 函数功能：RX 数据调度，DMA 接收完成后拷贝数据到对应编码器并调用 RxComplete
* 输入参量：data 接收数据指针，len 接收数据长度
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void Encoder_DispatchRx(uint8_t *data, uint16_t len)
{
    switch (g_EncoderConfig.Type) {
        case ENC_TYPE_MULTITURN_MAG:
            memcpy((uint8_t *)g_MulMag.RxData, data, len);
            g_MulMag.RxDataCnt = len;
            MulMag_RxComplete();
            break;
            
        case ENC_TYPE_MULTITURN_OPT:
            memcpy(g_MulOpt.RxData, data, len);
            g_MulOpt.RxDataCnt = len;
            MulOpt_RxComplete();
            break;
            
        case ENC_TYPE_MGT_MAG:
            memcpy(g_MGT.RxData, data, len);
            g_MGT.RxDataCnt = len;
            MGT_RxComplete();
            break;
            
        case ENC_TYPE_TAMAGAWA:
            memcpy(g_Tmgw.RxData, data, len);
            g_Tmgw.RxDataCnt = len;
            Tmgw_RxComplete();
            break;
            
        default:
            break;
    }
}

// ================= Modbus 操作分发 =================

/****************************************************************************************
* 函数名称：Encoder_GetSingleTurnPos
* 函数功能：获取当前编码器单圈位置
* 输入参量：无
* 输出参量：单圈位置值
* 编写日期：2026-2-24
****************************************************************************************/
static uint32_t Encoder_GetSingleTurnPos(void)
{
    switch (g_EncoderConfig.Type) {
        case ENC_TYPE_MULTITURN_MAG: return g_MulMag.SingleTurnPos;
        case ENC_TYPE_MULTITURN_OPT: return g_MulOpt.SingleTurnPosition;
        case ENC_TYPE_MGT_MAG:       return g_MGT.SingleTurnPosition;
        case ENC_TYPE_TAMAGAWA:      return g_Tmgw.SingleTurnPosition;
        default: return 0;
    }
}

/****************************************************************************************
* 函数名称：Encoder_GetResolutionBits
* 函数功能：获取当前编码器分辨率位数
* 输入参量：无
* 输出参量：分辨率位数（如 17, 23）
* 编写日期：2026-2-24
****************************************************************************************/
static uint8_t Encoder_GetResolutionBits(void)
{
    switch (g_EncoderConfig.Type) {
        case ENC_TYPE_MULTITURN_MAG: return g_MulMag.ResolutionID;
        case ENC_TYPE_MULTITURN_OPT: return g_MulOpt.ResolutionID;
        case ENC_TYPE_MGT_MAG:       return g_MGT.ResolutionID;
        case ENC_TYPE_TAMAGAWA:      return g_Tmgw.ResolutionID;
        default: return 0;
    }
}

/****************************************************************************************
* 函数名称：HAL_TIM_PeriodElapsedCallback
* 函数功能：TIM1 周期回调（HAL），调度编码器测试和转速计算
* 输入参量：htim 定时器句柄
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        // 先检查上周期的接收是否超时未完成
        EncDrv_TimeoutCheck();
        
        Encoder_DispatchTest();
        
        // 速度计算 (SensAR 无单圈位置, 跳过)
        if (g_EncoderConfig.Type != ENC_TYPE_NONE) {
            // 自动检测编码器报告的分辨率, 动态更新 SpeedCalc
            uint8_t res_bits = Encoder_GetResolutionBits();
            if (res_bits > 0 && res_bits != g_SpeedCalc.ResolutionBits) {
                SpeedCalc_UpdateResolution(&g_SpeedCalc, res_bits);
            }
            SpeedCalc_Update(&g_SpeedCalc, Encoder_GetSingleTurnPos());
            g_MotorEncoder.ActualSpeed = g_SpeedCalc.FilteredSpeed;
        }
    }
}
