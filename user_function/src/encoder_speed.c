/****************************************************************************************
 * @file      encoder_speed.c
 * @brief     编码器转速计算模块实现
 * @date      2026-02-10
 * @note      通用速度计算器, 基于位置差分 + 中位值滤波
 *            RPM = (ΔPos / 2^bits) × (60,000,000 / CommCycle_us)
 ****************************************************************************************/
#include "encoder_speed.h"
#include <string.h>

// ================= 全局实例 =================
SpeedCalc_t g_SpeedCalc = {0};

/****************************************************************************************
* 函数名称：SpeedCalc_UpdateScaleFactor
* 函数功能：更新转速计算缩放因子，根据分辨率和采样周期计算
* 输入参量：sc 转速计算器指针
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void SpeedCalc_UpdateScaleFactor(SpeedCalc_t *sc)
{
    // RPM = diff * 60,000,000 / (2^bits * period_us)
    // ScaleFactor = 60,000,000 / (2^bits * period_us)
    if (sc->SamplePeriod_us > 0.0f && sc->ResolutionBits > 0) {
        float full_range = (float)(1UL << sc->ResolutionBits);
        sc->ScaleFactor = 60000000.0f / (full_range * sc->SamplePeriod_us);
    } else {
        sc->ScaleFactor = 0.0f;
    }
}

/****************************************************************************************
* 函数名称：SpeedCalc_MedianFilter
* 函数功能：中位值滤波，对转速采样缓冲区排序取中值
* 输入参量：sc 转速计算器指针
* 输出参量：滤波后的转速值
* 编写日期：2026-2-24
****************************************************************************************/
static int16_t SpeedCalc_MedianFilter(SpeedCalc_t *sc)
{
    uint8_t count = sc->BufFilled ? SPEED_FILTER_SIZE : sc->BufIndex;
    if (count == 0) return 0;
    
    // 拷贝到排序缓冲
    memcpy(sc->SortBuf, sc->Buffer, count * sizeof(int16_t));
    
    // 冒泡排序
    for (uint8_t i = 0; i < count - 1; i++) {
        for (uint8_t j = 0; j < count - 1 - i; j++) {
            if (sc->SortBuf[j] > sc->SortBuf[j + 1]) {
                int16_t tmp = sc->SortBuf[j];
                sc->SortBuf[j] = sc->SortBuf[j + 1];
                sc->SortBuf[j + 1] = tmp;
            }
        }
    }
    
    // 取中间两个值的平均
    uint8_t mid = count / 2;
    if (count % 2 == 0) {
        return (sc->SortBuf[mid - 1] + sc->SortBuf[mid]) / 2;
    } else {
        return sc->SortBuf[mid];
    }
}

/****************************************************************************************
* 函数名称：SpeedCalc_Init
* 函数功能：初始化转速计算器，设置分辨率和采样周期
* 输入参量：sc 转速计算器指针，resolution_bits 分辨率位数，sample_period_us 采样周期
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SpeedCalc_Init(SpeedCalc_t *sc, uint8_t resolution_bits, float sample_period_us)
{
    memset(sc, 0, sizeof(SpeedCalc_t));
    sc->ResolutionBits = resolution_bits;
    sc->SamplePeriod_us = sample_period_us;
    SpeedCalc_UpdateScaleFactor(sc);
}

/****************************************************************************************
* 函数名称：SpeedCalc_SetPeriod
* 函数功能：设置采样周期并更新缩放因子
* 输入参量：sc 转速计算器指针，sample_period_us 采样周期（微秒）
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SpeedCalc_SetPeriod(SpeedCalc_t *sc, float sample_period_us)
{
    sc->SamplePeriod_us = sample_period_us;
    SpeedCalc_UpdateScaleFactor(sc);
}

/****************************************************************************************
* 函数名称：SpeedCalc_UpdateResolution
* 函数功能：更新分辨率并重新计算缩放因子
* 输入参量：sc 转速计算器指针，resolution_bits 分辨率位数
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SpeedCalc_UpdateResolution(SpeedCalc_t *sc, uint8_t resolution_bits)
{
    sc->ResolutionBits = resolution_bits;
    SpeedCalc_UpdateScaleFactor(sc);
}

/****************************************************************************************
* 函数名称：SpeedCalc_Update
* 函数功能：更新转速计算，基于位置差分和中位值滤波
* 输入参量：sc 转速计算器指针，position 当前位置值
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void SpeedCalc_Update(SpeedCalc_t *sc, uint32_t position)
{
    // 计算位置差 (处理溢出/翻转)
    int32_t diff = (int32_t)(position - sc->PrevPosition);
    sc->PrevPosition = position;
    
    // 处理翻转: 如果差值超过半圈, 认为是反向翻转
    int32_t half_range = (int32_t)(1UL << (sc->ResolutionBits - 1));
    if (diff > half_range) {
        diff -= (int32_t)(1UL << sc->ResolutionBits);
    } else if (diff < -half_range) {
        diff += (int32_t)(1UL << sc->ResolutionBits);
    }
    
    sc->PosDiff = diff;
    
    // 启动阶段忽略 (等编码器数据稳定)
    if (sc->StartupCnt < SPEED_STARTUP_CNT) {
        sc->StartupCnt++;
        return;
    }
    
    // 计算瞬时速度 (RPM)
    sc->RawSpeed = (int16_t)(diff * sc->ScaleFactor);
    
    // 写入环形滤波缓冲区
    sc->Buffer[sc->BufIndex] = sc->RawSpeed;
    sc->BufIndex++;
    if (sc->BufIndex >= SPEED_FILTER_SIZE) {
        sc->BufIndex = 0;
        sc->BufFilled = 1;
    }
    
    // 中位值滤波
    sc->FilteredSpeed = SpeedCalc_MedianFilter(sc);
}
