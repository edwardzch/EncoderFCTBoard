/****************************************************************************************
 * @file      encoder_speed.h
 * @brief     编码器转速计算模块 (通用, 基于位置差分)
 * @date      2026-02-10
 * @note      RPM = (ΔPos / 2^bits) × (60,000,000 / CommCycle_us)
 *            支持任意分辨率编码器, 中位值滤波
 ****************************************************************************************/
#ifndef __ENCODER_SPEED_H
#define __ENCODER_SPEED_H

#include "main.h"

// ================= 配置参数 =================
#define SPEED_FILTER_SIZE   16      // 中位值滤波缓冲区大小
#define SPEED_STARTUP_CNT   50      // 启动忽略前 N 次采样 (等待稳定)

// ================= 速度计算上下文 =================
typedef struct {
    // 配置
    uint8_t  ResolutionBits;        // 单圈分辨率 (如 17, 23)
    float    SamplePeriod_us;       // 采样周期 (us), 即 TIM1 中断周期
    
    // 内部状态
    uint32_t PrevPosition;          // 上一次位置
    int32_t  PosDiff;               // 位置差
    uint16_t StartupCnt;            // 启动计数器
    float    ScaleFactor;           // 预计算: 60,000,000 / (2^bits * period_us)
    
    // 滤波
    int16_t  Buffer[SPEED_FILTER_SIZE];  // 速度滤波缓冲区
    int16_t  SortBuf[SPEED_FILTER_SIZE]; // 排序用临时缓冲
    uint8_t  BufIndex;              // 环形写入索引
    uint8_t  BufFilled;             // 缓冲区是否已填满
    
    // 输出
    int16_t  RawSpeed;              // 瞬时转速 (RPM)
    int16_t  FilteredSpeed;         // 滤波后转速 (RPM)
} SpeedCalc_t;

// ================= 接口函数 =================

/**
 * @brief  初始化速度计算器
 * @param  sc: 计算上下文
 * @param  resolution_bits: 单圈分辨率位数 (如 17, 23)
 * @param  sample_period_us: TIM1中断周期 (us)
 */
void SpeedCalc_Init(SpeedCalc_t *sc, uint8_t resolution_bits, float sample_period_us);

/**
 * @brief  更新采样周期 (TIM1周期改变后调用)
 */
void SpeedCalc_SetPeriod(SpeedCalc_t *sc, float sample_period_us);

/**
 * @brief  更新分辨率位数 (不清除历史数据)
 * @param  sc: 计算上下文
 * @param  resolution_bits: 新的分辨率位数
 */
void SpeedCalc_UpdateResolution(SpeedCalc_t *sc, uint8_t resolution_bits);

/**
 * @brief  每个 TIM1 中断周期调用一次, 传入当前单圈位置
 * @param  sc: 计算上下文
 * @param  position: 当前单圈位置 (原始值)
 */
void SpeedCalc_Update(SpeedCalc_t *sc, uint32_t position);

// ================= 全局实例 =================
extern SpeedCalc_t g_SpeedCalc;

#endif /* __ENCODER_SPEED_H */
