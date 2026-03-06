/****************************************************************************************
 * @file      blackbox.h
 * @brief     通讯黑匣子模块 — 全程记录四通道帧数据与内部事件
 * @author    Gemini
 * @date      2026-03-03
 ****************************************************************************************/
#ifndef __BLACKBOX_H
#define __BLACKBOX_H

#include "main.h"
#include <stdint.h>

// ================= 方向标签定义 =================
#define BB_A    'A'   // FCT上位机 → 测试板 (Modbus 接收帧)
#define BB_a    'a'   // 测试板 → FCT上位机 (Modbus 响应帧)
#define BB_B    'B'   // 测试板 → 编码器 (DMA 发送帧)
#define BB_b    'b'   // 编码器 → 测试板 (DMA 接收帧)
#define BB_EVT  '*'   // 内部事件 / 计算结果 (文本字符串)

// ================= 记录结构体 =================
typedef struct {
    char     dir;        // 方向标签 (A / a / B / b / *)
    uint32_t ts_ms;      // 时间戳 (HAL_GetTick())
    uint8_t  len;        // 数据字节数 或 字符串长度
    uint8_t  data[40];   // 帧原始数据 或 事件字符串
} BlackBox_Record_t;

// ================= API =================
void BlackBox_Enable(void);
void BlackBox_Disable(void);
void BlackBox_Clear(void);
void BlackBox_Log(char dir, const uint8_t *data, uint8_t len);
void BlackBox_LogEvent(const char *fmt, ...);
void BlackBox_SendAll(void);

#endif /* __BLACKBOX_H */
