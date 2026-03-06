/****************************************************************************************
 * @file      blackbox.c
 * @brief     通讯黑匣子模块 — 四通道帧 + 内部事件全程记录，支持分段导出
 * @author    Gemini
 * @date      2026-03-03
 * @note      采用滚动覆盖环形缓冲区，上位机可在测试过程中多次分段读取
 ****************************************************************************************/
#include "blackbox.h"
#include "uart_config.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// ================= 配置 =================
#define BLACKBOX_SIZE   500   // 环形缓冲区条数

// ================= 内部变量 =================
static BlackBox_Record_t s_Buffer[BLACKBOX_SIZE];
static uint16_t          s_Head     = 0;    // 写入位置 (下一条写入的索引)
static uint16_t          s_Count    = 0;    // 当前缓冲区中有效记录数
static uint8_t           s_Enabled  = 0;
static uint32_t          s_StartTs  = 0;    // 黑匣子启动时的基准时间戳
static uint32_t          s_SeqNum   = 0;    // 全局递增序号 (跨分段不清零)
static uint8_t           s_Wrapped  = 0;    // 本段是否发生过滚动覆盖

/****************************************************************************************
* 函数名称：BlackBox_Enable
* 函数功能：清空缓冲区并开启黑匣子记录功能，全局序号归零
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-03
****************************************************************************************/
void BlackBox_Enable(void)
{
    s_Head     = 0;
    s_Count    = 0;
    s_SeqNum   = 0;
    s_Wrapped  = 0;
    s_Enabled  = 1;
    s_StartTs  = HAL_GetTick();
}

/****************************************************************************************
* 函数名称：BlackBox_Disable
* 函数功能：关闭黑匣子记录功能（不清空缓冲区，仍可导出）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-05
****************************************************************************************/
void BlackBox_Disable(void)
{
    s_Enabled = 0;
}

/****************************************************************************************
* 函数名称：BlackBox_Clear
* 函数功能：清空缓冲区（不改变使能状态，不清零全局序号）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-03
****************************************************************************************/
void BlackBox_Clear(void)
{
    s_Head    = 0;
    s_Count   = 0;
    s_Wrapped = 0;
}

/****************************************************************************************
* 函数名称：BlackBox_Log
* 函数功能：记录一条通讯帧（方向 + 时间戳 + 原始数据），缓冲区满则滚动覆盖最旧记录
* 输入参量：dir  方向标签 (BB_A/BB_a/BB_B/BB_b/BB_EVT)
*           data 数据指针
*           len  数据长度（超过40字节只取前40字节）
* 输出参量：无
* 编写日期：2026-3-03
****************************************************************************************/
void BlackBox_Log(char dir, const uint8_t *data, uint8_t len)
{
    if (!s_Enabled) return;

    BlackBox_Record_t *rec = &s_Buffer[s_Head];
    rec->dir    = dir;
    rec->ts_ms  = HAL_GetTick() - s_StartTs;
    rec->len    = (len > 40) ? 40 : len;
    memcpy(rec->data, data, rec->len);

    s_Head = (s_Head + 1) % BLACKBOX_SIZE;
    s_SeqNum++;

    if (s_Count < BLACKBOX_SIZE) {
        s_Count++;
    } else {
        s_Wrapped = 1;  // 发生了滚动覆盖
    }
}

/****************************************************************************************
* 函数名称：BlackBox_LogEvent
* 函数功能：记录一条内部事件日志（printf 风格字符串），缓冲区满则滚动覆盖
* 输入参量：fmt 格式化字符串
* 输出参量：无
* 编写日期：2026-3-03
****************************************************************************************/
void BlackBox_LogEvent(const char *fmt, ...)
{
    if (!s_Enabled) return;

    BlackBox_Record_t *rec = &s_Buffer[s_Head];
    rec->dir   = BB_EVT;
    rec->ts_ms = HAL_GetTick() - s_StartTs;

    va_list args;
    va_start(args, fmt);
    int n = vsnprintf((char *)rec->data, sizeof(rec->data), fmt, args);
    va_end(args);

    rec->len = (n > 0 && n < (int)sizeof(rec->data)) ? (uint8_t)n : (uint8_t)(sizeof(rec->data) - 1);

    s_Head = (s_Head + 1) % BLACKBOX_SIZE;
    s_SeqNum++;

    if (s_Count < BLACKBOX_SIZE) {
        s_Count++;
    } else {
        s_Wrapped = 1;
    }
}

/****************************************************************************************
* 函数名称：BlackBox_SendAll
* 函数功能：将缓冲区中全部记录以可读文本格式通过 USART1 发送给上位机
*           发送完毕后清空缓冲区，但保持录制使能不变（支持分段导出）
* 输入参量：无
* 输出参量：无
* 编写日期：2026-3-03
* 备注：上位机可在测试过程中多次调用此函数分段导出，录制持续进行
*       只有发送 BlackBox_Disable 或新的 Enable 命令才会改变录制状态
****************************************************************************************/
void BlackBox_SendAll(void)
{
    if (s_Count == 0) {
        Usart1_Print("EMPTY\r\n");
        return;
    }

    // 计算读取起点：如果未满则从 0 开始，否则从 s_Head 开始（最旧的记录）
    uint16_t read_start;
    if (s_Count < BLACKBOX_SIZE) {
        read_start = 0;
    } else {
        read_start = s_Head;  // s_Head 指向下一条要写入的位置，即最旧的记录
    }

    // 计算本段起始序号（全局序号 - 当前有效条数 + 1）
    uint32_t base_seq = s_SeqNum - s_Count + 1;

    if (s_Wrapped) {
        Usart1_Print("WRAPPED\r\n");
    }

    for (uint16_t i = 0; i < s_Count; i++) {
        uint16_t idx = (read_start + i) % BLACKBOX_SIZE;
        BlackBox_Record_t *rec = &s_Buffer[idx];
        uint32_t seq = base_seq + i;

        if (rec->dir == BB_EVT) {
            Usart1_Print("[%05lu][%c][+%08lums] %s\r\n",
                         seq, rec->dir, rec->ts_ms, (char *)rec->data);
        } else {
            char line[256];
            int  pos = 0;
            pos += snprintf(line + pos, sizeof(line) - pos,
                            "[%05lu][%c][+%08lums]",
                            seq, rec->dir, rec->ts_ms);

            for (uint8_t j = 0; j < rec->len && pos < (int)(sizeof(line) - 4); j++) {
                pos += snprintf(line + pos, sizeof(line) - pos, " %02X", rec->data[j]);
            }

            pos += snprintf(line + pos, sizeof(line) - pos, "\r\n");
            Usart1_Print("%s", line);
        }
    }

    Usart1_Print("END\r\n");

    // 清空缓冲区，但保持 s_Enabled 和 s_SeqNum 不变（继续录制）
    BlackBox_Clear();
}
