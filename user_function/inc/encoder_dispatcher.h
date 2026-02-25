/****************************************************************************************
 * @file      encoder_dispatcher.h
 * @brief     编码器统一调度头文件
 ****************************************************************************************/
#ifndef __ENCODER_DISPATCHER_H
#define __ENCODER_DISPATCHER_H

#include "main.h"

// ================= 调度函数 =================
void Encoder_DispatchTest(void);
void Encoder_DispatchRx(uint8_t *data, uint16_t len);

#endif /* __ENCODER_DISPATCHER_H */
