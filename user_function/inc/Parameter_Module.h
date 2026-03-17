/****************************************************************************************
* @file      Parameter_Module.h
* @brief     参数管理模块头文件 (定义参数表、缓冲区与属性)
* @author    Gemini
* @date      2026-02-09
****************************************************************************************/
#ifndef __PARAMETER_MODULE_H
#define __PARAMETER_MODULE_H

#include "main.h"

// ================= 缓冲区大小定义 =================
#define PA_SIZE 50                              // PA 参数组容量
#define DP_SIZE 50                              // dP 参数组容量

// ================= 枚举定义 =================

// 数据显示进制
typedef enum { 
    FMT_DEC = 0,                                // 十进制
    FMT_HEX,                                    // 十六进制 (H.)
    FMT_BIN                                     // 二进制 (b.)
} PM_Format_t;

// 数据符号属性
typedef enum { 
    SIGNED = 0,                                 // 有符号
    UNSIGNED                                    // 无符号
} PM_Sign_t;

// 数据位宽属性
typedef enum { 
    BIT_16 = 0,                                 // 16位数据
    BIT_32                                      // 32位数据 (启用分页)
} PM_Width_t; 

// 单个参数的属性配置
typedef struct {
    PM_Sign_t   Sign;                          // 符号属性
    PM_Format_t Format;                        // 显示进制
    PM_Width_t  Width;                         // 数据位宽
    int32_t      Min;                           // 最小值限制
    int32_t      Max;                           // 最大值限制
} PM_ParamConfig_t;

// ================= 外部接口声明 =================
void PM_Init(void);                            // 初始化参数模块 (加载默认值)
PM_ParamConfig_t PM_GetConfig(uint8_t group, uint16_t index); // 获取参数配置
void PM_SaveParams(void);                      // 保存参数到 Flash

// 访问外部变量
extern int32_t PA_Buffer[PA_SIZE];
extern int32_t DP_Buffer[DP_SIZE];

#endif /* __PARAMETER_MODULE_H */
