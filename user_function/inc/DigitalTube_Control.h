/****************************************************************************************
* @file      DigitalTube_Control.h
* @brief     数码管驱动头文件 (包含属性配置、状态机定义与硬件映射)
* @author    Gemini
* @date      2026-02-06
****************************************************************************************/
#ifndef __DIGITALTUBE_CONTROL_H
#define __DIGITALTUBE_CONTROL_H

#include "main.h"

// ================= 缓冲区大小定义 =================
#define PA_SIZE 50                              // PA 参数组容量
#define DP_SIZE 50                              // dP 参数组容量

// ================= 硬件引脚映射 =================
// 锁存引脚 (RCLK/NSS)
#define DTC_RCLK_PORT    SPI2_NSS_GPIO_Port
#define DTC_RCLK_PIN     SPI2_NSS_Pin
#define DTC_RCLK_L()     (DTC_RCLK_PORT->BRR  = (uint32_t)DTC_RCLK_PIN)
#define DTC_RCLK_H()     (DTC_RCLK_PORT->BSRR = (uint32_t)DTC_RCLK_PIN)

// 按键引脚
#define DTC_KEY_PORT     GPIOB
#define PIN_MODE         KEY1_Pin               // 功能键: 切换组/退出
#define PIN_UP           KEY2_Pin               // 加键
#define PIN_DOWN         KEY3_Pin               // 减键
#define PIN_SHIFT        KEY4_Pin               // 移位键: 短按翻页/移位, 长按进入/保存

// ================= 交互时间参数 =================
#define KEY_DEBOUNCE_MS  20                     // 消抖时间 (ms)
#define KEY_LONG_MS      1000                   // 长按判定阈值 (ms)
#define ACCEL_START_MS   250                    // 连发初始间隔 (ms)
#define ACCEL_MIN_MS     30                     // 连发最小间隔 (最快速度)
#define ACCEL_STEP       15                     // 连发加速步进 (ms)

// ================= 枚举定义 =================

// 数据显示进制
typedef enum { 
    FMT_DEC = 0,                                // 十进制
    FMT_HEX,                                    // 十六进制 (H.)
    FMT_BIN                                     // 二进制 (b.)
} DTC_Format_t;

// 数据符号属性
typedef enum { 
    SIGNED = 0,                                 // 有符号
    UNSIGNED                                    // 无符号
} DTC_Sign_t;

// 数据位宽属性
typedef enum { 
    BIT_16 = 0,                                 // 16位数据
    BIT_32                                      // 32位数据 (启用分页)
} DTC_Width_t; 

// 32位数据分页状态
typedef enum { 
    PAGE_LOW = 0,                               // 低位页 (_ 1234)
    PAGE_MID,                                   // 中位页 (- 5678)
    PAGE_HIGH                                   // 高位页 (FE 90)
} DTC_Page_t;

// 主显示模式
typedef enum {
    DTC_MODE_ANIMATION = 0,                     // 开机动画模式
    DTC_MODE_SELECT,                            // 参数选择模式 (PA 001)
    DTC_MODE_EDIT,                              // 参数编辑模式 (数值)
    DTC_MODE_ERROR,                             // 故障报错模式
    DTC_MODE_MESSAGE                            // 消息提示模式 (donE)
} DTC_DispMode_t;

// 单个参数的属性配置
typedef struct {
    DTC_Sign_t   Sign;                          // 符号属性
    DTC_Format_t Format;                        // 显示进制
    DTC_Width_t  Width;                         // 数据位宽
    int32_t      Min;                           // 最小值限制
    int32_t      Max;                           // 最大值限制
} DTC_ParamConfig_t;

// 开机动画子状态
typedef enum {
    ANIM_TYPEWRITER = 0,                        // 打字机阶段 (E -> Et...)
    ANIM_WAIT_KEY,                              // 等待按键阶段 (Etest 常亮)
    ANIM_DONE                                   // 动画完成
} DTC_AnimState_t;

// 全局运行状态
typedef struct {
    uint8_t  RawData[5];                        // 显存 (存储字库索引或特殊段码)
    uint8_t  GroupIdx;                          // 当前参数组 (0:PA, 1:dP)
    uint16_t ParamNum;                          // 当前参数编号 (0 ~ SIZE-1)
    
    DTC_DispMode_t Mode;                        // 当前 UI 模式
    DTC_Page_t     Page;                        // 当前分页状态 (仅32位有效)
    int32_t        EditVal;                     // 正在编辑的临时数值
    uint8_t        EditBit;                     // 当前光标位置 (0-3)

    uint16_t BlinkCnt;                          // 闪烁计时器
    uint16_t KeyTimer;                          // 按键按下计时器
    uint16_t RepeatTimer;                       // 连发间隔计时器
    uint16_t CurrentSpeed;                      // 当前连发速度
    uint8_t  LastKey;                           // 上一次按下的键值
    uint8_t  LongPressDone;                     // 长按已处理标志
    uint16_t ErrCode;                           // 错误代码

    // 动画专用变量
    DTC_AnimState_t AnimState;                  // 动画子状态
    uint16_t        AnimTimer;                  // 动画计时器
    uint16_t        MsgTimer;                   // 消息计时器
    uint8_t         AnimStep;                   // 动画步骤索引
} DTC_State_t;

// ================= 外部接口声明 =================
void DTC_Init(void);                            // 初始化函数
void DTC_ScanHandler(void);                     // 扫描中断处理函数 (1ms)
void DTC_SetError(uint16_t code);               // 报错显示函数

// 用户需实现的回调函数 (模拟 Flash 保存)
void DTC_SaveParams_Callback(void); 

extern int32_t PA_Buffer[PA_SIZE];              // PA 参数数组
extern int32_t DP_Buffer[DP_SIZE];              // dP 参数数组

#endif
