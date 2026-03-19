/****************************************************************************************
 * @file      encoder_driver.h
 * @brief     编码器底层驱动 (RS485 DMA + TIM1 直接寄存器操作)
 * @author    Gemini
 * @date      2026-02-09
 * @note      TX/RX 均使用 DMA, 非阻塞, 解析在中断中完成
 ****************************************************************************************/
#ifndef __ENCODER_DRIVER_H
#define __ENCODER_DRIVER_H

#include "main.h"

// ================= RS485 控制引脚定义 =================
// 根据实际硬件修改 (PB3, shared DE/RE)
#define RS485_DE_PORT       USART3_EN_GPIO_Port
#define RS485_DE_PIN        USART3_EN_Pin
#define RS485_RE_PORT       USART3_EN_GPIO_Port
#define RS485_RE_PIN        USART3_EN_Pin

// 收发模式切换 (直接寄存器操作)
// TX: Pin High (Driver ON, Receiver OFF if /RE)
// RX: Pin Low  (Driver OFF, Receiver ON if /RE)
#define RS485_TX_ENABLE()   (RS485_DE_PORT->BSRR = RS485_DE_PIN)
#define RS485_RX_ENABLE()   (RS485_DE_PORT->BRR = RS485_DE_PIN)

// ================= 缓冲区定义 =================
#define ENC_TX_BUFFER_SIZE  10
#define ENC_RX_BUFFER_SIZE  135

// ================= 驱动状态 =================
typedef enum {
    ENC_DRV_IDLE = 0,
    ENC_DRV_TX_BUSY,
    ENC_DRV_RX_WAIT,
    ENC_DRV_COMPLETE,
    ENC_DRV_TIMEOUT,
    ENC_DRV_ERROR
} EncDrvState_t;

// ================= 时间单位 =================
typedef enum
{
    ENC_UNIT_US = 0, // 微秒
    ENC_UNIT_MS,     // 毫秒
    ENC_UNIT_S       // 秒
} EncTimeUnit_t;

// ================= MotorEncoder 结构体 =================
typedef struct {
    volatile uint8_t TestItem;      // 当前测试项 (各编码器模块共用, 值为各自枚举)
    uint8_t SetResolutionID;
    uint8_t HWRevData;
    uint16_t HWRevAddr;             // 硬件版本写入当前地址 (MGT: 0x030D-0x0310)
    uint8_t TestYear;
    uint8_t TestMoon;
    uint8_t TestDay;
    uint8_t TestHour;
    uint8_t TestDate;               // MGT 日期写入: 当前要写入的日期数据
    uint8_t TestDateCnt;            // MGT 日期写入: 当前写入地址偏移 (0x0304 + Cnt)
    uint16_t PitCnt;
    int16_t ActualSpeed;
	  uint32_t RxWaitDelayCnt;
    
    // ASCII 协议响应缓冲区
    char HWRevBuffer[16];           // 硬件版本字符串 (如 "H.M.1.1")
    char FWRevBuffer[16];           // 固件版本字符串 (如 "2.1.1.R")
} MotorEncoder_t;

typedef struct{
  int32_t       DifBuffer[2];
  int32_t       Difference;
  uint8_t       StartTime;
  int32_t       ActualSpeed;
  int32_t       FilterSpeed;
  int16_t       SpeedBuffer[16];
  uint32_t      Cnt;
  int32_t       AllData; 
  uint8_t       EncoderType;
  uint8_t       MTPType;
  uint32_t      FctPosition;
  int32_t       FctPos_MPos_Diff[2];
} Motor_t;

extern MotorEncoder_t g_MotorEncoder;
extern Motor_t g_Motor;
// ================= 接口声明 =================

/**
 * @brief  初始化编码器驱动 (USART3 + DMA)
 * @param  baudrate: 波特率
 */
void EncDrv_Init(uint32_t baudrate);

/**
 * @brief  动态修改波特率 (直接寄存器)
 * @param  baudrate: 新波特率
 */
void EncDrv_SetBaudRate(uint32_t baudrate);

/**
 * @brief  DMA 发送数据 (非阻塞)
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @param  rx_len: 期望接收长度 (0 表示不等待接收)
 */
void EncDrv_SendDMA(uint8_t *data, uint16_t len, uint16_t rx_len);

/**
 * @brief  USART3 中断处理 (处理 TC 切换 RS485 方向)
 */
void EncDrv_USART_IRQHandler(void);

/**
 * @brief  获取驱动状态
 */
EncDrvState_t EncDrv_GetState(void);

/**
 * @brief  获取接收数据指针
 */
uint8_t* EncDrv_GetRxBuffer(void);

/**
 * @brief  获取实际接收长度
 */
uint16_t EncDrv_GetRxLen(void);

/**
 * @brief  超时检查 (可由 TIM1 中断调用)
 */
void EncDrv_TimeoutCheck(void);

/**
 * @brief  设置等待指定时间的额外超时 Delay 计数值
 */
void EncDrv_SetRxWaitDelay(uint32_t time_val, EncTimeUnit_t unit);
/**
 * @brief  TX 完成回调 (由 DMA ISR 调用)
 */
void EncDrv_TxCompleteCallback(void);

/**
 * @brief  RX 完成回调 (由 DMA ISR 调用)
 * @param  len: 接收到的数据长度
 */
void EncDrv_RxCompleteCallback(uint16_t len);

/**
 * @brief  设置 TIM1 周期 (直接寄存器)
 * @param  cycle_us: 周期 (微秒), 支持小数 (如 62.5)
 */
void TIM1_SetPeriod_Direct(float cycle_us);

/**
 * @brief  启动/停止 TIM1
 */
void TIM1_Start_Direct(void);
void TIM1_Stop_Direct(void);

// ================= DMA 中断处理 (在 stm32g4xx_it.c 中调用) =================
void EncDrv_DMA_TX_IRQHandler(void);    // DMA1_Channel5 (USART3_TX)
void EncDrv_DMA_RX_IRQHandler(void);    // DMA1_Channel4 (USART3_RX)

uint8_t Enc_CRC8(uint8_t *data, uint16_t len);

#endif /* __ENCODER_DRIVER_H */
