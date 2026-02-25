/****************************************************************************************
 * @file      encoder_driver.c
 * @brief     编码器底层驱动实现 (RS485 DMA, 纯寄存器操作)
 * @author    Gemini
 * @date      2026-02-10
 * @note      USART3: TX=DMA1_Channel5, RX=DMA1_Channel4
 *            不使用 HAL 库 DMA 函数，全部直接操作寄存器
 ****************************************************************************************/
#include "encoder_driver.h"
#include "encoder_dispatcher.h"
#include "Encoder_MultiturnMag.h"
#include <string.h>

// ================= DMA 通道映射 =================
// CubeMX 配置: USART3_RX -> DMA1_Channel4, USART3_TX -> DMA1_Channel5
#define DMA_TX_CH       DMA1_Channel5
#define DMA_RX_CH       DMA1_Channel4

// DMA1 中断标志位 (Channel4=RX, Channel5=TX)
// Channel4: TCIF4=bit13, HTIF4=bit12, TEIF4=bit11, GIF4=bit10  (IFCR 清除)
// Channel5: TCIF5=bit17, HTIF5=bit16, TEIF5=bit15, GIF5=bit14
#define DMA_RX_TCIF     DMA_ISR_TCIF4
#define DMA_RX_GIF      DMA_ISR_GIF4
#define DMA_RX_CGIF     DMA_IFCR_CGIF4
#define DMA_TX_TCIF     DMA_ISR_TCIF5
#define DMA_TX_GIF      DMA_ISR_GIF5
#define DMA_TX_CGIF     DMA_IFCR_CGIF5

// ================= 内部状态 =================
static volatile EncDrvState_t s_State = ENC_DRV_IDLE;
static uint8_t s_TxBuffer[ENC_TX_BUFFER_SIZE];
static uint8_t s_RxBuffer[ENC_RX_BUFFER_SIZE];
static volatile uint16_t s_RxLen = 0;
static volatile uint16_t s_ExpectedRxLen = 0;

/****************************************************************************************
* 函数名称：EncDrv_Init
* 函数功能：初始化编码器驱动，配置波特率
* 输入参量：baudrate 波特率
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_Init(uint32_t baudrate)
{
    EncDrv_SetBaudRate(baudrate);
    RS485_RX_ENABLE();
    s_State = ENC_DRV_IDLE;
}

/****************************************************************************************
* 函数名称：EncDrv_SetBaudRate
* 函数功能：动态修改 USART3 波特率（直接寄存器操作）
* 输入参量：baudrate 新的波特率值
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_SetBaudRate(uint32_t baudrate)
{
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    
    USART3->CR1 &= ~USART_CR1_UE;
    USART3->BRR = (pclk + baudrate / 2) / baudrate;
    USART3->CR1 |= USART_CR1_UE;
}

/****************************************************************************************
* 函数名称：EncDrv_StartDMA_TX
* 函数功能：启动 DMA 发送（寄存器操作），配置 RS485 为发送模式
* 输入参量：data 发送数据指针，len 发送长度
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void EncDrv_StartDMA_TX(uint8_t *data, uint16_t len)
{
    // 1. 关闭 DMA TX 通道
    DMA_TX_CH->CCR &= ~DMA_CCR_EN;
    
    // 2. 清除 DMA TX 中断标志
    DMA1->IFCR = DMA_TX_CGIF;
    
    // 3. 配置 DMA TX
    DMA_TX_CH->CPAR = (uint32_t)&USART3->TDR;     // 外设地址
    DMA_TX_CH->CMAR = (uint32_t)data;               // 内存地址
    DMA_TX_CH->CNDTR = len;                          // 传输长度
    
    // 4. 清除 USART3 TC 标志
    USART3->ICR = USART_ICR_TCCF;
    
    // 5. 使能 USART3 DMA 发送请求
    USART3->CR3 |= USART_CR3_DMAT;
    
    // 6. 使能发送器
    USART3->CR1 |= USART_CR1_TE;
    
    // 7. 启动 DMA TX (只使能 TCIE, 禁用 HTIE/TEIE)
    DMA_TX_CH->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TEIE);
    DMA_TX_CH->CCR |= (DMA_CCR_EN | DMA_CCR_TCIE);
}

/****************************************************************************************
* 函数名称：EncDrv_StartDMA_RX
* 函数功能：启动 DMA 接收（寄存器操作），配置接收通道和长度
* 输入参量：data 接收缓冲区指针，len 期望接收长度
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void EncDrv_StartDMA_RX(uint8_t *data, uint16_t len)
{
    // 1. 关闭 DMA RX 通道
    DMA_RX_CH->CCR &= ~DMA_CCR_EN;
    
    // 2. 清除 DMA RX 中断标志
    DMA1->IFCR = DMA_RX_CGIF;
    
    // 3. 清除 USART3 ORE 等错误标志
    USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    
    // 4. 配置 DMA RX
    DMA_RX_CH->CPAR = (uint32_t)&USART3->RDR;     // 外设地址
    DMA_RX_CH->CMAR = (uint32_t)data;               // 内存地址
    DMA_RX_CH->CNDTR = len;                          // 传输长度
    
	  // 5. 使能 USART3 DMA 接收请求
    USART3->CR3 |= USART_CR3_DMAR;
    
    // 6. 使能接收器
    USART3->CR1 |= USART_CR1_RE;
    
    // 7. 启动 DMA RX (只使能 TCIE, 禁用 HTIE/TEIE)
    DMA_RX_CH->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TEIE);
    DMA_RX_CH->CCR |= (DMA_CCR_EN | DMA_CCR_TCIE);
}


/****************************************************************************************
* 函数名称：EncDrv_StopDMA
* 函数功能：停止 DMA 收发通道，关闭 USART DMA 请求
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
static void EncDrv_StopDMA(void)
{
    // 关闭 DMA TX/RX 通道
    DMA_TX_CH->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE);
    DMA_RX_CH->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE);
    
    // 关闭 USART3 DMA 请求
    USART3->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
    
    // 清除所有 DMA 中断标志
    DMA1->IFCR = DMA_TX_CGIF | DMA_RX_CGIF;
}

/****************************************************************************************
* 函数名称：EncDrv_SendDMA
* 函数功能：编码器 DMA 发送对外接口（非阻塞），拷贝数据并启动发送
* 输入参量：data 发送数据指针，len 发送长度，rx_len 期望接收长度
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_SendDMA(uint8_t *data, uint16_t len, uint16_t rx_len)
{
    if (s_State == ENC_DRV_TX_BUSY || s_State == ENC_DRV_RX_WAIT) {
        return;
    }
    
    if (len > ENC_TX_BUFFER_SIZE) len = ENC_TX_BUFFER_SIZE;
    memcpy(s_TxBuffer, data, len);
    
    s_ExpectedRxLen = rx_len;
    s_RxLen = 0;
    s_State = ENC_DRV_TX_BUSY;
    
    RS485_TX_ENABLE();
    EncDrv_StartDMA_TX(s_TxBuffer, len);
}

/****************************************************************************************
* 函数名称：EncDrv_TxCompleteCallback
* 函数功能：DMA 发送完成回调，关闭 DMA TX 并使能 USART TC 中断
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_TxCompleteCallback(void)
{
    // 1. 关闭 DMA TX
    DMA_TX_CH->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE);
    USART3->CR3 &= ~USART_CR3_DMAT;
    
    // 2. 清除 TC 标志, 使能 USART3 TC 中断
    //    等待最后一位从移位寄存器移出后再切换 RS485 方向
    USART3->ICR = USART_ICR_TCCF;
    USART3->CR1 |= USART_CR1_TCIE;
}

/****************************************************************************************
* 函数名称：EncDrv_USART_IRQHandler
* 函数功能：USART3 中断处理，处理 TC 标志并切换 RS485 收发方向
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_USART_IRQHandler(void)
{
    if (USART3->ISR & USART_ISR_TC) {
        // 1. 清除 TC 标志, 禁用 TC 中断
        USART3->ICR = USART_ICR_TCCF;
        USART3->CR1 &= ~USART_CR1_TCIE;
        
        // 2. 切换到接收模式
        RS485_RX_ENABLE();
        
        // 3. 启动 DMA 接收
        if (s_ExpectedRxLen > 0) {
            s_State = ENC_DRV_RX_WAIT;
            EncDrv_StartDMA_RX(s_RxBuffer, s_ExpectedRxLen);
        } else {
            s_State = ENC_DRV_COMPLETE;
        }
    }
}

/****************************************************************************************
* 函数名称：EncDrv_RxCompleteCallback
* 函数功能：DMA 接收完成回调，保存接收长度并调用调度分发
* 输入参量：len 实际接收到的数据长度
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_RxCompleteCallback(uint16_t len)
{
    // 关闭 DMA RX
    DMA_RX_CH->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE);
    USART3->CR3 &= ~USART_CR3_DMAR;
    
    s_RxLen = len;
    s_State = ENC_DRV_COMPLETE;
    
    // 分发到对应编码器
    Encoder_DispatchRx(s_RxBuffer, len);
}

/****************************************************************************************
* 函数名称：EncDrv_GetState
* 函数功能：查询编码器驱动当前状态
* 输入参量：无
* 输出参量：当前驱动状态枚举值
* 编写日期：2026-2-24
****************************************************************************************/
EncDrvState_t EncDrv_GetState(void)
{
    return s_State;
}

uint8_t* EncDrv_GetRxBuffer(void)
{
    return s_RxBuffer;
}

/****************************************************************************************
* 函数名称：EncDrv_GetRxLen
* 函数功能：获取最近一次接收的数据长度
* 输入参量：无
* 输出参量：接收数据长度
* 编写日期：2026-2-24
****************************************************************************************/
uint16_t EncDrv_GetRxLen(void)
{
    return s_RxLen;
}

/****************************************************************************************
* 函数名称：EncDrv_TimeoutCheck
* 函数功能：超时处理，由 TIM1 中断调用，检测 RX 等待超时
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_TimeoutCheck(void)
{
    if (s_State == ENC_DRV_RX_WAIT) {
        uint16_t remaining = DMA_RX_CH->CNDTR;
        uint16_t received = s_ExpectedRxLen - remaining;
        if (received >= s_ExpectedRxLen) {
            EncDrv_StopDMA();
            EncDrv_RxCompleteCallback(received);
        }
    }
}

/****************************************************************************************
* 函数名称：TIM1_SetPeriod_Direct
* 函数功能：直接寄存器设置 TIM1 周期，自动计算 PSC 和 ARR
* 输入参量：cycle_us 周期（微秒），支持小数，如 62.5us
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void TIM1_SetPeriod_Direct(float cycle_us)
{
    TIM1->CR1 &= ~TIM_CR1_CEN;
    
    uint32_t total_ticks = (uint32_t)(cycle_us * (SystemCoreClock / 1000000.0f));
    if (total_ticks == 0) total_ticks = 1;
    
    // 计算最小 PSC, 使 ARR <= 65536
    uint16_t psc = 0;
    if (total_ticks > 65536) {
        psc = (uint16_t)((total_ticks - 1) / 65536);  // 向上取整
    }
    
    uint32_t arr = (total_ticks / (psc + 1)) - 1;
    if (arr > 0xFFFF) arr = 0xFFFF;
    
    TIM1->PSC = psc;
    TIM1->ARR = (uint16_t)arr;
    TIM1->CNT = 0;
    TIM1->EGR = TIM_EGR_UG;  // 产生更新事件, 立即加载 PSC
    TIM1->SR &= ~TIM_SR_UIF;  // 清除更新中断标志 (避免误触发)
    TIM1->DIER |= TIM_DIER_UIE; // 使能更新中断
    TIM1->CR1 |= TIM_CR1_CEN;
}

/****************************************************************************************
* 函数名称：TIM1_Start_Direct
* 函数功能：启动 TIM1 定时器，使能更新中断和计数器
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void TIM1_Start_Direct(void)
{
    TIM1->CNT = 0;
    TIM1->SR &= ~TIM_SR_UIF; // 清除标志
    TIM1->DIER |= TIM_DIER_UIE; // 使能更新中断
    TIM1->CR1 |= TIM_CR1_CEN;
}

/****************************************************************************************
* 函数名称：TIM1_Stop_Direct
* 函数功能：停止 TIM1 定时器，禁用计数器和更新中断
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void TIM1_Stop_Direct(void)
{
    TIM1->DIER &= ~TIM_DIER_UIE; // 关闭更新中断
    TIM1->CR1 &= ~TIM_CR1_CEN;
}

/****************************************************************************************
* 函数名称：EncDrv_DMA_TX_IRQHandler
* 函数功能：DMA TX 中断处理（寄存器方式），由 DMA1_Channel5_IRQHandler 调用
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_DMA_TX_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF5) {
        DMA1->IFCR |= DMA_IFCR_CTCIF5;
        EncDrv_TxCompleteCallback();
    }
}

/****************************************************************************************
* 函数名称：EncDrv_DMA_RX_IRQHandler
* 函数功能：DMA RX 中断处理（寄存器方式），由 DMA1_Channel4_IRQHandler 调用
* 输入参量：无
* 输出参量：无
* 编写日期：2026-2-24
****************************************************************************************/
void EncDrv_DMA_RX_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF4) {
        DMA1->IFCR |= DMA_IFCR_CTCIF4;
        EncDrv_RxCompleteCallback(s_ExpectedRxLen);
    }
}
