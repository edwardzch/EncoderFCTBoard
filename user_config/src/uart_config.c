/****************************************************************************************
  * @file      uart_config.c
  * @brief     UART 配置与 DMA 发送实现
  ****************************************************************************************/
#include "uart_config.h"
#include "usart.h"
#include "modbus_function.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

volatile strUsart1  Usart1 = {0};

/**************************************************************************************
* 函数名称：uart_config()
* 函数功能：配置 UART
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void uart_config(void)
{
    // 开启 USART1 DMA 发送请求
    USART1->CR3 |= USART_CR3_DMAT;
    
    // 开启 DMA 发送完成中断 (DMA1_Channel2 用于 USART1_TX)
    DMA1_Channel2->CCR |= DMA_CCR_TCIE;
    
    // 开启 RXNE 接收中断和 IDLE 空闲中断
    USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_IDLEIE;
    
    // 设置为接收模式
    Usart1RxEnable();
}

/**************************************************************************************
* 函数名称：Usart1TransmitterDMA()
* 函数功能：配置并启动 Usart1 DMA 发送 (使用 DMA1_Channel2)
* 输入参量：p->DataSize 数据长度
* 输出参量：无
***************************************************************************************/
void Usart1TransmitterDMA(volatile strUsart1Tx * p)
{
    Usart1TxEnable();
    USART1->CR1 |= USART_CR1_TE;
    
    // 禁用 DMA 通道以便重新配置
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    
    // 清除 DMA 标志
    DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CGIF2;
    
    // 配置 DMA 传输参数
    DMA1_Channel2->CMAR = (uint32_t)(p->Data);
    DMA1_Channel2->CPAR = (uint32_t)&USART1->TDR;
    DMA1_Channel2->CNDTR = p->DataSize;
    
    // 使能 DMA 通道
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

/**************************************************************************************
* 函数名称：DisableUARTReceive
* 函数功能：禁止 UART 接收器
* 输入参量：huart 串口句柄
* 输出参量：无
***************************************************************************************/
void DisableUARTReceive(UART_HandleTypeDef *huart) 
{
    huart->Instance->CR1 &= ~USART_CR1_RE;
}

/**************************************************************************************
* 函数名称：EnableUARTReceive
* 函数功能：使能 UART 接收器
* 输入参量：huart 串口句柄
* 输出参量：无
***************************************************************************************/
void EnableUARTReceive(UART_HandleTypeDef *huart) 
{
    huart->Instance->CR1 |= USART_CR1_RE;
}

/****************************************************************************************
* 函数名称：Usart1_Print
* 函数功能：通过 DMA 发送格式化字符串数据
* 输入参量：
* - format：格式化字符串
* - ...：可变参数
* 输出参量：无
****************************************************************************************/
void Usart1_Print(const char *format, ...)
{
    static char buffer[512];  
    va_list args;

    // 安全检查：如果 DMA 正在发送，则等待
    while (DMA1_Channel2->CNDTR != 0); 

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Usart1.Tx.Data = (uint8_t *)buffer;
    Usart1.Tx.DataSize = strlen(buffer);

    Usart1TransmitterDMA(&Usart1.Tx);
    
    // 等待发送完成（RS485 方向控制）
    while((GPIOA->ODR & (1 << 8)));
}
