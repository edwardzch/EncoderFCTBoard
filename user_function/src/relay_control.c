/****************************************************************************************
  * @file      relay_control.c
  * @brief     继电器控制模块 (PA0-PA7, 高电平导通)
  ****************************************************************************************/
#include "relay_control.h"

/* 继电器引脚和端口数组，便于索引访问 */
static const uint16_t RelayPins[RELAY_COUNT] = {
    MCU_RLY_K1_Pin, MCU_RLY_K2_Pin, MCU_RLY_K3_Pin, MCU_RLY_K4_Pin,
    MCU_RLY_K5_Pin, MCU_RLY_K6_Pin, MCU_RLY_K7_Pin, MCU_RLY_K8_Pin
};

static GPIO_TypeDef* const RelayPorts[RELAY_COUNT] = {
    MCU_RLY_K1_GPIO_Port, MCU_RLY_K2_GPIO_Port, MCU_RLY_K3_GPIO_Port, MCU_RLY_K4_GPIO_Port,
    MCU_RLY_K5_GPIO_Port, MCU_RLY_K6_GPIO_Port, MCU_RLY_K7_GPIO_Port, MCU_RLY_K8_GPIO_Port
};

/**************************************************************************************
* 函数名称：Relay_Init
* 函数功能：初始化继电器GPIO（默认全部关闭）
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void Relay_Init(void)
{
    /* GPIO 时钟已由 CubeMX 初始化 */
    /* 初始状态全部关闭 */
    Relay_AllOff();
}

/**************************************************************************************
* 函数名称：Relay_On
* 函数功能：打开单个继电器
* 输入参量：relayNum - 继电器编号 (1-8)
* 输出参量：无
***************************************************************************************/
void Relay_On(uint8_t relayNum)
{
    if(relayNum >= 1 && relayNum <= RELAY_COUNT)
    {
        HAL_GPIO_WritePin(RelayPorts[relayNum - 1], RelayPins[relayNum - 1], GPIO_PIN_SET);
    }
}

/**************************************************************************************
* 函数名称：Relay_Off
* 函数功能：关闭单个继电器
* 输入参量：relayNum - 继电器编号 (1-8)
* 输出参量：无
***************************************************************************************/
void Relay_Off(uint8_t relayNum)
{
    if(relayNum >= 1 && relayNum <= RELAY_COUNT)
    {
        HAL_GPIO_WritePin(RelayPorts[relayNum - 1], RelayPins[relayNum - 1], GPIO_PIN_RESET);
    }
}

/**************************************************************************************
* 函数名称：Relay_Toggle
* 函数功能：切换单个继电器状态
* 输入参量：relayNum - 继电器编号 (1-8)
* 输出参量：无
***************************************************************************************/
void Relay_Toggle(uint8_t relayNum)
{
    if(relayNum >= 1 && relayNum <= RELAY_COUNT)
    {
        HAL_GPIO_TogglePin(RelayPorts[relayNum - 1], RelayPins[relayNum - 1]);
    }
}

/**************************************************************************************
* 函数名称：Relay_AllOn
* 函数功能：打开全部继电器
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void Relay_AllOn(void)
{
    for(uint8_t i = 1; i <= RELAY_COUNT; i++)
    {
        Relay_On(i);
    }
}

/**************************************************************************************
* 函数名称：Relay_AllOff
* 函数功能：关闭全部继电器
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void Relay_AllOff(void)
{
    for(uint8_t i = 1; i <= RELAY_COUNT; i++)
    {
        Relay_Off(i);
    }
}

/**************************************************************************************
* 函数名称：Relay_GetStatus
* 函数功能：获取继电器当前状态
* 输入参量：relayNum - 继电器编号 (1-8)
* 输出参量：1 = 导通, 0 = 断开
***************************************************************************************/
uint8_t Relay_GetStatus(uint8_t relayNum)
{
    if(relayNum >= 1 && relayNum <= RELAY_COUNT)
    {
        return (HAL_GPIO_ReadPin(RelayPorts[relayNum - 1], RelayPins[relayNum - 1]) == GPIO_PIN_SET) ? 1 : 0;
    }
    return 0;
}

/**************************************************************************************
* 函数名称：Relay_SetMultiple
* 函数功能：根据位掩码设置多个继电器
* 输入参量：mask - 位掩码 (bit0=K1, bit7=K8)
*           state - 1 = 导通, 0 = 断开
* 输出参量：无
***************************************************************************************/
void Relay_SetMultiple(uint8_t mask, uint8_t state)
{
    for(uint8_t i = 0; i < RELAY_COUNT; i++)
    {
        if(mask & (1 << i))
        {
            if(state)
                Relay_On(i + 1);
            else
                Relay_Off(i + 1);
        }
    }
}
