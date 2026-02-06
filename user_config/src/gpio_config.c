/****************************************************************************************
  * @file     函数名称：gpio_config.c
  * @brief    IO口配置
  *           
****************************************************************************************/
#include "gpio_config.h"
/****************************************************************************************
* 函数名称：Get_GPIO_Output_Status
* 函数功能：获取GPIOA(PA0-PA7)和GPIOB(PB0-PB7)所有引脚的输出状态
* 输入参量：无
* 输出参量：uint16_t -> 16位数据，低8位表示PA0-PA7的输出状态，高8位表示PB0-PB7的输出状态
* 编写日期：2025-8-28
****************************************************************************************/
uint16_t Get_GPIO_Output_Status(void) {
    uint16_t status = 0;

    // 直接读取GPIOA和GPIOB的输出数据寄存器
    status |= (GPIOA->ODR & 0xFF);       // PA0到PA7的状态
    status |= ((GPIOB->ODR & 0xFF) << 8); // PB0到PB7的状态

    return status;
}

/****************************************************************************************
* 函数名称：Get_GPIOA_Output_Status
* 函数功能：根据输入的引脚编号(1-4)，获取对应GPIO引脚(PA0-PA3)的输出状态
* 输入参量：
*   - pin_index (uint8_t): 要查询的引脚编号 (1, 2, 3, 4)
*         - 1: 对应 PA0
*         - 2: 对应 PA1
*         - 3: 对应 PA2
*         - 4: 对应 PA3
* 输出参量：uint8_t -> 对应引脚的输出状态 (1 表示高电平, 0 表示低电平)
* 编写日期：2025-8-28
****************************************************************************************/
uint8_t Get_GPIOA_Output_Status(uint8_t pin_index) 
{
    // 直接读取整个GPIOA的输出数据寄存器
    uint32_t odr = GPIOA->ODR;

    switch (pin_index) {
        case 1:
            // 检查 PA0 (第0位) 的状态
            return (odr & GPIO_PIN_0) ? 1 : 0;
        
        case 2:
            // 检查 PA1 (第1位) 的状态
            return (odr & GPIO_PIN_1) ? 1 : 0;
            
        case 3:
            // 检查 PA2 (第2位) 的状态
            return (odr & GPIO_PIN_2) ? 1 : 0;
            
        case 4:
            // 检查 PA3 (第3位) 的状态
            return (odr & GPIO_PIN_3) ? 1 : 0;
            
        default:
            // 如果输入了无效的引脚编号，返回0或一个错误码
            return 0; 
    }
}

/****************************************************************************************
* 函数名称：Get_Relay_Status_By_StationID
* 函数功能：根据站号获取特定组的3个继电器的当前状态
* 输入参量：
*   - station_id：站号 (1-5)，用于选择要读取的GPIO组
* 输出参量：
*   - uint8_t -> 8位数据，其低3位(bit0-bit2)分别对应组内3个继电器的状态
*                 - bit0: 组内第1个继电器的状态 (0或1)
*                 - bit1: 组内第2个继电器的状态 (0或1)
*                 - bit2: 组内第3个继电器的状态 (0或1)
* 编写日期：2025-8-28
****************************************************************************************/
uint8_t Get_Relay_Status_By_StationID(uint8_t station_id) 
{
    uint8_t status = 0;

    switch (station_id) {
        case 1: // 站号1: 读取 PA0, PA1, PA2
            status |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) << 0); // 读取PA0状态，存入bit0
            status |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) << 1); // 读取PA1状态，存入bit1
            status |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) << 2); // 读取PA2状态，存入bit2
            break;

        case 2: // 站号2: 读取 PA3, PA4, PA5
            status |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) << 0);
            status |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) << 1);
            status |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) << 2);
            break;

        case 3: // 站号3: 读取 PA6, PA7, PB0
            status |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) << 0);
            status |= (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) << 1);
            status |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 2);
            break;

        case 4: // 站号4: 读取 PB1, PB2, PB3
            status |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) << 0);
            status |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) << 1);
            status |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) << 2);
            break;

        case 5: // 站号5: 读取 PB4, PB5, PB6
            status |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) << 0);
            status |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) << 1);
            status |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) << 2);
            break;

        default:
            // 如果站号无效，返回0
            status = 0;
            break;
    }

    return status;
}

