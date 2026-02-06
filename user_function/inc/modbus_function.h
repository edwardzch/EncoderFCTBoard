/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_FUNCTION_H
#define __MODBUS_FUNCTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"


/* 寄存器数组大小 */
#define MODBUS_REGISTER_COUNT 58

/* Modbus 功能码 */
#define MODBUS_FUNC_READ_HOLDING_REGISTERS  0x03
#define MODBUS_FUNC_READ_INPUT_REGISTERS    0x04
#define MODBUS_FUNC_WRITE_SINGLE_REGISTER   0x06
#define MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS 0x10

#define FirmwareVersion  1.0
/* Modbus 状态枚举 */
typedef enum {
    MODBUS_OK = 0,
    MODBUS_ERROR_CRC,
    MODBUS_ERROR_LENGTH,
    MODBUS_ERROR_EXCEPTION,
    MODBUS_ERROR_SLAVE_ADDR,
    MODBUS_ERROR_FUNC_CODE,
    MODBUS_ERROR_TIMEOUT
} ModbusStatus_t;

/* --- Modbus 数据结构定义 --- */

// 原始接收帧结构 (主要用于从机解析请求)
typedef struct{
	uint8_t		DataAddrHigh;
	uint8_t		DataAddrLow;
	uint16_t	DataAddr;
	uint8_t		DataCountHigh;
	uint8_t		DataCountLow;
	uint8_t     DataHigh[MODBUS_REGISTER_COUNT]; // 缓冲区大小
	uint8_t     DataLow[MODBUS_REGISTER_COUNT];
	uint16_t    Data[MODBUS_REGISTER_COUNT];
	uint8_t		DataSize; // 字节数
	uint8_t		CRCLow;
	uint8_t		CRCHigh;
} strModBusRx;

// 原始发送帧结构 (基本用不到，因为数据在函数内直接组装)
typedef struct{
	uint8_t		DataAddrHigh;
	uint8_t		DataAddrLow;
	uint16_t	DataAddr;
	uint8_t		DataCountHigh;
	uint8_t		DataCountLow;
	uint8_t		DataSize;
	uint8_t		CRCLow;
	uint8_t		CRCHigh;
} strModBusTx;

// 主机结构体
typedef struct{
	uint8_t    	ADDR; // 主机自身地址(通常为0或不使用)
	uint8_t    	CMD;  // 接收到的命令
	strModBusRx	Rx;   // 对主机接收意义不大，保留以兼容旧结构
	strModBusTx Tx;
  uint16_t    DisplayRegisters[MODBUS_REGISTER_COUNT]; // 存储从设备读回的数据

    // 【关键】保存上一次发送请求的信息
    struct {
        uint8_t  SlaveADDR;   // 请求的从站地址
        uint16_t StartAddr;   // 请求的起始寄存器地址
    } LastReq;

} strModBusMaster;

// 从机结构体
typedef struct{
	uint8_t    	ADDR; // 从机自身地址
	uint8_t    	CMD;  // 接收到的命令
	strModBusRx	Rx;
	strModBusTx Tx;
  uint16_t    DisplayRegisters[MODBUS_REGISTER_COUNT]; // 存储供主机读写的数据
} strModBusSlave;

typedef struct{
	strModBusMaster	Master;
	strModBusSlave  Slave;
} strModBus;

extern volatile strModBus   ModBus;
void ModBus_SlaveRx(void);
void Usart1_ReceiveStringHandler(void);
void Usart1_SendStringHandler(void);
#ifdef __cplusplus
}
#endif

#endif
