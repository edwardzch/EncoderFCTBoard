#ifndef _MODBUS_H_
#define _MODBUS_H_

#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include "fsl_uart_edma.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"


//==============================================================================
// Definitions
//==============================================================================
#define TamagawaEncoderBaudRate						2500000//
#define SensAREncoderBaudRate						115200//
#define ModBusBaudRate							57600				                         
#define ModBusAddress                                                   5

#define FWVersion  2.2

#define MULTITURNMagneticEncoder         1//多圈磁编
#define MULTITURNOpticalEncoder          2//多圈光编
#define MGTMagneticEncoder               3//单圈麦歌恩磁编
#define NXPMagneticEncoder               4//单圈NXP磁编
#define TAMAGAWAEncoder                  5//多摩川编码器
#define SENSAREncoder                    6//SensAR编码器

#define TuosidaMotor                     7               

#define SetSamplingCycle                 192000

#define EncoderTestFW                                             100
#define EncoderTestHW                                             200
#define ModBusTxSize      					  0x32
#define ModBusRxSize 					          0x32

#define Encoder_OIP_Test                                          1
#define Encoder_CIP_Test                                          2
#define Encoder_ced_Test                                          3
//==============================================================================
// Variables
//==============================================================================
//
typedef struct{
  uint8_t  		DataCnt;
  uint8_t  		DataAddrLow;         //发送起始数据地址(先高后低)
  uint8_t  		DataAddrHigh;        //发送起始数据地址(先高后低)
  uint16_t 		DataAddr;            //发送起始数据地址
  uint16_t 		Data;                //发送数据  
  uint8_t  		DataCountLow;        //发送数据字数(先高后低)
  uint8_t  		DataCountHigh;       //发送数据字数(先高后低)  
  uint16_t 		DataSize;            //发送数据数(以byte计算) 
  uint8_t  		DataLow;  
  uint8_t  		DataHigh;
  uint8_t  		Buffer[ModBusTxSize];//发送起始数据内容(先高后低)  
  uint8_t  		CRCLow;	             //发送CRC低位	
  uint8_t  		CRCHigh;             //发送CRC高位
  uint16_t 		CRCData;	
  //UniModBusTxDMA DMA;
} strTxData;	

typedef struct{
  uint8_t  		DataCnt;
  uint8_t  		DataAddrLow;         //接收起始数据地址(先高后低)
  uint8_t  		DataAddrHigh;        //接收起始数据地址(先高后低)
  uint16_t 		DataAddr;            //接收起始数据地址
  uint16_t 		Data;                //接收数据  
  uint8_t 		DataCountLow;        //接收数据字数(先高后低)
  uint8_t  		DataCountHigh;       //接收数据字数(先高后低)  
  uint16_t 		DataSize;            //接收数据数(以byte计算) 
  uint8_t  		DataLow;  
  uint8_t  		DataHigh;
  char  		Buffer[ModBusRxSize];//接收起始数据内容(先高后低)  
  uint8_t  		CRCLow;	            //接收CRC低位	
  uint8_t  		CRCHigh;             //接收CRC高位
  uint16_t 		CRCData;	
	
} strRxData;	

struct strFlag{
  uint8_t  		Busy0x03:1;	//
  uint8_t  		Busy0x06:1; //
  uint8_t  		Busy0x10:1; //
  uint8_t  		BaudRate:1; //  
  uint8_t               DmaBusy:1; 
  uint8_t               DriverRx:1; 
  uint8_t               DriverTx:1;  
  uint8_t               Pit:1; 	
};		
typedef union{
  uint8_t	        all;
  struct strFlag  bit;  	
}	uniFlag;

struct strModALMC{
  uint8_t  DC:1; //bit0: 编码器没有通讯上
  uint8_t  EC:1; //bit0: 编码器Crc校验错误
  uint8_t  SN:1; //bit0: SN位数错误
};	
typedef union{
  uint8_t	           all;
  struct strModALMC  bit;
}	uniModALMC;

typedef struct{
  uint8_t     ADR;
  uint8_t     CMD;
  uint8_t     TimeoutCnt;
  strTxData   TX;
  strRxData   RX;
  uniFlag     FLAG;
  uniModALMC  Error;
  uint32_t    PitCnt;
} strMod;	

extern strMod      ModBus;
extern uart_transfer_t uart1_transfer;
//==============================================================================
// Functions
//==============================================================================
void Init_UART1(void);
__ramfunc void ModBus_RTU(void);
__ramfunc uint16_t RTU_CRC(volatile uint8_t * Data, uint8_t Len);
void hexToAsciiString(uint8_t* hexArray, char* output, int length);
__ramfunc void UART1_SendDMA0(uart_transfer_t * p);
void ModBus_Tx06(volatile uint8_t DriveAddr, uint16_t DataAddr, uint16_t Data);
void ModBus_Tx03(volatile uint8_t DriveAddr, uint16_t DataAddr, uint16_t DataLen);
void ModBus_Rx03(void);
void ModBus_Rx06(void);
void ModBus_FCT(void);
void Encoder_Timeout(void);
void Encoder_CrcError(void);
void Encoder_CeError(void);
void Communication_Address_Error(void);
void ModBus_Crc_Error(void);
void NotSetEncoderType(void);
void SNDigitsAreIncorrect(void);
void TuosidaMTPprintf(void);
#endif /* _MODBUS_H_ */
