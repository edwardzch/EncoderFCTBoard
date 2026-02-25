#ifndef _SENSARENCODER_H_
#define _SENSARENCODER_H_

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
#define SensARTxSize                                                            0x64
#define SensARRxSize                                                            0x80

#define SensARStopTest       			                                0
#define SensAREnterTest       							1
#define SensARgetprdinfoTest                                                    2
#define SensARgethallvalue1Test      						3
#define SensARgethallvalue2Test      						4
#define SensARgethallvalue3Test      						5
#define SensARgethallvalue4Test      						6
#define SensARgethallvalue5Test      						7
#define SensARgethallvalue6Test      						8
#define SensARgethallvalue7Test      						9
#define SensARsetprdinfoTest      					  	10
#define SensARverTest      					  		11

#define SensAREnterDataID       						0x0001//0b000000000001
#define SensARgetprdinfoDataID       						0x0002//0b000000000010
#define SensARgethallvalue1DataID       				        0x0004//0b000000000100
#define SensARgethallvalue2DataID       				        0x0008//0b000000001000
#define SensARgethallvalue3DataID       				        0x0010//0b000000010000
#define SensARgethallvalue4DataID       				        0x0020//0b000000100000
#define SensARgethallvalue5DataID      					        0x0040//0b000001000000
#define SensARgethallvalue6DataID      					        0x0080//0b000010000000
#define SensARgethallvalue7DataID       				        0x0100//0b000100000000
#define SensARsetprdinfoDataID       						0x0200//0b001000000000
#define SensARverDataID       							0x0400//0b010000000000

struct strSensARFlag{
  uint8_t  Enter:1;		
  uint8_t  getprdinfo:1; 
  uint8_t  gethallvalue1:1; 
  uint8_t  gethallvalue2:1;
  uint8_t  gethallvalue3:1;
  uint8_t  gethallvalue4:1;	
  uint8_t  gethallvalue5:1;	
  uint8_t  gethallvalue6:1;
  uint8_t  gethallvalue7:1;
  uint8_t  setprdinfo:1;
  uint8_t  ver:1;
  uint8_t  GetEncoderData:1;
};		
typedef union{
  uint16_t	   				       all;
  struct strSensARFlag  bit;  	
}	uniSensARFlag;

typedef struct{
  uint8_t    	  TxID;
  uint8_t    	  RxID;		   
  uint8_t     	  ResolutionID;
  unsigned char  CrcData;
  uint32_t  	  SingleTurnPosition;	
  uint16_t  	  MultiTurnPosition;		
  uint8_t  	  TxData[SensARTxSize];
  uint8_t  	  RxData[SensARRxSize];	
  uint16_t 	  TimeoutCnt;
  uint8_t         CrcError;
  uint8_t         DataCnt;
  uniSensARFlag   Flag;
  uint8_t         TestContent;
  uint8_t         TestFlag;
  uint8_t         CycleFlag;
  char	          *DataString;
  uint8_t         RxDataCnt;
  uint16_t        Hall_1;
  uint16_t        Hall_2;
  uint16_t        Hall_3;
  uint16_t        Hall_4;
  uint16_t        Hall_5;
  uint16_t        Hall_6;
  uint16_t        Hall_7;
  char            getprdinfodata[14];	
  char            getprdinfo[16];
  char            setprdinfo[12];
  char            ReadEncoderPN[17];
  char            ReadEncoderRev[4];
  char            ReadEncoderSN[13];
  char            ReadPCBAPN[9];
  char            ReadPCBARev[4];
  char            ReadPCBASN[10];	
  char            StringCRC[5];
  char            SetEncoderPN[17];
  char            SetEncoderRev[4];	
  char            SetEncoderSN[13];	
  char            GetVER[75];	
  char            GetPrdInfoData[108];
  char            SetPrdInfoData[69];	//目前发现这个一定要定义在最后，否则printf的时候，会发送错误	
} strSensAR;	


extern strSensAR   SensAR;
void SensARTest(uint8_t TestContent);
void SensAR_RX(void);
void SensARDataInitialization(void);
#endif /* _SENSARENCODER_H_ */
