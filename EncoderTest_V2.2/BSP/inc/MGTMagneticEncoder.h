#ifndef _MGTMAGNETICENCODER_H_
#define _MGTMAGNETICENCODER_H_

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
#define MgtMagneticEncoderStopTest            0
#define MgtMagneticEncoderMultiturnTest       1
#define MgtMagneticEncoderGetAllDataTest      2
#define MgtMagneticEncoderReadAllEepromTest   3
#define MgtMagneticEncoderInitializeTest      4
#define MgtMagneticEncoderSpeedTest           5
#define MgtMagneticEncoderFirmwareVersionTest 6
#define MgtMagneticEncoderWriteAllEepromTest  7
#define MgtMagneticEncoderCommand0x32Test     8
#define MgtMagneticEncoderCommand0xEATest     9
#define MgtMagneticEncoderOIPTest             10//OpenInternalProtocol
#define MgtMagneticEncoderCIPTest             11//CloseInternalProtocol
#define MgtMagneticEncoderCommand0x7ATest     12
#define MgtMagneticEncoderCommand0xA2Test     13
#define MgtMagneticEncoderFirmwareTest        14
#define MgtMagneticEncoderWriteDateTest       15
#define MgtMagneticEncoderSetResolutionTest   16
#define MgtMagneticEncoderReadResolutionTest  17
#define MgtMagneticEncoderWriteHWRevTest      18
#define MgtMagneticEncoderReadHWRevTest       19
#define MgtMagneticEncoderTimeoutTest         20

#define MGTMagneticEncoderTxSize              0x05
#define MGTMagneticEncoderRxSize              0x0C
#define MGTMagneticEncoderPageNumber          6
#define MGTMagneticEncoderEepromAdress        0x80
#define MGTMagneticEncoderPNSAddress          0x7F//MGTMagneticEncoderPageNumberStorageAddress
//==============================================================================
// Variables
//==============================================================================
struct strMGTSF{
  uint8_t  dd0:1;	//bit0: fixed to "0"	
  uint8_t  dd1:1;       //bit1: fixed to "0"	
  uint8_t  dd2:1;       //bit2: fixed to "0"	
  uint8_t  dd3:1;       //bit3: fixed to "0"	
  uint8_t  ea0:1;	//bit4:	CE
  uint8_t  ea1:1;	//bit5:	Logic-OR of OH, ME, BE, BA
  uint8_t  ca0:1;	//bit6:	Parity error 
  uint8_t  ca1:1;	//bit7:	Delimiter error
};		
typedef union{
  uint8_t	         all;
  struct strMGTSF  bit;  	
}	 uniMGTSF;

struct strMGTALMC{
  uint8_t  OS:1; //bit0: Over speed	(SA)	
  uint8_t  FS:1; //bit1: Full absolute status (SA/SI)
  uint8_t  CE:1; //bit2: Counting error (SA/SI)
  uint8_t  OF:1; //bit3: Counter overflow (SA)
  uint8_t  OH:1; //bit4: Over heat (SA)		
  uint8_t  ME:1; //bit5: Multi-turn error (SA)		
  uint8_t  BE:1; //bit6: Battery error (SA)		
  uint8_t  BA:1; //bit7: Battery alarm (SA)	
};	
typedef union{
  uint8_t	           all;
  struct strMGTALMC  bit;
}	 uniMGTALMC;

//typedef union
//{
//  uint8_t *Data;
//  uint8_t DataSize;
//} UniTmgwTx;
struct strMGTEE{
  uint8_t  Busy:1; 
  uint8_t  Write:1; 
  uint8_t  Read:1; 
  uint8_t  Res:5;  
};	
typedef union{
  uint8_t	           all;
  struct strMGTEE  bit;
}	 uniMGTEE;

typedef struct
{
  uint8_t  SetAddress;
  uint8_t  ReturnAddress;
  uint16_t Address;
  uint8_t  Data;	
  uint8_t  SetPage;
  uint8_t  ReturnPage;	
  uniMGTEE Status;	
  uint8_t  DataBuffer[MGTMagneticEncoderPageNumber][MGTMagneticEncoderEepromAdress];
} strMGTEep;

struct strMGTFLAG{
  uint8_t  CF02:1; 	
  uint8_t  CF08:1; 
  uint8_t  CF92:1; 
  uint8_t  CF1A:1; 
  uint8_t  CF32:1; 
  uint8_t  CFEA:1; 	
  uint8_t  CFBA:1; 		
  uint8_t  CFC2:1; 	
  uint8_t  CF62:1; 
  uint8_t  CFPit:1;   
  uint8_t  Res:6;  
};	
typedef union{
  uint16_t	       all;
  struct strMGTFLAG  bit;
}	uniMGTFLAG;
typedef struct{
  uint8_t   CF02;
  uint8_t   CF1A;
  uint8_t   CF32;
  uint8_t   CFEA;  
  uint8_t   CFBA;
  uint8_t   CFC2;
  uint8_t   CF62;
  uint8_t   CFDA;  
  uint8_t   CFOIP;    
  uint8_t   CFCIP;   
} strMGTCNT;

typedef struct{
  uint8_t      TxID;
  uint8_t      RxID;	
  uniMGTSF     Status;		   
  uint8_t      ResolutionID;
  uniMGTALMC   Error;
  uint8_t      XorCrcData;
  uint32_t     SingleTurnPosition;	
  uint16_t     MultiTurnPosition;		
  uint8_t      TxData[MGTMagneticEncoderTxSize];
  uint8_t      RxData[MGTMagneticEncoderRxSize];
  uint8_t      RxDataCnt;
  uint16_t     TimeoutCnt;
  uint8_t      XorCrcError;
  strMGTEep    Eeprom;
  uniMGTFLAG   Flag;
  strMGTCNT    Cnt;  
  uint32_t     Data;
  uint8_t      FirmwareVersion;
  uint8_t      OIPStatus;//内部协议状态
  uint32_t     Firmware;
  uint8_t      TimeOutFlag;
  //UniTmgwTx    TmgwEncTx;
} strMGTEnc;	


extern strMGTEnc   MgtMagneticEncoder;

//==============================================================================
// Functions
//==============================================================================
void MgtMagneticEncoderRX(void);
__ramfunc void MgtMagneticEncoderTest(volatile uint8_t TestItems);
#endif /* _MGTMAGNETICENCODER_H_ */
