#ifndef _MULTITURNMAGNETICENCODER_H_
#define _MULTITURNMAGNETICENCODER_H_

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
#define MultiturnMagneticEncoderStopTest             0
#define MultiturnMagneticEncoderMultiturnTest        1
#define MultiturnMagneticEncoderGetAllDataTest       2
#define MultiturnMagneticEncoderReadAllEepromTest    3
#define MultiturnMagneticEncoderInitializeTest       4
#define MultiturnMagneticEncoderSpeedTest            5
#define MultiturnMagneticEncoderFirmwareVersionTest  6
#define MultiturnMagneticEncoderWriteAllEepromTest   7
#define MultiturnMagneticEncoderHallTest             8 
#define MultiturnMagneticEncoderWriteResultTest      9 
#define MultiturnMagneticEncoderTBTest               10//Temperature&BatteryVoltage
#define MultiturnMagneticEncoderOIPTest              11
#define MultiturnMagneticEncoderCIPTest              12
#define MultiturnMagneticEncoderWriteDateTest        13
#define MultiturnMagneticEncoderSetResolutionTest    14
#define MultiturnMagneticEncoderReadResolutionTest   15
#define MultiturnMagneticEncoderWriteHWRevTest       16
#define MultiturnMagneticEncoderWriteRegisterTest    17
#define MultiturnMagneticEncoderReadRegisterTest     18

#define MultiturnMagneticEncoderTxSize              10
#define MultiturnMagneticEncoderRxSize              135
#define MultiturnMagneticEncoderPageNumber          4
#define MultiturnMagneticEncoderEepromAdress        0x80
#define MultiturnMagneticEncoderPNSAddress          0x7F//MGTMagneticEncoderPageNumberStorageAddress
//==============================================================================
// Variables
//==============================================================================
struct strMULMGTSF{
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
  struct strMULMGTSF  bit;  	
}	 uniMULMGTSF;

struct strMULMGTALMC{
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
  struct strMULMGTALMC  bit;
}	 uniMULMGTALMC;

//typedef union
//{
//  uint8_t *Data;
//  uint8_t DataSize;
//} UniTmgwTx;
struct strMULMGTEE{
  uint8_t  Busy:1; 
  uint8_t  Write:1; 
  uint8_t  Read:1; 
  uint8_t  Res:5;  
};	
typedef union{
  uint8_t	           all;
  struct strMULMGTEE  bit;
}	 uniMULMGTEE;

typedef struct
{
  uint8_t SetAddress;
  uint8_t ReturnAddress;  
  uint8_t Address;
  uint8_t Data;	
  uint8_t SetPage;
  uint8_t ReturnPage;	  
  uint8_t Page;	
  uint8_t SetDNum;
  uint8_t ReturnDNum;
  uniMULMGTEE Status;
  uint8_t  DataBuffer[MultiturnMagneticEncoderPageNumber][MultiturnMagneticEncoderEepromAdress];
} strMULMGTEep;

struct strMULMGTFLAG{
  uint8_t  CF02:1; 	
  uint8_t  CF08:1; 
  uint8_t  CF92:1; 
  uint8_t  CF1A:1; 
  uint8_t  CF32:1; 
  uint8_t  CFEA:1; 	
  uint8_t  CFBA:1; 		
  uint8_t  CFC2:1; 	
  uint8_t  CF62:1; 
  uint8_t  CF25:1;   
  uint8_t  Res:6;  
};	
typedef union{
  uint16_t	       all;
  struct strMULMGTFLAG  bit;
}	uniMULMGTFLAG;
typedef struct{
  uint8_t   CF02;
  uint8_t   CF1A;
  uint8_t   CF32;
  uint8_t   CFEA;  
  uint8_t   CFBA;
  uint8_t   CFC2;
  uint8_t   CF62;
  uint8_t   CFDA; 
  uint8_t   CF25;   
  uint8_t   CFOIP;  
  uint8_t   CFCIP;   
  uint8_t   CFced;     
} strMULMGTCNT;

typedef struct{
  uint8_t   Sector;
  uint8_t   Buffer[4];
  uint8_t   Result; 
  uint8_t   Cnt;
  uint8_t   Check;
} strMULMGTHALL;

typedef struct{
  uint8_t       TxID;
  uint8_t       RxID;	
  uint8_t       IPID;//内部协议指令	
  uniMULMGTSF   Status;	
  uint8_t       OIPStatus;
  uint8_t       ResolutionID;
  uniMULMGTALMC Error;
  uint16_t      TimeoutCnt;
  uint32_t      SingleTurnPosition;	
  uint16_t      MultiTurnPosition;		
  uint8_t       TxData[MultiturnMagneticEncoderTxSize];
  uint8_t       RxData[MultiturnMagneticEncoderRxSize];	
  uint8_t       RxDataCnt;
  uint8_t       XorCrcData;  
  uint8_t       XorCrcError;
  strMULMGTEep  Eeprom;
  uniMULMGTFLAG Flag;
  strMULMGTCNT  Cnt;  
  uint32_t      Data;
  uint8_t       FirmwareVersion;
  strMULMGTHALL Hall;
  uint8_t       Status0x6D;
  uint8_t       Temperature;
  uint8_t       BatteryVoltage;
  uint8_t       ReadResolutionID;
  uint8_t        InternalAlarm1;  
  uint8_t        InternalAlarm2;
  uint8_t        InternalAlarm3;  
  //UniTmgwTx    TmgwEncTx;
} strMULMGTEnc;	


extern strMULMGTEnc   MultiturnMagneticEncoder;

//==============================================================================
// Functions
//==============================================================================
void MultiturnMagneticEncoderRX(void);
__ramfunc void MultiturnMagneticEncoderTest(volatile uint8_t TestItems);
#endif /* _MULTITURNMAGNETICENCODER_H_ */
