#ifndef _NXPMAGNETICENCODER_H_
#define _NXPMAGNETICENCODER_H_

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
#define NxpMagneticEncoderStopTest            0
#define NxpMagneticEncoderMultiturnTest       1
#define NxpMagneticEncoderGetAllDataTest      2
#define NxpMagneticEncoderReadAllEepromTest   3
#define NxpMagneticEncoderInitializeTest      4
#define NxpMagneticEncoderSpeedTest           5
#define NxpMagneticEncoderFirmwareVersionTest 6
#define NxpMagneticEncoderWriteAllEepromTest  7

#define NXPMagneticEncoderTxSize              5
#define NXPMagneticEncoderRxSize              12
#define NXPMagneticEncoderEepromAdress        128

//==============================================================================
// Variables
//==============================================================================
struct strNXPSF{
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
  struct strNXPSF  bit;  	
}	 uniNXPSF;

struct strNXPALMC{
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
  struct strNXPALMC  bit;
}	 uniNXPALMC;

//typedef union
//{
//  uint8_t *Data;
//  uint8_t DataSize;
//} UniTmgwTx;
struct strNXPEE{
  uint8_t  Busy:1; 
  uint8_t  Write:1; 
  uint8_t  Read:1; 
  uint8_t  Res:5;  
};	
typedef union{
  uint8_t	           all;
  struct strNXPEE  bit;
}	 uniNXPEE;

typedef struct
{
  uint8_t  SetAddress;
  uint8_t  ReturnAddress;
  uint16_t Address;
  uint8_t  Data;	
  uint8_t  SetPage;
  uint8_t  ReturnPage;	
  uniNXPEE Status;	
  uint8_t  DataBuffer[NXPMagneticEncoderEepromAdress];
} strNXPEep;

struct strNXPFLAG{
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
  struct strNXPFLAG  bit;
}	uniNXPFLAG;
typedef struct{
  uint8_t   CF02;
  uint8_t   CF1A;
  uint8_t   CF32;
  uint8_t   CFEA;  
  uint8_t   CFBA;
  uint8_t   CFC2;
  uint8_t   CF62;
  uint8_t   CFDA;  
} strNXPCNT;

typedef struct{
  uint8_t      TxID;
  uint8_t      RxID;	
  uniNXPSF     Status;		   
  uint8_t      ResolutionID;
  uniNXPALMC   Error;
  uint8_t      XorCrcData;
  uint32_t     SingleTurnPosition;	
  uint16_t     MultiTurnPosition;		
  uint8_t      TxData[NXPMagneticEncoderTxSize];
  uint8_t      RxData[NXPMagneticEncoderRxSize];
  uint8_t      RxDataCnt;
  uint16_t     TimeoutCnt;
  uint8_t      XorCrcError;
  strNXPEep    Eeprom;
  uniNXPFLAG   Flag;
  strNXPCNT    Cnt;  
  uint32_t     Data;
  uint8_t      FirmwareVersion;
  //UniTmgwTx    TmgwEncTx;
} strNXPEnc;	


extern strNXPEnc   NxpMagneticEncoder;

//==============================================================================
// Functions
//==============================================================================
void NxpMagneticEncoderRX(void);
__ramfunc void NxpMagneticEncoderTest(volatile uint8_t TestItems);
#endif /* _NXPMAGNETICENCODER_H_ */
