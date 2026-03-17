#ifndef _MULTITURNOPTICALENCODER_H_
#define _MULTITURNOPTICALENCODER_H_

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
#define MultiturnOpticalEncoderStopTest              0
#define MultiturnOpticalEncoderMultiturnTest         1
#define MultiturnOpticalEncoderGetAllDataTest        2
#define MultiturnOpticalEncoderReadAllEepromTest     3
#define MultiturnOpticalEncoderInitializeTest        4
#define MultiturnOpticalEncoderSpeedTest             5
#define MultiturnOpticalEncoderFirmwareVersionTest   6
#define MultiturnOpticalEncoderWriteAllEepromTest    7
#define MultiturnOpticalEncoderAnalogDataTest        8 
#define MultiturnOpticalEncoderWriteResultTest       9 
#define MultiturnOpticalEncoderTBTest                10//Temperature&BatteryVoltage
#define MultiturnOpticalEncoderOIPTest               11
#define MultiturnOpticalEncoderCIPTest               12
#define MultiturnOpticalEncoderWriteDateTest         13
#define MultiturnOpticalEncoderDACTest               14
#define MultiturnOpticalEncoderLightIntensityTest    15
#define MultiturnOpticalEncoderReadAnalogDataTest    16
#define MultiturnOpticalEncoderReadInternalAlarmTest 17
#define MultiturnOpticalEncoderRMRNRSInitializeTest  18
#define MultiturnOpticalEncoderWriteHWRevTest        19
#define MultiturnOpticalEncoderSectorTest            20
#define MultiturnOpticalEncoderMNSPositionTest       21
#define MultiturnOpticalEncoderDACAdjustmentTest     22   
#define MultiturnOpticalEncoderMNSAnalogMaxCheckTest 23   

#define MultiturnOpticalEncoderTxSize              10
#define MultiturnOpticalEncoderRxSize              135
#define MultiturnOpticalEncoderPageNumber          4//32
#define MultiturnOpticalEncoderEepromAdress        0x80
#define MultiturnOpticalEncoderPNSAddress          0x7F//MGTOpticalEncoderPageNumberStorageAddress
//==============================================================================
// Variables
//==============================================================================
struct strMulOptSF{
  uint8_t  dd0:1;	//bit0: fixed to "0"	
  uint8_t  dd1:1; //bit1: fixed to "0"	
  uint8_t  dd2:1; //bit2: fixed to "0"	
  uint8_t  dd3:1; //bit3: fixed to "0"	
  uint8_t  ea0:1;	//bit4:	CE
  uint8_t  ea1:1;	//bit5:	Logic-OR of OH, ME, BE, BA
  uint8_t  ca0:1;	//bit6:	Parity error 
  uint8_t  ca1:1;	//bit7:	Delimiter error
};		
typedef union{
  uint8_t	         all;
  struct strMulOptSF  bit;  	
}	uniMulOptSF;

struct strMulOptALMC{
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
  struct strMulOptALMC  bit;
}	uniMulOptALMC;

//typedef union
//{
//  uint8_t *Data;
//  uint8_t DataSize;
//} UniAbsTx;
struct strMULOPTEE{
  uint8_t  Busy:1; 
  uint8_t  Write:1; 
  uint8_t  Read:1; 
  uint8_t  Res:5;  
};	
typedef union{
  uint8_t	           all;
  struct strMULOPTEE  bit;
}	 uniMULOPTEE;

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
  uniMULOPTEE Status;	
  uint8_t DataBuffer[MultiturnOpticalEncoderPageNumber][MultiturnOpticalEncoderEepromAdress];
} strMulOptEep;

struct strMulOptFLAG{
  uint8_t  CF02:1; 	
//  uint8_t  CF08:1; 
  uint8_t  CF92:1; 
  uint8_t  CF1A:1; 
  uint8_t  CF32:1; 
//  uint8_t  CFEA:1; 	
  uint8_t  CFBA:1; 		
  uint8_t  CFC2:1; 	
  uint8_t  CF62:1; 
  uint8_t  LedDacAdjust:1; 
  uint8_t  AMD:2;//AnalogMaxData
  uint8_t  LedDac:1;
  uint8_t  MTABHALL:3; 
  uint8_t  MNSAMDOneLapCheck:2;
};	
typedef union{
  uint16_t	       all;
  struct strMulOptFLAG  bit;
}	uniMulOptFLAG;

typedef struct{
  uint8_t  CF02;
  uint8_t  CF1A;
  uint8_t  CF32;
  uint8_t  CFEA;  
  uint8_t  CFBA;
  uint8_t  CFC2;
  uint8_t  CF62;
  uint8_t  CFOIP;
  uint8_t  CFCIP; 
  uint8_t  CFced;  
} strMulOptCNT;

typedef struct{
    uint16_t  Sector;
    uint8_t   Buffer[4];
    uint8_t   Result;      // 0:失败, 1:成功
    uint8_t   Cnt;         // 记录实际走的步数 (change_cnt)
    uint8_t   Check;       // 状态码 (0:测试中, 1:Pass, 2:Glitch, 3:Jitter, 4:CountErr)
    uint16_t  GlitchCnt;   // 【异常1】跨区突变计数 (diff==2)
    uint16_t  JitterCnt;   // 【异常2】抖动/反转计数 (新增, diff==3)
    uint8_t   CountErrCnt; // 【异常3】计数值异常计数 (新增, 一圈结束时!=4则+1)
} strMULOPTHALL;

typedef struct{
    uint16_t  Sector;
    uint8_t   Buffer[4];
    uint8_t   Result;      
    uint8_t   Cnt;         
    uint8_t   Check;       
    uint16_t  GlitchCnt;   
    uint16_t  JitterCnt;   // 新增
    uint8_t   CountErrCnt; // 新增
} strMULOPTMTAB;

typedef struct{
  uint8_t        TxID;
  uint8_t        RxID;	
  uint8_t        IPID;
  uniMulOptSF    Status;
  uint8_t        OIPStatus;
  uint8_t        ResolutionID;
  uniMulOptALMC  Error;
  uint8_t        XorCrcData;
  uint32_t       SingleTurnPosition;	
  uint16_t       MultiTurnPosition;		
  uint8_t        TxData[MultiturnOpticalEncoderTxSize];
  uint8_t        RxData[MultiturnOpticalEncoderRxSize];	
  uint8_t        RxDataCnt;
  uint16_t       TimeoutCnt;
  uint8_t        XorCrcError;
  strMulOptEep   Eeprom;
  //UniAbsTx    AbsEncTx;
  uniMulOptFLAG  Flag;
  strMulOptCNT   Cnt;  
  uint32_t       Data;
  int16_t        Speed;
  uint8_t        Status0x6D;
  int8_t         Temperature; 
  uint8_t        BatteryVoltage;
  uint8_t        InternalAlarm1;  
  uint8_t        InternalAlarm2;
  uint8_t        InternalAlarm3;
  uint8_t        TestResult;
  strMULOPTHALL  Hall;
  strMULOPTMTAB  MTAB;
} strMulOptEnc;	

typedef struct{
  uint16_t  SinData;
  uint16_t  CosData;
  uint16_t  Absolute;
  uint16_t  AbsoluteBuffer[2];
  int16_t   AbsoluteDifference;
  int16_t   Speed;
  int16_t   FeedbackSpeed;
  uint8_t   SpeedResult;
  uint8_t   SpeedFlag;
  uint16_t  SinDataMax;
  uint16_t  SinDataMin;
  uint16_t  CosDataMax;
  uint16_t  CosDataMin;  
  uint8_t   PositionFlag;
  uint16_t  MarkPositionBuffer[2];
  uint16_t  MarkPositionDifference;
} strMulOptEncM;	
typedef struct{
  uint16_t  SinData;
  uint16_t  CosData;
  uint16_t  Absolute;
  uint16_t  AbsoluteBuffer[2];
  int16_t   AbsoluteDifference;
  int16_t   Speed;
  int16_t   FeedbackSpeed;
  uint8_t   SpeedResult;
  uint8_t   SpeedFlag;
  uint16_t  SinDataMax;
  uint16_t  SinDataMin;
  uint16_t  CosDataMax;
  uint16_t  CosDataMin;  
} strMulOptEncN;
typedef struct{
  uint16_t  SinData;
  uint16_t  CosData;
  uint16_t  Absolute;
  uint16_t  AbsoluteBuffer[2];
  int16_t   AbsoluteDifference;
  int16_t   Speed;
  int16_t   FeedbackSpeed;
  uint8_t   SpeedResult;
  uint8_t   SpeedFlag;
  uint16_t  SinDataMax;
  uint16_t  SinDataMin;
  uint16_t  CosDataMax;
  uint16_t  CosDataMin; 
} strMulOptEncS;
typedef struct{
  uint8_t  MTABSector;
  uint8_t  MTABSectorraw ;
  uint8_t  HallSector;
  uint16_t MNdelta;
  uint8_t  Q4D;
  uint16_t Q14D;
  uint32_t AbsolutePosition;
  uint16_t LEDReadDAC;
  uint16_t LEDTestDAC;  
  uint8_t  LEDTestDACCnt;  
  uint8_t  LEDTestDACFlag;   
  uint8_t  LEDTestDACResult;
  uint16_t LEDAdjustDAC;
  uint8_t  LEDAdjustDACCnt; 
  int8_t   LEDAdjustDACData;
  uint8_t  MNSAnalogResult;
  uint8_t  MNSAnalogSynchronousCheckResult;  
  uint8_t  MNSAnalogWaitCnt;
  uint16_t MNSAnalogChekcCnt;
  uint16_t MSinDataBuffer[16];  
  uint8_t  LEDDAC0;
  uint8_t  LEDDAC1;  
  uint8_t  RMSQUA;
  uint8_t  RNSQUA;
  uint8_t  RSSQUA;
  strMulOptEncM M;
  strMulOptEncN N;
  strMulOptEncS S;  
  uint32_t SamplingCnt;
  uint16_t MNSSinDataMaxBuffer[30];  
  uint16_t MNSCosDataMaxBuffer[30];    
}strICPNH2612;

extern strMulOptEnc   MultiturnOpticalEncoder;
extern strICPNH2612 PNH2612; 

//==============================================================================
// Functions
//==============================================================================
__ramfunc void MultiturnOpticalEncoderTX(volatile uint8_t Idle, uint8_t Addr, uint8_t Data, uint8_t LEDDAC0, uint8_t LEDDAC1);
__ramfunc void MultiturnOpticalEncoderRX(void);
__ramfunc void MultiturnOpticalEncoderTest(volatile uint8_t TestItems);
#endif /* _MULTITURNOPTICALENCODER_H_ */
