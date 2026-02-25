#ifndef _TAMAGAWAENCODER_H_
#define _TAMAGAWAENCODER_H_

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
#define TmagawaEncoderStopTest          0
#define TmagawaEncoderMultiturnTest     1
#define TmagawaEncoderGetAllDataTest    2
#define TmagawaEncoderReadMTPTest       3
#define TmagawaEncoderSingleTurnTest    4
#define TmagawaEncoderReadMTPDefineTest 5

#define TamagawaEncoderPageNumber       6
#define TamagawaEncoderEepromAdress     0x80
#define TamagawaEncoderPNSAddress       0x7F//MGTMagneticEncoderPageNumberStorageAddress
#define TamagawaEncoderTxSize           4
#define TamagawaEncoderRxSize           11
//==============================================================================
// Variables
//==============================================================================
struct strTmgwSF{
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
  struct strTmgwSF  bit;  	
}	 uniTmgwSF;

struct strTmgwALMC{
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
  struct strTmgwALMC  bit;
}	 uniTmgwALMC;

//typedef union
//{
//  uint8_t *Data;
//  uint8_t DataSize;
//} UniTmgwTx;

struct strTmgwEE{
  uint8_t  Busy:1; 
  uint8_t  Write:1; 
  uint8_t  Read:1; 
  uint8_t  Res:5;  
};	
typedef union{
  uint8_t	           all;
  struct strTmgwEE  bit;
}	 uniTmgwEE;

typedef struct
{
  uint8_t  SetAddress;
  uint8_t  ReturnAddress;
  uint16_t Address;
  uint8_t  Data;	
  uint8_t  SetPage;
  uint8_t  ReturnPage;	
  uniTmgwEE Status;	
  uint8_t  DataBuffer[TamagawaEncoderPageNumber][TamagawaEncoderEepromAdress];
} strTmgwEep;

struct strTmgwFLAG{
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
  struct strTmgwFLAG  bit;
}	uniTmgwFLAG;
typedef struct{
  uint16_t  CF02;
  uint16_t  CF1A;
  uint16_t  CF32;
  uint16_t  CFEA;  
  uint16_t  CFBA;
  uint16_t  CFC2;
  uint16_t  CF62;
} strTmgwCNT;

typedef struct{
  uint8_t      TxID;
  uint8_t      RxID;	
  uniTmgwSF    Status;		   
  uint8_t      ResolutionID;
  uniTmgwALMC  Error;
  uint8_t      XorCrcData;
  uint32_t     SingleTurnPosition;	
  uint16_t     MultiTurnPosition;		
  uint8_t      TxData[TamagawaEncoderTxSize];
  uint8_t      RxData[TamagawaEncoderRxSize];	
  uint8_t      RxDataCnt;
  uint16_t     TimeoutCnt;
  uint8_t      XorCrcError;
  strTmgwEep   Eeprom;
  uniTmgwFLAG  Flag;
  strTmgwCNT   Cnt;  
  uint32_t     Data;
  uint32_t     Resolution;
  uint32_t     Phase;
  uint32_t     SinglePoleResolution;
  uint16_t        PhaseAngle;
  //UniTmgwTx    TmgwEncTx;
} strTmgwEnc;

typedef struct{
  uint16_t      No8;
  uint16_t      No27;
  float         No28;
  uint16_t      No35;
  uint16_t      No36;  
  uint16_t      No37;
  uint16_t      No38;
  uint16_t      No39;
  uint16_t      No42;
  uint16_t      No43;  
  uint16_t      No44;
  uint16_t      No45; 
  uint16_t      No46;   
} strRES;

typedef struct{
  uint8_t      EPSDriverType;
  uint8_t      EPSMotorType;
  uint8_t      EPSEncoderType;
  uint8_t      MotorType;
  uint8_t      EncoderType; 
  uint8_t      VoltageType;
  uint16_t     MotorPower;
  strRES       Reserve;
  uint16_t     MultiTurnPosition;
  uint16_t     RatedSpeed;
  uint16_t     MaximumSpeed;
  uint8_t      NumberOfPolePairs;
  uint8_t      OverSpeedLevel;
  float        RatedTorque;
  uint16_t     MaximumTorque;         
  float        RatedPeakCurrent;
  float        InstantaneousMaximumPeakCurrent;
  float        EMFConstant;
  uint16_t     RotorInertia;
  float        ArmatureWindingResistance;
  float        ArmatureInductance;
  uint16_t     OverloadDetectionBaseBurrent;
  uint16_t     OverloadDetectionIntermediateCurrent_1;
  uint16_t     OverloadDetectionIntermediateTime_1;
  uint16_t     OverloadDetectionIntermediateCurrent_2;
  uint16_t     OverloadDetectionIntermediateTime_2;
  float        Lq0;
  float        Lq1;
  uint8_t      RatedTorqueIndex;
  uint8_t      MomentOfInertiaIndex;
  uint8_t      RatedOutputIndex;
  uint8_t      RotationSpeedIndex;
  uint16_t     PhaseLow;
  uint16_t     PhaseHigh;
  uint8_t      CrcDataLow;
  uint8_t      CrcDataHigh;
  uint16_t     CrcData;
  uint16_t     CrcDataCalculate;
  
  float        RatedCurrent;
  float        OverloadCapacity;
  float        RotorInertiaFloat;
  float        OppositePotentialConstant;
  float        MagneticFluxAlarmLimit;
  float        TorqueAlarmLimit;
  float        TorqueOverloadThreshold;
  uint16_t      TwotimesOverloadTime;
  float        ElectricalAngle;
  float        CurrenLoopGain;
  float        CurrentLoopIntegrationTime;
  
  
} strMTP;
extern strTmgwEnc   TamagawaEncoder;
extern strMTP   MTP;
//==============================================================================
// Functions
//==============================================================================
void TamagawaEncoderRX(void);
__ramfunc void TamagawaEncoderTest(volatile uint8_t TestItems);
#endif /* _TAMAGAWAENCODER_H_ */
