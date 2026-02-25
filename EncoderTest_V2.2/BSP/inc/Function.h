#ifndef _FUNCTION_H_
#define _FUNCTION_H_

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
#define BufferCnt                                                               16      
#define Check_Cnt                                                               8


//==============================================================================
// Variables
//==============================================================================
struct strMoterFLAG{
  uint8_t  ReadyTest:1; 	
  uint8_t  Res:7;  
};	
typedef union{
  uint8_t	       all;
  struct strMoterFLAG  bit;
}	uniMoterFLAG;

typedef struct{
  int32_t       DifBuffer[2];
  int32_t       Difference;
  uint8_t       StartTime;
  int32_t       ActualSpeed;
  int32_t       FilterSpeed;
  int16_t       SpeedBuffer[BufferCnt];
  uint32_t      Cnt;
  int32_t       AllData; 
  uint8_t       EncoderType;
  uint8_t       MTPType;
  uniMoterFLAG  Flag;
  uint32_t      FctPosition;
  int32_t       FctPos_MPos_Diff[2];
} strServoMotor;



struct strEncoderFLAG{
  uint8_t  PowerOn:1; 
  uint8_t  PowerOff:1;
  uint8_t  Year:1;
  uint8_t  Moon:1;
  uint8_t  Day:1;
  uint8_t  Hour:1;
  uint8_t  Res:2;  
};	
typedef union{
  uint8_t	       all;
  struct strEncoderFLAG  bit;
}	uniEncoderFLAG;

typedef struct{
  uint32_t        SingleTurnPosition;
  uint8_t         ResolutionID;
  uint8_t         SetResolutionID;
  uint16_t        MultiTurnPosition;
  uint32_t        PitCycle;
  uint16_t        SamplingCycle;
  uint32_t        SamplingCycleCnt;
  uint16_t        PitCnt;
  uint8_t         TestCycle;
  uint8_t         TestItem;
  uint8_t         TestYear; 
  uint8_t         TestMoon;
  uint8_t         TestDay;
  uint8_t         TestHour;
  uint8_t         TestDate;
  uint8_t         TestDateCnt; 
  uint8_t         HWRevData;
  uint8_t         HWRevCnt;
  uint16_t        HWRevAddr;
  uint8_t           HWRev[8];
  uint8_t           FWRev[8];  
  char            HWRevBuffer[8];       
  char            FWRevBuffer[8];       
  uniEncoderFLAG  Flag;
} strServoEncoder;

struct strMT6835AddrRegBit{
  uint8_t  Bit11:1;	//bit0: 	
  uint8_t  BitDA:1;      //bit1: 
  uint8_t  BitEA:1;      //bit2: 	
  uint8_t  BitEC:1;      //bit3: 	
  uint8_t  Bit12:1;	//bit4:	
  uint8_t  Res:3;	//bit5:	
};	
typedef union{
  uint8_t	      all;
  struct strMT6835AddrRegBit  bit;  	
}	uniRegBit;

typedef struct{
  uint8_t     Reg0x11_True;	
  uint8_t     Reg0xDA_True;
  uint8_t     Reg0xEA_True;
  uint8_t     Reg0xEC_True;
  uint8_t     Reg0x12_True;
  uint8_t     Reg0x11;	
  uint8_t     Reg0xDA;
  uint8_t     Reg0xEA;	
  uint8_t     Reg0xEC;
  uint8_t     Reg0x12;	
  uniRegBit   RegBit;
  uint8_t     TestRegData;
} strMT6835Addr;
typedef struct{
  uint8_t         Ones;
  uint8_t         Tenth;
  uint32_t        LightsTime;
} strDisplay;

extern strServoMotor     Motor;
extern strServoEncoder   MotorEncoder;
extern strDisplay        Display;
extern strMT6835Addr  MT6835Addr;	
//==============================================================================
// Functions
//==============================================================================

__ramfunc void UART_SendDMA0(uart_transfer_t * p);
__ramfunc uint8_t CRC8_Check(volatile uint8_t * Data, uint16_t Len);
void Variable_Reset(void);
void Motor_SpeedTest(volatile uint8_t Encoder_ID, volatile uint32_t SingleTurnPosition);
void EncoderPrintf(const char *format,...);
#endif /* _FUNCTION_H_ */
