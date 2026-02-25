//------------------------------------------------------------------------------ 
// INCLUDES
//------------------------------------------------------------------------------ 
#include "stdio.h"
#include "math.h"
#include "clock_config.h"
#include "uart_config.h"
#include "pit_config.h"
#include "gpio_config.h"
#include "ModBus.h"
#include "Function.h"
#include "TamagawaEncoder.h"
#include "SensAREncoder.h"
#include "MGTMagneticEncoder.h"
#include "MultiturnMagneticEncoder.h"
#include "NXPMagneticEncoder.h"
#include "MultiturnOpticalEncoder.h"
//------------------------------------------------------------------------------ 
// DEFINES
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
void Lights_Cycle_On(uint8_t Alarm);
void DisplayDate(uint8_t Date);
void SingleTurnPosition_Display(uint32_t SingleTurnPosition);
void MultiTurnPosition_Display(uint32_t MultiTurnPosition);
void DisplayFirmwareVersion(float FWData);
volatile uint32_t PITCnt = 0;
volatile uint8_t  Work_Alarm = 0;
volatile uint8_t  Variable_ReadReset_Flag = 0;
volatile uint8_t  EepromPage = 0,EepromAddr = 0,EepromData = 0,EepromCnt = 0;
extern volatile uint8_t  Variable_Reset_Flag;
//------------------------------------------------------------------------------ 
// PROTOTYPES
//------------------------------------------------------------------------------ 


//------------------------------------------------------------------------------ 
// CODE
//------------------------------------------------------------------------------ 
//==============================================================================
// main function
//==============================================================================
int main(void)
{ 
    BOARD_BootClockRUN();                                                       //Configurate clock
    Init_GPIO();  
    //Init_UART0();  
    Init_UART1();    
    Init_PIT();
    
//------------------------------------------------------------------------------
//主循环  
//------------------------------------------------------------------------------
    while(1){ 
      if(Work_Alarm == 0){
//        if(MotorEncoder.Flag.bit.PowerOn){
//          MultiTurnPosition_Display(MotorEncoder.MultiTurnPosition);
//          //SingleTurnPosition_Display(MotorEncoder.SingleTurnPosition);
//        }else 
        if(MotorEncoder.Flag.bit.Year){
          DisplayDate(MotorEncoder.TestYear);
        }else if(MotorEncoder.Flag.bit.Moon){
          DisplayDate(MotorEncoder.TestMoon);
        }else if(MotorEncoder.Flag.bit.Day){
          DisplayDate(MotorEncoder.TestDay);
        }else if(MotorEncoder.Flag.bit.Hour){
          DisplayDate(MotorEncoder.TestHour);
        }else{
          DisplayFirmwareVersion(FWVersion);
        }
      }else{
        Lights_Cycle_On(Work_Alarm);
      }
      
      //ModBus_FCT();
      Variable_Reset();
      
    }  
}

//==============================================================================
//UART0 RX Interrupt Handler
//void UART0_RX_TX_IRQHandler(void)
//==============================================================================
__ramfunc void UART0_RX_TX_IRQHandler(void)
{
  if(UART0->S1 & kUART_RxDataRegFullFlag){                                    //Read flag and auto clear flag
    UART_ClearStatusFlags(UART0, kUART_RxDataRegFullFlag);
    switch(Motor.EncoderType){
      case MULTITURNMagneticEncoder:
          MultiturnMagneticEncoderRX();
        break;
      case MULTITURNOpticalEncoder:
          MultiturnOpticalEncoderRX();   
        break;
      case MGTMagneticEncoder:
          MgtMagneticEncoderRX();
        break;
      case NXPMagneticEncoder:
          NxpMagneticEncoderRX();    
        break;
      case TAMAGAWAEncoder:
          TamagawaEncoderRX();
        break; 
      case SENSAREncoder:
          SensAR_RX();
        break; 
    default:
      
      break;
    } 
  }
//  if(UART0->S1 & kUART_IdleLineFlag){
//    UART_ClearStatusFlags(UART0, kUART_IdleLineFlag);
//    switch(Motor.EncoderType){
//      case MULTITURNMagneticEncoder:
//          MultiturnMagneticEncoderRX();
//        break;
//      case MULTITURNOpticalEncoder:
//          MultiturnOpticalEncoderRX();       
//        break;
//      case MGTMagneticEncoder:
//          MgtMagneticEncoderRX();  
//        break;
//      case NXPMagneticEncoder:
//          NxpMagneticEncoderRX();     
//        break;
//      case TAMAGAWAEncoder:
//          TamagawaEncoderRX();
//        break; 
//      case SENSAREncoder:
//          SensAR_RX();
//        break;          
//    }
//  }
}

//==============================================================================
//UART1 RX Interrupt Handler
//void UART1_RX_TX_IRQHandler(void)
//==============================================================================
__ramfunc void UART1_RX_TX_IRQHandler(void)
{
  if(UART1->S1 & kUART_RxDataRegFullFlag){
    UART_ClearStatusFlags(UART1, kUART_RxDataRegFullFlag);
    ModBus.RX.Buffer[ModBus.RX.DataCnt++] = UART1->D;
  }
  if(UART1->S1 & kUART_IdleLineFlag){                                    //Read flag and auto clear flag                                                  //Recieve request and transfer data
    UART_ClearStatusFlags(UART1, kUART_IdleLineFlag);
    ModBus_RTU();
    ModBus.RX.DataCnt = 0;
    ModBus.RX.Buffer[0] = 0;
    ModBus.RX.Buffer[1] = 0; 
  }
}



//==============================================================================
//DMA0 Interrupt Handler
//void DMA0_IRQHandler(void)
//==============================================================================
__ramfunc void DMA0_IRQHandler(void)
{
  if(UART0->S1 & kUART_TransmissionCompleteFlag){
    for (volatile uint16_t i=0; i<5; i++);
    Encoder485RE();
    UART_EnableRx(UART0, true);                                                 //50us delay compelete after request, Tx->Rx 
    EDMA_ClearChannelStatusFlags(DMA0, 0U, kEDMA_InterruptFlag);
    //传输完成后，开启DMA接收！
  }
}
//==============================================================================
//DMA1 Interrupt Handler
//void DMA1_IRQHandler(void)
//==============================================================================
__ramfunc void DMA1_IRQHandler(void)
{
  if(UART1->S1 & kUART_TransmissionCompleteFlag){
    EDMA_DisableChannelRequest(DMA0, 1U);//DMA0->ERQ &=~DMA_ERQ_ERQ1_MASK;
    for (volatile uint16_t i=0; i<50; i++);
    SerialPort485RE();
    UART_EnableRx(UART1, true);                                                 
    EDMA_ClearChannelStatusFlags(DMA0, 1U, kEDMA_InterruptFlag); 
    if(Variable_ReadReset_Flag){
      Variable_Reset_Flag = 0x01;
    }
    ModBus.FLAG.bit.DmaBusy = 0;
  }
}

//==============================================================================
//PIT0 Interrupt Handler
//void PIT0_IRQHandler(void)
//==============================================================================
__ramfunc void PIT0_IRQHandler(void)
{    
  if(PIT_GetStatusFlags(PIT, kPIT_Chnl_0) == kPIT_TimerFlag){
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag); 
//    if(MotorEncoder.PitCnt < 250){
//      MotorEncoder.PitCnt ++;
//      if(MotorEncoder.PitCnt == 250){
//        MotorEncoder.TestCycle = 1;
//        MotorEncoder.PitCnt = 0;
//      }
//    }
    switch(Motor.EncoderType){
      case MULTITURNMagneticEncoder:
          MultiturnMagneticEncoderTest(MotorEncoder.TestItem);  
        break;     
      case MULTITURNOpticalEncoder:
          MultiturnOpticalEncoderTest(MotorEncoder.TestItem);       
        break;         
      case MGTMagneticEncoder:
          MgtMagneticEncoderTest(MotorEncoder.TestItem);   
        break;      
      case NXPMagneticEncoder:
          NxpMagneticEncoderTest(MotorEncoder.TestItem);      
        break;      
      case TAMAGAWAEncoder:
          TamagawaEncoderTest(MotorEncoder.TestItem);
        break;   
      case SENSAREncoder:
          if(SensAR.CycleFlag){
            SensARTest(SensAR.TestContent);
          }
        break;         
    }      
  }
}

//==============================================================================
//PORTE_IRQn Interrupt Handler
//void PORTE_IRQHandler(void)
//==============================================================================
__ramfunc void PORTE_IRQHandler(void)
{
  if(PORTE->PCR[19] & PORT_PCR_ISF(1)){//
    PORT_ClearPinsInterruptFlags(PORTE, 1U << 19U);    

  }else if(PORTE->PCR[18] & PORT_PCR_ISF(1)){//
    PORT_ClearPinsInterruptFlags(PORTE, 1U << 18U);      
  }else if(PORTE->PCR[17] & PORT_PCR_ISF(1)){ //                                 
    PORT_ClearPinsInterruptFlags(PORTE, 1U << 17U);

  }else if(PORTE->PCR[16] & PORT_PCR_ISF(1)){  //
    PORT_ClearPinsInterruptFlags(PORTE, 1U << 16U);

  }
}

void Lights_TwoDigits(void)
{
    if(Display.LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H(); 
      switch(Display.Ones){
      case 0:
        Display0();
        break;
      case 1:
        Display1();
        break;
      case 2:
        Display2();
        break;
      case 3:
        Display3();
        break;
      case 4:
        Display4();
        break;
      case 5:
        Display5();
        break;
      case 6:
        Display6();
        break;
      case 7:
        Display7();
        break;
      case 8:
        Display8();
        break;
      case 9:
        Display9();
        break;
      default:
        
        break;
      }    
      Display.LightsTime ++;
    }else if(Display.LightsTime > 2000 && Display.LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      switch(Display.Tenth){
      case 0:
        Display0();
        break;
      case 1:
        Display1();
        break;
      case 2:
        Display2();
        break;
      case 3:
        Display3();
        break;
      case 4:
        Display4();
        break;
      case 5:
        Display5();
        break;
      case 6:
        Display6();
        break;
      case 7:
        Display7();
        break;
      case 8:
        Display8();
        break;
      case 9:
        Display9();
        break;
      default:
        
        break;
      }   
      Display.LightsTime ++;
    }else{
      Display.LightsTime = 0;
    }  
}

void Lights_TwoDigitsOnesDot(void)
{
    if(Display.LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H(); 
      switch(Display.Ones){
      case 0:
        Display0Dot();
        break;
      case 1:
        Display1Dot();
        break;
      case 2:
        Display2Dot();
        break;
      case 3:
        Display3Dot();
        break;
      case 4:
        Display4Dot();
        break;
      case 5:
        Display5Dot();
        break;
      case 6:
        Display6Dot();
        break;
      case 7:
        Display7Dot();
        break;
      case 8:
        Display8Dot();
        break;
      case 9:
        Display9Dot();
        break;
      default:
        
        break;
      }    
      Display.LightsTime ++;
    }else if(Display.LightsTime > 2000 && Display.LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      switch(Display.Tenth){
      case 0:
        Display0();
        break;
      case 1:
        Display1();
        break;
      case 2:
        Display2();
        break;
      case 3:
        Display3();
        break;
      case 4:
        Display4();
        break;
      case 5:
        Display5();
        break;
      case 6:
        Display6();
        break;
      case 7:
        Display7();
        break;
      case 8:
        Display8();
        break;
      case 9:
        Display9();
        break;
      default:
        
        break;
      }   
      Display.LightsTime ++;
    }else{
      Display.LightsTime = 0;
    }   
}

void Lights_TwoDigitsTenthDot(void)
{
    if(Display.LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H(); 
      switch(Display.Ones){
      case 0:
        Display0();
        break;
      case 1:
        Display1();
        break;
      case 2:
        Display2();
        break;
      case 3:
        Display3();
        break;
      case 4:
        Display4();
        break;
      case 5:
        Display5();
        break;
      case 6:
        Display6();
        break;
      case 7:
        Display7();
        break;
      case 8:
        Display8();
        break;
      case 9:
        Display9();
        break;
      default:
        
        break;
      }    
      Display.LightsTime ++;
    }else if(Display.LightsTime > 2000 && Display.LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      switch(Display.Tenth){
      case 0:
        Display0Dot();
        break;
      case 1:
        Display1Dot();
        break;
      case 2:
        Display2Dot();
        break;
      case 3:
        Display3Dot();
        break;
      case 4:
        Display4Dot();
        break;
      case 5:
        Display5Dot();
        break;
      case 6:
        Display6Dot();
        break;
      case 7:
        Display7Dot();
        break;
      case 8:
        Display8Dot();
        break;
      case 9:
        Display9Dot();
        break;
      default:
        
        break;
      }   
      Display.LightsTime ++;
    }else{
      Display.LightsTime = 0;
    }   
}

void Lights_TwoDigitsOnesDotTenthDot(void)
{
    if(Display.LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H(); 
      switch(Display.Ones){
      case 0:
        Display0Dot();
        break;
      case 1:
        Display1Dot();
        break;
      case 2:
        Display2Dot();
        break;
      case 3:
        Display3Dot();
        break;
      case 4:
        Display4Dot();
        break;
      case 5:
        Display5Dot();
        break;
      case 6:
        Display6Dot();
        break;
      case 7:
        Display7Dot();
        break;
      case 8:
        Display8Dot();
        break;
      case 9:
        Display9Dot();
        break;
      default:
        
        break;
      }    
      Display.LightsTime ++;
    }else if(Display.LightsTime > 2000 && Display.LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      switch(Display.Tenth){
      case 0:
        Display0Dot();
        break;
      case 1:
        Display1Dot();
        break;
      case 2:
        Display2Dot();
        break;
      case 3:
        Display3Dot();
        break;
      case 4:
        Display4Dot();
        break;
      case 5:
        Display5Dot();
        break;
      case 6:
        Display6Dot();
        break;
      case 7:
        Display7Dot();
        break;
      case 8:
        Display8Dot();
        break;
      case 9:
        Display9Dot();
        break;
      default:
        
        break;
      }   
      Display.LightsTime ++;
    }else{
      Display.LightsTime = 0;
    }   
}

void Lights_Cycle_On(uint8_t Alarm)
{
  static uint32_t LightsTime = 0;
  
  switch(Alarm){
  case 0x01://编码器通讯断开
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display1();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }
    break;
  case 0x02://编码器通讯CRC校验错误
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display2();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }  
    break;
  case 0x03://编码器计数出错 
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display3();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    } 
    break;
  case 0x04://ModBus通讯地址出错 
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display4();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }
    break;
  case 0x05://ModBusCRC出错 
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display5();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }   
    break;
  case 0x06:
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display6();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    } 
    break;
  case 0x07:
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display7();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }  
    break;
  case 0x08:
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display8();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    } 
    break;
  case 0x09:
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      Display9();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }
    break;
  case 0x0A:
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      DisplayA();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }  
    break;
  case 0x0B:
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      DisplayB();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }  
    break;
  case 0x0C:
    if(LightsTime <= 2000){
      DisplayClear();
      Ddig1L();
      Ddig2H();    
      DisplayC();      
      LightsTime ++;
    }else if(LightsTime > 2000 && LightsTime <= 4000){
      DisplayClear();
      Ddig1H(); 
      Ddig2L();
      DisplayE();
      LightsTime ++;
    }else{
      LightsTime = 0;
    }    
    break;    
  default:
    DisplayClear();
    Ddig2H();
    Ddig1H();
    Display0();  
    break;
  }
}

void SingleTurnPosition_Display(uint32_t SingleTurnPosition)
{    
    if(MotorEncoder.ResolutionID == 0x11){
      Display.Tenth = SingleTurnPosition / 100000;
      Display.Ones = SingleTurnPosition / 10000 - Display.Tenth * 10;  ;        
    }else if( MotorEncoder.ResolutionID == 0x17){
      Display.Tenth = SingleTurnPosition / 1000000;
      Display.Ones = SingleTurnPosition / 100000 - Display.Tenth * 10;        
    }
    Lights_TwoDigits();
}

void MultiTurnPosition_Display(uint32_t MultiTurnPosition)
{    
    if(MultiTurnPosition < 100){
      Display.Tenth = MultiTurnPosition / 10;
      Display.Ones = MultiTurnPosition - Display.Tenth * 10;
      Lights_TwoDigits();
    }else if(MultiTurnPosition >= 100 && MultiTurnPosition < 1000){
      Display.Tenth = MultiTurnPosition / 10 - (MultiTurnPosition / 100) * 10;
      Display.Ones = MultiTurnPosition - (MultiTurnPosition / 10) * 10;      
      Lights_TwoDigitsOnesDot(); 
    }else if(MultiTurnPosition >= 1000 && MultiTurnPosition < 10000){
      Display.Tenth = MultiTurnPosition / 10 - (MultiTurnPosition / 100) * 10;
      Display.Ones = MultiTurnPosition - (MultiTurnPosition / 10) * 10;      
      Lights_TwoDigitsTenthDot();       
    }else if(MultiTurnPosition >= 10000 && MultiTurnPosition < 65536){
      Display.Tenth = MultiTurnPosition / 10 - (MultiTurnPosition / 100) * 10;
      Display.Ones = MultiTurnPosition - (MultiTurnPosition / 10) * 10;      
      Lights_TwoDigitsOnesDotTenthDot();       
    }
}

void ConvertHexadecimalToDecimal(uint8_t Date)
{
    Display.Ones = Date%10;
    Date = Date/10;
    Display.Tenth = Date%10;
}
void DisplayDate(uint8_t Date)
{
    ConvertHexadecimalToDecimal(Date);
    Lights_TwoDigits();
}

void DisplayFirmwareVersion(float FWData)
{
  ConvertHexadecimalToDecimal(FWData*10);
  Lights_TwoDigitsTenthDot();
}