#include "gpio_config.h"
#include "pit_config.h"
#include "uart_config.h"
#include "NXPMagneticEncoder.h"
#include "Function.h"
#include "ModBus.h"
//==============================================================================
// DEFINES
//==============================================================================


strNXPEnc  NxpMagneticEncoder = {0};
extern volatile uint8_t  Work_Alarm;
extern uart_transfer_t uart0_transfer;
//==============================================================================
// NxpMagneticEncoderRX
//==============================================================================
void NxpMagneticEncoderRX(void)
{
    //static uint8_t   i = 0;
    NxpMagneticEncoder.RxData[NxpMagneticEncoder.RxDataCnt++] = UART0->D;                                                     //Recieve request to auto clear flag     
    
    if(NxpMagneticEncoder.RxDataCnt >= 1){
      switch(NxpMagneticEncoder.RxData[0]){
      case 0x1A:
        if(NxpMagneticEncoder.RxDataCnt == 11){
          UART_EnableRx(UART0, false);
          NxpMagneticEncoder.TimeoutCnt = 0;
          NxpMagneticEncoder.RxDataCnt = 0;
          NxpMagneticEncoder.RxID = NxpMagneticEncoder.RxData[0];
          NxpMagneticEncoder.Status.all = NxpMagneticEncoder.RxData[1];
          NxpMagneticEncoder.SingleTurnPosition = ((NxpMagneticEncoder.RxData[4] << 16) | (NxpMagneticEncoder.RxData[3] << 8) | NxpMagneticEncoder.RxData[2]) & 0xFFFFFFFF;
          MotorEncoder.SingleTurnPosition = NxpMagneticEncoder.SingleTurnPosition;
          NxpMagneticEncoder.ResolutionID = NxpMagneticEncoder.RxData[5];
          MotorEncoder.ResolutionID = NxpMagneticEncoder.ResolutionID;
          NxpMagneticEncoder.MultiTurnPosition = ((NxpMagneticEncoder.RxData[7] << 8) | NxpMagneticEncoder.RxData[6]) & 0xFFFF;
          NxpMagneticEncoder.Error.all = NxpMagneticEncoder.RxData[9];
          NxpMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)NxpMagneticEncoder.RxData, 10); 
          if(NxpMagneticEncoder.XorCrcData != NxpMagneticEncoder.RxData[10]){
            NxpMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = NxpMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
          if(Motor.Flag.bit.ReadyTest){
            Motor_SpeedTest(NxpMagneticEncoder.ResolutionID, NxpMagneticEncoder.SingleTurnPosition);
          }
        }
        break;      
      case 0x02:
        if(NxpMagneticEncoder.RxDataCnt == 6){
          UART_EnableRx(UART0, false);
          NxpMagneticEncoder.TimeoutCnt = 0;
          NxpMagneticEncoder.RxDataCnt = 0;
          NxpMagneticEncoder.RxID = NxpMagneticEncoder.RxData[0];
          NxpMagneticEncoder.Status.all = NxpMagneticEncoder.RxData[1];
          NxpMagneticEncoder.SingleTurnPosition = ((NxpMagneticEncoder.RxData[4] << 16) | (NxpMagneticEncoder.RxData[3] << 8) | NxpMagneticEncoder.RxData[2]) & 0xFFFFFFFF;
          MotorEncoder.SingleTurnPosition = NxpMagneticEncoder.SingleTurnPosition;
          NxpMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)NxpMagneticEncoder.RxData, 5);
          if(NxpMagneticEncoder.XorCrcData != NxpMagneticEncoder.RxData[5]){
            NxpMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = NxpMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }
        break;
      case 0xDA:
        if(NxpMagneticEncoder.RxDataCnt == 3){
          UART_EnableRx(UART0, false);
          NxpMagneticEncoder.TimeoutCnt = 0;
          NxpMagneticEncoder.RxDataCnt = 0;
          NxpMagneticEncoder.RxID = NxpMagneticEncoder.RxData[0];
          NxpMagneticEncoder.Status.all = NxpMagneticEncoder.RxData[1];        
          NxpMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)NxpMagneticEncoder.RxData, 2);
          if(NxpMagneticEncoder.XorCrcData != NxpMagneticEncoder.RxData[2]){
            NxpMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = NxpMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }
        break;        
      case 0x62:
        if(NxpMagneticEncoder.RxDataCnt == 6){
          UART_EnableRx(UART0, false);
          NxpMagneticEncoder.TimeoutCnt = 0;
          NxpMagneticEncoder.RxDataCnt = 0;
          NxpMagneticEncoder.RxID = NxpMagneticEncoder.RxData[0];
          NxpMagneticEncoder.Status.all = NxpMagneticEncoder.RxData[1];
          NxpMagneticEncoder.SingleTurnPosition = ((NxpMagneticEncoder.RxData[4] << 16) | (NxpMagneticEncoder.RxData[3] << 8) | NxpMagneticEncoder.RxData[2]) & 0xFFFFFFFF;
          NxpMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)NxpMagneticEncoder.RxData, 5);
          if(NxpMagneticEncoder.XorCrcData != NxpMagneticEncoder.RxData[5]){
            NxpMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = NxpMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }
        break;        
      case 0xEA:
        if(NxpMagneticEncoder.RxDataCnt==4){
          UART_EnableRx(UART0, false);
          NxpMagneticEncoder.TimeoutCnt = 0;
          NxpMagneticEncoder.RxDataCnt = 0;
          NxpMagneticEncoder.RxID = NxpMagneticEncoder.RxData[0];
          NxpMagneticEncoder.Eeprom.ReturnAddress = NxpMagneticEncoder.RxData[1] & 0x7F;
          NxpMagneticEncoder.Eeprom.Status.bit.Busy = (NxpMagneticEncoder.RxData[1] >> 7) & 0xFF;
          NxpMagneticEncoder.Eeprom.DataBuffer[NxpMagneticEncoder.Eeprom.ReturnAddress] = NxpMagneticEncoder.RxData[2];                         
          NxpMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)NxpMagneticEncoder.RxData, 3);
          if(NxpMagneticEncoder.XorCrcData != NxpMagneticEncoder.RxData[3]){
            NxpMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = NxpMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }  
        break;
      case 0x32:
        if(NxpMagneticEncoder.RxDataCnt==4){
          UART_EnableRx(UART0, false);
          NxpMagneticEncoder.TimeoutCnt = 0;
          NxpMagneticEncoder.RxDataCnt = 0;
          NxpMagneticEncoder.RxID = NxpMagneticEncoder.RxData[0];
          NxpMagneticEncoder.Eeprom.ReturnAddress = NxpMagneticEncoder.RxData[1] & 0x7F;
          NxpMagneticEncoder.Eeprom.Status.bit.Busy = (NxpMagneticEncoder.RxData[1] >> 7) & 0xFF;
          NxpMagneticEncoder.Eeprom.DataBuffer[NxpMagneticEncoder.Eeprom.ReturnAddress] = NxpMagneticEncoder.RxData[2];                      
          NxpMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)NxpMagneticEncoder.RxData, 3);
          if(NxpMagneticEncoder.XorCrcData != NxpMagneticEncoder.RxData[3]){
            NxpMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = NxpMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }          
        break;
      case 0xA2:
        if(NxpMagneticEncoder.RxDataCnt==5){
          UART_EnableRx(UART0, false);
          NxpMagneticEncoder.TimeoutCnt = 0;
          NxpMagneticEncoder.RxDataCnt = 0;
          NxpMagneticEncoder.RxID = NxpMagneticEncoder.RxData[0];
          NxpMagneticEncoder.Eeprom.Address = (NxpMagneticEncoder.RxData[2]<<8 | NxpMagneticEncoder.RxData[1]);
          if(NxpMagneticEncoder.Eeprom.Address == 0x0080){
            NxpMagneticEncoder.FirmwareVersion = NxpMagneticEncoder.RxData[3];
          }
          NxpMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)NxpMagneticEncoder.RxData, 4);
          if(NxpMagneticEncoder.XorCrcData != NxpMagneticEncoder.RxData[4]){
            NxpMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = NxpMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
          MotorEncoder.TestItem = NxpMagneticEncoderStopTest;
        }           
        break;
      default:
          NxpMagneticEncoder.RxDataCnt = 0;
          break;              
      }
    } 
}
//==============================================================================
// NxpMagneticEncoderTX
//==============================================================================
void NxpMagneticEncoderTX(volatile uint8_t Idle, uint16_t Addr, uint8_t Data)
{
  switch(Idle){
  case 0x62:
    NxpMagneticEncoder.TimeoutCnt ++;
    NxpMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&NxpMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);    
    break;
  case 0x1A:
    NxpMagneticEncoder.TimeoutCnt ++;
    NxpMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&NxpMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;
  case 0x02:
    NxpMagneticEncoder.TimeoutCnt ++;
    NxpMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&NxpMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;
  case 0xDA:
    NxpMagneticEncoder.TimeoutCnt ++;
    NxpMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&NxpMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;        
  case 0xEA:
    NxpMagneticEncoder.TimeoutCnt ++;
    NxpMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    NxpMagneticEncoder.TxData[1] = Addr; 
    NxpMagneticEncoder.TxData[2] = CRC8_Check((uint8_t *)NxpMagneticEncoder.TxData, 2); 
    uart0_transfer.data = (uint8_t *)&NxpMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 3;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);         
    break;
  case 0x32:
    NxpMagneticEncoder.TimeoutCnt ++;
    NxpMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    NxpMagneticEncoder.TxData[1] = Addr; 
    NxpMagneticEncoder.TxData[2] = Data; 
    NxpMagneticEncoder.TxData[3] = CRC8_Check((uint8_t *)NxpMagneticEncoder.TxData, 3); 
    uart0_transfer.data = (uint8_t *)&NxpMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 4;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);    
    break;
  case 0xA2:
    NxpMagneticEncoder.TimeoutCnt ++;
    NxpMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    NxpMagneticEncoder.TxData[1] = (Addr & 0xFF); 
    NxpMagneticEncoder.TxData[2] = ((Addr >> 8)& 0xFF); 
    NxpMagneticEncoder.TxData[3] = CRC8_Check((uint8_t *)NxpMagneticEncoder.TxData, 3); 
    uart0_transfer.data = (uint8_t *)&NxpMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 4;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);       
    break;
  }
  if(NxpMagneticEncoder.TimeoutCnt > 9){
    ModBus.Error.bit.DC = 1;
    Work_Alarm = 0x01;
  }     
}
////==============================================================================
//// NxpMagneticEncoderMultiturnReset
////==============================================================================
//void NxpMagneticEncoderMultiturnReset(void)
//{
//  if(NxpMagneticEncoder.Cnt.CF62 < 10){
//    NxpMagneticEncoderTX(0x62, 0x00, 0x00);
//    NxpMagneticEncoder.Cnt.CF62 ++;
//    if(NxpMagneticEncoder.Cnt.CF62 == 10){
//      MotorEncoder.TestItem = NxpMagneticEncoderStopTest;
//    }
//  }
//}
//==============================================================================
// NxpMagneticEncoderReadZeroPage
//==============================================================================
void NxpMagneticEncoderReadOnePage(void)
{
  if(NxpMagneticEncoder.Eeprom.SetAddress < 0x80){
    NxpMagneticEncoderTX(0xEA, NxpMagneticEncoder.Eeprom.SetAddress, 0x00); 
    NxpMagneticEncoder.Eeprom.SetAddress ++;
  }else{  
    MotorEncoder.TestItem = NxpMagneticEncoderStopTest;
  }   
}

//==============================================================================
// NxpMagneticEncoderInitialize
//==============================================================================
void NxpMagneticEncoderInitialize(void)
{
  if(NxpMagneticEncoder.Cnt.CFDA < 10){
    NxpMagneticEncoderTX(0xDA, 0x00, 0x00); 
    NxpMagneticEncoder.Cnt.CFDA ++;
  }else{
    MotorEncoder.TestItem = NxpMagneticEncoderStopTest;
  }
}
//==============================================================================
// NxpMagneticEncoderSpeed
//==============================================================================
void NxpMagneticEncoderSpeed(void)
{
  NxpMagneticEncoderTX(0x1A, 0x00, 0x00);
  if(NxpMagneticEncoder.Cnt.CF1A < 100){
    NxpMagneticEncoder.Cnt.CF1A ++;
  }else{
    Motor.Flag.bit.ReadyTest = 1;
  }
}
//==============================================================================
// NxpMagneticEncoderTest
//==============================================================================
__ramfunc void NxpMagneticEncoderTest(volatile uint8_t TestItems)
{
  switch(TestItems){
  case NxpMagneticEncoderStopTest:
    
    break;
  case NxpMagneticEncoderMultiturnTest:
    //NxpMagneticEncoderMultiturnReset();
    break;
  case NxpMagneticEncoderGetAllDataTest:
    NxpMagneticEncoderTX(0x1A, 0x00, 0x00);
    break;
  case NxpMagneticEncoderReadAllEepromTest:
    NxpMagneticEncoderReadOnePage();
    break;
  case NxpMagneticEncoderInitializeTest:
    NxpMagneticEncoderInitialize();
    break;    
  case NxpMagneticEncoderSpeedTest:
    NxpMagneticEncoderSpeed();
    break;
  case NxpMagneticEncoderFirmwareVersionTest:
    NxpMagneticEncoderTX(0xA2, 0x0080, 0x00);
    break;
  }
}