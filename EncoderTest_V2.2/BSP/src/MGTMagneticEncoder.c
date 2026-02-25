#include "gpio_config.h"
#include "pit_config.h"
#include "uart_config.h"
#include "MGTMagneticEncoder.h"
#include "Function.h"
#include "ModBus.h"
//==============================================================================
// DEFINES
//==============================================================================


strMGTEnc  MgtMagneticEncoder = {0};
extern volatile uint8_t  Work_Alarm;
extern uart_transfer_t uart0_transfer;
//==============================================================================
// MgtMagneticEncoderRX
//==============================================================================
void MgtMagneticEncoderRX(void)
{
    //static uint8_t   i = 0;
    MgtMagneticEncoder.RxData[MgtMagneticEncoder.RxDataCnt++] = UART0->D;                                                  //Recieve request to auto clear flag     
    
    if(MgtMagneticEncoder.RxDataCnt >= 1){
      switch(MgtMagneticEncoder.RxData[0]){
      case 0x1A:
        if(MgtMagneticEncoder.RxDataCnt == 11){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;
          MgtMagneticEncoder.RxDataCnt = 0;
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.Status.all = MgtMagneticEncoder.RxData[1];
          MgtMagneticEncoder.SingleTurnPosition = ((MgtMagneticEncoder.RxData[4] << 16) | (MgtMagneticEncoder.RxData[3] << 8) | MgtMagneticEncoder.RxData[2]) & 0xFFFFFFFF;
          MotorEncoder.SingleTurnPosition = MgtMagneticEncoder.SingleTurnPosition;
          MgtMagneticEncoder.ResolutionID = MgtMagneticEncoder.RxData[5];
          MotorEncoder.ResolutionID = MgtMagneticEncoder.ResolutionID;
          MgtMagneticEncoder.MultiTurnPosition = ((MgtMagneticEncoder.RxData[7] << 8) | MgtMagneticEncoder.RxData[6]) & 0xFFFF;
          MgtMagneticEncoder.Error.all = MgtMagneticEncoder.RxData[9];
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 10); 
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[10]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
          if(Motor.Flag.bit.ReadyTest){
            Motor_SpeedTest(MgtMagneticEncoder.ResolutionID, MgtMagneticEncoder.SingleTurnPosition);
          }
        }
        break;      
      case 0x02:
        if(MgtMagneticEncoder.RxDataCnt == 6){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;
          MgtMagneticEncoder.RxDataCnt = 0;
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.Status.all = MgtMagneticEncoder.RxData[1];
          MgtMagneticEncoder.SingleTurnPosition = ((MgtMagneticEncoder.RxData[4] << 16) | (MgtMagneticEncoder.RxData[3] << 8) | MgtMagneticEncoder.RxData[2]) & 0xFFFFFFFF;
          MotorEncoder.SingleTurnPosition = MgtMagneticEncoder.SingleTurnPosition;
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 5);
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[5]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }
        break;
      case 0xDA:
        if(MgtMagneticEncoder.RxDataCnt == 3){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;   
          MgtMagneticEncoder.RxDataCnt = 0;
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.Status.all = MgtMagneticEncoder.RxData[1];        
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 2);
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[2]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }
        break;        
      case 0x62:
        if(MgtMagneticEncoder.RxDataCnt == 6){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;
          MgtMagneticEncoder.RxDataCnt = 0;
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.Status.all = MgtMagneticEncoder.RxData[1];
          MgtMagneticEncoder.SingleTurnPosition = ((MgtMagneticEncoder.RxData[4] << 16) | (MgtMagneticEncoder.RxData[3] << 8) | MgtMagneticEncoder.RxData[2]) & 0xFFFFFFFF;
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 5);
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[5]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }
        break;        
      case 0xEA:
        if(MgtMagneticEncoder.RxDataCnt==4){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;
          MgtMagneticEncoder.RxDataCnt = 0;          
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.Eeprom.ReturnAddress = MgtMagneticEncoder.RxData[1] & 0x7F;
          if(MgtMagneticEncoder.Eeprom.ReturnAddress == MGTMagneticEncoderPNSAddress){
            MgtMagneticEncoder.Eeprom.ReturnPage = MgtMagneticEncoder.RxData[2];
            MgtMagneticEncoder.Eeprom.Status.bit.Busy = (MgtMagneticEncoder.RxData[1] >> 7) & 0xFF;
            MgtMagneticEncoder.Eeprom.DataBuffer[MgtMagneticEncoder.Eeprom.ReturnPage][MgtMagneticEncoder.Eeprom.ReturnAddress] = MgtMagneticEncoder.RxData[2];   
          }else if(MgtMagneticEncoder.Eeprom.ReturnAddress < MGTMagneticEncoderPNSAddress){
            MgtMagneticEncoder.Eeprom.Status.bit.Busy = (MgtMagneticEncoder.RxData[1] >> 7) & 0xFF;
            MgtMagneticEncoder.Eeprom.DataBuffer[MgtMagneticEncoder.Eeprom.ReturnPage][MgtMagneticEncoder.Eeprom.ReturnAddress] = MgtMagneticEncoder.RxData[2];            
          }   
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 3);
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[3]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }  
        break;
      case 0x32:
        if(MgtMagneticEncoder.RxDataCnt==4){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;
          MgtMagneticEncoder.RxDataCnt = 0;         
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.Eeprom.ReturnAddress = MgtMagneticEncoder.RxData[1] & 0x7F;
          if(MgtMagneticEncoder.Eeprom.ReturnAddress == MGTMagneticEncoderPNSAddress){
            MgtMagneticEncoder.Eeprom.ReturnPage = MgtMagneticEncoder.RxData[2];
            MgtMagneticEncoder.Eeprom.Status.bit.Busy = (MgtMagneticEncoder.RxData[1] >> 7) & 0xFF;
            MgtMagneticEncoder.Eeprom.DataBuffer[MgtMagneticEncoder.Eeprom.ReturnPage][MgtMagneticEncoder.Eeprom.ReturnAddress] = MgtMagneticEncoder.RxData[2];            
          }else if(MgtMagneticEncoder.Eeprom.ReturnAddress < MGTMagneticEncoderPNSAddress){
            MgtMagneticEncoder.Eeprom.Status.bit.Busy = (MgtMagneticEncoder.RxData[1] >> 7) & 0xFF;
            MgtMagneticEncoder.Eeprom.DataBuffer[MgtMagneticEncoder.Eeprom.ReturnPage][MgtMagneticEncoder.Eeprom.ReturnAddress] = MgtMagneticEncoder.RxData[2];            
          }
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 3);
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[3]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }          
        break;
      case 0xA2:
        if(MgtMagneticEncoder.RxDataCnt==5){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;
          MgtMagneticEncoder.RxDataCnt = 0;          
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.Eeprom.Address = ((MgtMagneticEncoder.RxData[2]<<8 | MgtMagneticEncoder.RxData[1]))&0x7FFFF;
          MgtMagneticEncoder.Eeprom.Status.bit.Busy = (MgtMagneticEncoder.Eeprom.Address >> 15) & 0xFFFF;
          if(MgtMagneticEncoder.Eeprom.Status.bit.Busy == 0){
            if(MgtMagneticEncoder.Eeprom.Address == 0x0305){
              MgtMagneticEncoder.FirmwareVersion = MgtMagneticEncoder.RxData[3];
            }
            MgtMagneticEncoder.Eeprom.Data = MgtMagneticEncoder.RxData[3];
          }
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 4);
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[4]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }
        }           
        break;
      case 0x7A:
        if(MgtMagneticEncoder.RxDataCnt==5){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;
          MgtMagneticEncoder.RxDataCnt = 0;  
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.Eeprom.Address = ((MgtMagneticEncoder.RxData[2]<<8 | MgtMagneticEncoder.RxData[1]))&0x7FFFF;
          MgtMagneticEncoder.Eeprom.Status.bit.Busy = (MgtMagneticEncoder.Eeprom.Address >> 15) & 0xFFFF;
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 4);
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[4]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }         
        }          
        break;
      case 0x6D:
        if(MgtMagneticEncoder.RxDataCnt==6){
          UART_EnableRx(UART0, false);
          MgtMagneticEncoder.TimeoutCnt = 0;
          MgtMagneticEncoder.RxDataCnt = 0;      
          MgtMagneticEncoder.RxID = MgtMagneticEncoder.RxData[0];
          MgtMagneticEncoder.OIPStatus = MgtMagneticEncoder.RxData[4];
          if(MgtMagneticEncoder.Cnt.CFOIP == 5){
            MotorEncoder.TestItem = MgtMagneticEncoderStopTest;      
            MgtMagneticEncoder.Cnt.CFOIP = 0;
            if(MgtMagneticEncoder.OIPStatus == 0){
              Work_Alarm = 0x07;
            }           
          }
          if(MgtMagneticEncoder.OIPStatus == 4){
             Work_Alarm = 0x08;
          }
          MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 5);
          if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[5]){
            MgtMagneticEncoder.XorCrcError = 1;
            ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
            Work_Alarm = 0x02;
          }          
        }
        break;
      case 0x40:
        if(MgtMagneticEncoder.RxDataCnt>=3){
          switch(MgtMagneticEncoder.RxData[1]){
          case 0x10:
            if(MgtMagneticEncoder.RxDataCnt==7){
              UART_EnableRx(UART0, false);
              MgtMagneticEncoder.TimeoutCnt = 0;
              MgtMagneticEncoder.RxDataCnt = 0;  
              MgtMagneticEncoder.Firmware = ((MgtMagneticEncoder.RxData[2] << 24) | (MgtMagneticEncoder.RxData[3] << 16) | (MgtMagneticEncoder.RxData[4] << 8) | MgtMagneticEncoder.RxData[5]) & 0xFFFFFFFF;
              MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 6);
              if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[6]){
                MgtMagneticEncoder.XorCrcError = 1;
                ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
                Work_Alarm = 0x02;
              }               
            }
            break;
          case 0x45:
            if(MgtMagneticEncoder.RxDataCnt==4){
              UART_EnableRx(UART0, false);
              MgtMagneticEncoder.TimeoutCnt = 0;
              MgtMagneticEncoder.RxDataCnt = 0;  
              MgtMagneticEncoder.ResolutionID = MgtMagneticEncoder.RxData[2];
              MgtMagneticEncoder.XorCrcData = CRC8_Check((uint8_t *)MgtMagneticEncoder.RxData, 3);
              if(MgtMagneticEncoder.XorCrcData != MgtMagneticEncoder.RxData[3]){
                MgtMagneticEncoder.XorCrcError = 1;
                ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
                Work_Alarm = 0x02;
              }               
            }
            break; 
          case 0x46:
            if(MgtMagneticEncoder.RxDataCnt==3){
              UART_EnableRx(UART0, false);
              MgtMagneticEncoder.TimeoutCnt = 0;
              MgtMagneticEncoder.RxDataCnt = 0;  
              if(MgtMagneticEncoder.RxData[2] != 0xF0){
                MgtMagneticEncoder.XorCrcError = 1;
                ModBus.Error.bit.EC = MgtMagneticEncoder.XorCrcError;
                Work_Alarm = 0x02;
              }               
            }
            break;            
          }
        }
        break;
      default:
          MgtMagneticEncoder.RxDataCnt = 0;
          break;              
      }
    }
}
//==============================================================================
// MgtMagneticEncoderTX
//==============================================================================
void MgtMagneticEncoderTX(volatile uint8_t Idle, uint16_t Addr, uint8_t Data)
{
  switch(Idle){
  case 0x62:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);    
    break;
  case 0x1A:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;
  case 0x02:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;
  case 0xDA:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;        
  case 0xEA:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    MgtMagneticEncoder.TxData[1] = Addr; 
    MgtMagneticEncoder.TxData[2] = CRC8_Check((uint8_t *)MgtMagneticEncoder.TxData, 2); 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 3;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);         
    break;
  case 0x32:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    MgtMagneticEncoder.TxData[1] = Addr; 
    MgtMagneticEncoder.TxData[2] = Data; 
    MgtMagneticEncoder.TxData[3] = CRC8_Check((uint8_t *)MgtMagneticEncoder.TxData, 3); 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 4;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);    
    break;
  case 0xA2:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    MgtMagneticEncoder.TxData[1] = (Addr & 0xFF); 
    MgtMagneticEncoder.TxData[2] = ((Addr >> 8)& 0xFF); 
    MgtMagneticEncoder.TxData[3] = CRC8_Check((uint8_t *)MgtMagneticEncoder.TxData, 3); 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 4;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);       
    break;
  case 0x7A:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF                 
    MgtMagneticEncoder.TxData[1] = (Addr & 0xFF); 
    MgtMagneticEncoder.TxData[2] = ((Addr >> 8)& 0xFF); 
    MgtMagneticEncoder.TxData[3] = Data;
    MgtMagneticEncoder.TxData[4] = CRC8_Check((uint8_t *)MgtMagneticEncoder.TxData, 4); 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 5;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);     break;
  case 0x6D:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF  
    MgtMagneticEncoder.TxData[1] = Data; 
    MgtMagneticEncoder.TxData[2] = 0x49; 
    MgtMagneticEncoder.TxData[3] = 0x50; 
    MgtMagneticEncoder.TxData[4] = CRC8_Check((uint8_t *)MgtMagneticEncoder.TxData, 4); 
    uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
    uart0_transfer.dataSize = 5;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);          
    break;
  case 0x40:
    MgtMagneticEncoder.TimeoutCnt ++;
    MgtMagneticEncoder.TxData[0] = Idle;                                                //CF  
    MgtMagneticEncoder.TxData[1] = (uint8_t)(Addr & 0xFF); 
    if(MgtMagneticEncoder.TxData[1] == 0x46){
      MgtMagneticEncoder.TxData[2] = Data;
      MgtMagneticEncoder.TxData[3] = CRC8_Check((uint8_t *)MgtMagneticEncoder.TxData, 3); 
      uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
      uart0_transfer.dataSize = 4;       
    }else{
      MgtMagneticEncoder.TxData[2] = CRC8_Check((uint8_t *)MgtMagneticEncoder.TxData, 2); 
      uart0_transfer.data = (uint8_t *)&MgtMagneticEncoder.TxData[0];
      uart0_transfer.dataSize = 3;         
    }            
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);     
    break;
  }
  if(MgtMagneticEncoder.TimeoutCnt > 5){
    ModBus.Error.bit.DC = 1;
    Work_Alarm = 0x01;
  }     
}
////==============================================================================
//// MgtMagneticEncoderMultiturnReset
////==============================================================================
//void MgtMagneticEncoderMultiturnReset(void)
//{
//  if(MgtMagneticEncoder.Cnt.CF62 < 10){
//    MgtMagneticEncoderTX(0x62, 0x00, 0x00);
//    MgtMagneticEncoder.Cnt.CF62 ++;
//    if(MgtMagneticEncoder.Cnt.CF62 == 10){
//      MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
//    }
//  }
//}
//==============================================================================
// MgtMagneticEncoderReadZeroPage
//==============================================================================
void MgtMagneticEncoderReadOnePage(void)
{
  if(MgtMagneticEncoder.Eeprom.SetAddress < 0x80){
    MgtMagneticEncoderTX(0xEA, MgtMagneticEncoder.Eeprom.SetAddress, 0x00); 
    MgtMagneticEncoder.Eeprom.SetAddress ++;
  }else{     
    if(MgtMagneticEncoder.Eeprom.Status.bit.Write == 0){
      MgtMagneticEncoder.Eeprom.SetPage ++;
      if(MgtMagneticEncoder.Eeprom.SetPage < MGTMagneticEncoderPageNumber){        
        MgtMagneticEncoderTX(0x32, MGTMagneticEncoderPNSAddress, MgtMagneticEncoder.Eeprom.SetPage); 
        MgtMagneticEncoder.Eeprom.Status.bit.Write = 1;
      }else{  
        MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
      }
    }else{
      MgtMagneticEncoderTX(0xEA, MGTMagneticEncoderPNSAddress, 0x00); 
      MgtMagneticEncoder.Eeprom.Status.bit.Write = 0;
      MgtMagneticEncoder.Eeprom.SetAddress = 0;
      MgtMagneticEncoder.Eeprom.ReturnAddress = 0;         
    }
  }   
}
//==============================================================================
// MgtMagneticEncoderReadAllEeprom
//==============================================================================
void MgtMagneticEncoderReadAllEeprom(void)
{
  if(MgtMagneticEncoder.Eeprom.SetPage < MGTMagneticEncoderPageNumber){
    if(MgtMagneticEncoder.Eeprom.Status.bit.Busy == 0){
      MgtMagneticEncoderReadOnePage(); 
    }else{
      MgtMagneticEncoderTX(0xEA, MGTMagneticEncoderPNSAddress, 0x00); 
    }      
  }
}
//==============================================================================
// MgtMagneticEncoderWriteOnePage
//==============================================================================
void MgtMagneticEncoderWriteOnePage(void)
{
  if(MgtMagneticEncoder.Eeprom.ReturnAddress < 0x80){
    if(MgtMagneticEncoder.Eeprom.Status.bit.Busy == 0){
      if(MgtMagneticEncoder.Eeprom.Status.bit.Read){
        MgtMagneticEncoderTX(0xEA, MgtMagneticEncoder.Eeprom.ReturnAddress, 0x00); 
        MgtMagneticEncoder.Eeprom.Status.bit.Read = 0;
      }else{
        if(MgtMagneticEncoder.Eeprom.ReturnAddress == 0x7E){
          MgtMagneticEncoder.Eeprom.SetPage ++;
          MgtMagneticEncoderTX(0x32, MGTMagneticEncoderPNSAddress, MgtMagneticEncoder.Eeprom.SetPage); 
          MgtMagneticEncoder.Eeprom.SetAddress = 0;
          MgtMagneticEncoder.Eeprom.ReturnAddress = 0;
          MgtMagneticEncoder.Eeprom.Status.bit.Read = 1;
        }else{
          MgtMagneticEncoderTX(0x32, MgtMagneticEncoder.Eeprom.SetAddress, (MgtMagneticEncoder.Eeprom.SetPage+1)); 
          MgtMagneticEncoder.Eeprom.Status.bit.Read = 1;
          MgtMagneticEncoder.Eeprom.SetAddress ++;          
        }
      }
    }else{
      MgtMagneticEncoderTX(0xEA, MgtMagneticEncoder.Eeprom.ReturnAddress, 0x00); 
    }    
  }
}
//==============================================================================
// MgtMagneticEncoderWriteAllEeprom
//==============================================================================
void MgtMagneticEncoderWriteAllEeprom(void)
{
  if(MgtMagneticEncoder.Eeprom.SetPage < MGTMagneticEncoderPageNumber){
    MgtMagneticEncoderWriteOnePage();
  }else{
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }
}
//==============================================================================
// MgtMagneticEncoderInitialize
//==============================================================================
void MgtMagneticEncoderInitialize(void)
{
  if(MgtMagneticEncoder.Cnt.CFDA < 10){
    MgtMagneticEncoderTX(0xDA, 0x00, 0x00); 
    MgtMagneticEncoder.Cnt.CFDA ++;
  }else{
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }
}
//==============================================================================
// MgtMagneticEncoderSpeed
//==============================================================================
void MgtMagneticEncoderSpeed(void)
{
  MgtMagneticEncoderTX(0x1A, 0x00, 0x00);
  if(MgtMagneticEncoder.Cnt.CF1A < 100){
    MgtMagneticEncoder.Cnt.CF1A ++;
  }else{
    Motor.Flag.bit.ReadyTest = 1;
  }
}
//==============================================================================
// MgtMagneticEncoderOpenInternalProtocol
//==============================================================================
void MgtMagneticEncoderOpenInternalProtocol(void)
{
  if(MgtMagneticEncoder.Cnt.CFOIP < 5){
    MgtMagneticEncoderTX(0x6D, 0x00,0x4F); 
    MgtMagneticEncoder.Cnt.CFOIP ++;
  }else{
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }
}
//==============================================================================
// MgtMagneticEncoderCommand0x32
//==============================================================================
void MgtMagneticEncoderCommand0x32(void)
{
  if(MgtMagneticEncoder.Eeprom.Status.bit.Busy == 0){
    MgtMagneticEncoderTX(0x32, MgtMagneticEncoder.Eeprom.SetAddress, MgtMagneticEncoder.Eeprom.Data); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }else{
    MgtMagneticEncoderTX(0xEA, MgtMagneticEncoder.Eeprom.SetAddress, 0x00); 
  }
}
//==============================================================================
// MgtMagneticEncoderCommand0x7A
//==============================================================================
void MgtMagneticEncoderCommand0x7A(void)
{
  if(MgtMagneticEncoder.Eeprom.Status.bit.Busy == 0){
    MgtMagneticEncoderTX(0x7A, MgtMagneticEncoder.Eeprom.Address, MgtMagneticEncoder.Eeprom.Data); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }else{
    MgtMagneticEncoderTX(0xA2, MgtMagneticEncoder.Eeprom.Address, 0x00); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }
}
//==============================================================================
// MgtMagneticEncoderWriteDate
//==============================================================================
void MgtMagneticEncoderWriteDate(void)
{
  if(MgtMagneticEncoder.Eeprom.Status.bit.Busy == 0){
    MgtMagneticEncoderTX(0x7A, (0x0304 + MotorEncoder.TestDateCnt), MotorEncoder.TestDate); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }else{
    MgtMagneticEncoderTX(0xA2, (0x0304 + MotorEncoder.TestDateCnt), 0x00); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }
}
//==============================================================================
// MgtMagneticEncoderSetResolution
//==============================================================================
void MgtMagneticEncoderSetResolution(void)
{
  MgtMagneticEncoderTX(0x40, 0x46,MotorEncoder.SetResolutionID);
  MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
}
//==============================================================================
// MgtMagneticEncoderReadResolution
//==============================================================================
void MgtMagneticEncoderReadResolution(void)
{
  MgtMagneticEncoderTX(0x40, 0x45,0x00);
  MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
}
//==============================================================================
// MgtMagneticEncoderWriteHWRev
//==============================================================================
void MgtMagneticEncoderWriteHWRev(void)
{
  if(MgtMagneticEncoder.Eeprom.Status.bit.Write == 1){
    switch(MotorEncoder.HWRevAddr){
    case 0x030D:
      MgtMagneticEncoderTX(0x7A, MotorEncoder.HWRevAddr, 'H');
      MgtMagneticEncoder.Eeprom.Status.bit.Write = 0;      
      break;
    case 0x030E:
      MgtMagneticEncoderTX(0x7A, MotorEncoder.HWRevAddr, 'M');
      MgtMagneticEncoder.Eeprom.Status.bit.Write = 0;       
      break;
    case 0x030F:
      MgtMagneticEncoderTX(0x7A, MotorEncoder.HWRevAddr, 1);
      MgtMagneticEncoder.Eeprom.Status.bit.Write = 0;       
      break;
    case 0x0310:
      MgtMagneticEncoderTX(0x7A, MotorEncoder.HWRevAddr, MotorEncoder.HWRevData);
      MgtMagneticEncoder.Eeprom.Status.bit.Write = 0;       
      break;  
    default:
      
      break;
    }
  }else{
    if(MgtMagneticEncoder.Eeprom.Status.bit.Read == 1){
      if(MgtMagneticEncoder.Eeprom.Status.bit.Busy == 0){
        if(MotorEncoder.HWRevAddr == 0x0310){
          MgtMagneticEncoder.Eeprom.Status.bit.Read = 0;
          MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
        }else{
          MgtMagneticEncoder.Eeprom.Status.bit.Write = 1;
          MgtMagneticEncoder.Eeprom.Status.bit.Read = 0;
          MotorEncoder.HWRevAddr ++;          
        }
      }else{
        MgtMagneticEncoderTX(0xA2, MotorEncoder.HWRevAddr, 0x00); 
      }
    }else{
      MgtMagneticEncoderTX(0xA2, MotorEncoder.HWRevAddr, 0x00); //发送0x7A后，此时busy还是0
      MgtMagneticEncoder.Eeprom.Status.bit.Read = 1;      
    }
  }
}

//==============================================================================
// MgtMagneticEncoderReadHWRev
//==============================================================================
void MgtMagneticEncoderReadHWRev(void)
{
  if(MotorEncoder.HWRevAddr == 0x030E){
    MotorEncoder.HWRev[0] = MgtMagneticEncoder.Eeprom.Data;
    MotorEncoder.HWRev[1] = '.';
  }else if(MotorEncoder.HWRevAddr == 0x030F){
    MotorEncoder.HWRev[2] = MgtMagneticEncoder.Eeprom.Data;
    MotorEncoder.HWRev[3] = '.';    
  }else if(MotorEncoder.HWRevAddr == 0x0310){
    MotorEncoder.HWRev[4] = MgtMagneticEncoder.Eeprom.Data + 0x30;
    MotorEncoder.HWRev[5] = '.';    
  }else if(MotorEncoder.HWRevAddr == 0x0311){
    MotorEncoder.HWRev[6] = MgtMagneticEncoder.Eeprom.Data + 0x30;    
    MotorEncoder.HWRev[7] = '\0';
    hexToAsciiString(MotorEncoder.HWRev, MotorEncoder.HWRevBuffer, 7);
  }
  
  if(MotorEncoder.HWRevAddr < 0x0311){
    MgtMagneticEncoderTX(0xA2, MotorEncoder.HWRevAddr, 0x00); 
    MotorEncoder.HWRevAddr ++;
  }else{
    MotorEncoder.HWRevAddr = 0;
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }
}
//==============================================================================
// MgtMagneticEncoderReadHWRev
//==============================================================================
void MgtMagneticEncoderTimeOut(void)
{
  uint16_t i = 0;
  if(i < 1000){
    i ++;
    MgtMagneticEncoderTX(0x02, 0x00, 0x00);
  }else{
    MgtMagneticEncoder.TimeOutFlag = 1;
    i = 0;
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
  }
}
//==============================================================================
// MgtMagneticEncoderTest
//==============================================================================
__ramfunc void MgtMagneticEncoderTest(volatile uint8_t TestItems)
{
  switch(TestItems){
  case MgtMagneticEncoderStopTest:
    
    break;
  case MgtMagneticEncoderMultiturnTest:
    //MgtMagneticEncoderMultiturnReset();
    break;
  case MgtMagneticEncoderGetAllDataTest:
    MgtMagneticEncoderTX(0x1A, 0x00, 0x00);
    break;
  case MgtMagneticEncoderReadAllEepromTest:
    MgtMagneticEncoderReadAllEeprom();
    break;
  case MgtMagneticEncoderInitializeTest:
    MgtMagneticEncoderInitialize();
    break;    
  case MgtMagneticEncoderSpeedTest:
    MgtMagneticEncoderSpeed();
    break;
  case MgtMagneticEncoderFirmwareVersionTest:
    MgtMagneticEncoderTX(0xA2, 0x0305, 0x00);
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
    break;
  case MgtMagneticEncoderWriteAllEepromTest:
    MgtMagneticEncoderWriteAllEeprom();
    break;
  case MgtMagneticEncoderCommand0x32Test:
    MgtMagneticEncoderCommand0x32();
    break;
  case MgtMagneticEncoderCommand0xEATest:
    MgtMagneticEncoderTX(0xEA, MgtMagneticEncoder.Eeprom.SetAddress, 0x00); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;
    break;
  case MgtMagneticEncoderOIPTest:
    MgtMagneticEncoderOpenInternalProtocol();    
    break;
  case MgtMagneticEncoderCIPTest:
    MgtMagneticEncoderTX(0x6D, 0x00,0x43); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;      
    break;
  case MgtMagneticEncoderCommand0x7ATest:
    MgtMagneticEncoderCommand0x7A();
    break;
  case MgtMagneticEncoderCommand0xA2Test:
    MgtMagneticEncoderTX(0xA2, MgtMagneticEncoder.Eeprom.Address,0x00); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;        
    break;
  case MgtMagneticEncoderFirmwareTest:
    MgtMagneticEncoderTX(0x40, 0x10,0x00); 
    MotorEncoder.TestItem = MgtMagneticEncoderStopTest;     
    break;
  case MgtMagneticEncoderWriteDateTest:
    MgtMagneticEncoderWriteDate();
    break;
  case MgtMagneticEncoderSetResolutionTest:
    MgtMagneticEncoderSetResolution();
    break;
  case MgtMagneticEncoderReadResolutionTest:
    MgtMagneticEncoderReadResolution();
    break;
  case MgtMagneticEncoderWriteHWRevTest:
    MgtMagneticEncoderWriteHWRev();
    break;   
  case MgtMagneticEncoderReadHWRevTest:
    MgtMagneticEncoderReadHWRev();
    break;      
  case MgtMagneticEncoderTimeoutTest:
    MgtMagneticEncoderTimeOut();
  }
}