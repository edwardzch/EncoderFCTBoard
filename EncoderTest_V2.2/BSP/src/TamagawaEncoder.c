#include "gpio_config.h"
#include "pit_config.h"
#include "uart_config.h"
#include "TamagawaEncoder.h"
#include "Function.h"
#include "ModBus.h"
//==============================================================================
// DEFINES
//==============================================================================
strMTP  MTP = {0};
strTmgwEnc  TamagawaEncoder = {0};
volatile uint8_t   MultiturnResetCnt = 0; 
extern volatile uint8_t  Work_Alarm;
extern uart_transfer_t uart0_transfer;
//==============================================================================
// TmagawaEncoderRX
//==============================================================================
void TamagawaEncoderRX(void)
{
    //static uint8_t   i = 0;
    TamagawaEncoder.RxData[TamagawaEncoder.RxDataCnt++] = UART0->D;                                                  //Recieve request to auto clear flag     
    
    if(TamagawaEncoder.RxDataCnt >= 1){
      switch(TamagawaEncoder.RxData[0]){
      case 0x1A:
        if(TamagawaEncoder.RxDataCnt == 11){
          UART_EnableRx(UART0, false);
          TamagawaEncoder.TimeoutCnt = 0;
          TamagawaEncoder.RxDataCnt = 0;
          TamagawaEncoder.RxID = TamagawaEncoder.RxData[0];
          TamagawaEncoder.Status.all = TamagawaEncoder.RxData[1];
          TamagawaEncoder.SingleTurnPosition = ((TamagawaEncoder.RxData[4] << 16) | (TamagawaEncoder.RxData[3] << 8) | TamagawaEncoder.RxData[2]) & 0xFFFFFFFF;
          MotorEncoder.SingleTurnPosition = TamagawaEncoder.SingleTurnPosition;
          TamagawaEncoder.ResolutionID = TamagawaEncoder.RxData[5];
          TamagawaEncoder.Resolution = 1<<TamagawaEncoder.ResolutionID;
          TamagawaEncoder.SinglePoleResolution = TamagawaEncoder.Resolution/5;
          TamagawaEncoder.Phase = TamagawaEncoder.SingleTurnPosition % TamagawaEncoder.SinglePoleResolution;
          TamagawaEncoder.PhaseAngle = 360 - (((float)TamagawaEncoder.Phase / (float)TamagawaEncoder.SinglePoleResolution) * 360);
          MotorEncoder.ResolutionID = TamagawaEncoder.ResolutionID;
          TamagawaEncoder.MultiTurnPosition = ((TamagawaEncoder.RxData[7] << 8) | TamagawaEncoder.RxData[6]) & 0xFFFF;
          MotorEncoder.MultiTurnPosition = TamagawaEncoder.MultiTurnPosition;
          TamagawaEncoder.Error.all = TamagawaEncoder.RxData[9];
          TamagawaEncoder.XorCrcData = CRC8_Check((uint8_t *)TamagawaEncoder.RxData, 10); 
          if(TamagawaEncoder.XorCrcData != TamagawaEncoder.RxData[10]){
            TamagawaEncoder.XorCrcError = 1;
            Work_Alarm = 0x02;
          }
        }
        break;      
      case 0x02:
        if(TamagawaEncoder.RxDataCnt == 6){
          UART_EnableRx(UART0, false);
          TamagawaEncoder.TimeoutCnt = 0;
          TamagawaEncoder.RxDataCnt = 0;
          TamagawaEncoder.RxID = TamagawaEncoder.RxData[0];
          TamagawaEncoder.Status.all = TamagawaEncoder.RxData[1];
          TamagawaEncoder.SingleTurnPosition = ((TamagawaEncoder.RxData[4] << 16) | (TamagawaEncoder.RxData[3] << 8) | TamagawaEncoder.RxData[2]) & 0xFFFFFFFF;
          MotorEncoder.SingleTurnPosition = TamagawaEncoder.SingleTurnPosition;
          TamagawaEncoder.XorCrcData = CRC8_Check((uint8_t *)TamagawaEncoder.RxData, 5);
          if(TamagawaEncoder.XorCrcData != TamagawaEncoder.RxData[5]){
            TamagawaEncoder.XorCrcError = 1;
            Work_Alarm = 0x02;
          }
        }
        break;
      case 0x62:
        if(TamagawaEncoder.RxDataCnt == 6){
          UART_EnableRx(UART0, false);
          TamagawaEncoder.TimeoutCnt = 0;
          TamagawaEncoder.RxDataCnt = 0;
          TamagawaEncoder.RxID = TamagawaEncoder.RxData[0];
          TamagawaEncoder.Status.all = TamagawaEncoder.RxData[1];
          TamagawaEncoder.SingleTurnPosition = ((TamagawaEncoder.RxData[4] << 16) | (TamagawaEncoder.RxData[3] << 8) | TamagawaEncoder.RxData[2]) & 0xFFFFFFFF;
          TamagawaEncoder.XorCrcData = CRC8_Check((uint8_t *)TamagawaEncoder.RxData, 5);
          if(TamagawaEncoder.XorCrcData != TamagawaEncoder.RxData[5]){
            TamagawaEncoder.XorCrcError = 1;
            Work_Alarm = 0x02;
          }
        }
        break; 
      case 0xC2:
        if(TamagawaEncoder.RxDataCnt == 6){
          UART_EnableRx(UART0, false);
          TamagawaEncoder.TimeoutCnt = 0;
          TamagawaEncoder.RxDataCnt = 0;
          TamagawaEncoder.RxID = TamagawaEncoder.RxData[0];
          TamagawaEncoder.Status.all = TamagawaEncoder.RxData[1];
          TamagawaEncoder.SingleTurnPosition = ((TamagawaEncoder.RxData[4] << 16) | (TamagawaEncoder.RxData[3] << 8) | TamagawaEncoder.RxData[2]) & 0xFFFFFFFF;
          TamagawaEncoder.XorCrcData = CRC8_Check((uint8_t *)TamagawaEncoder.RxData, 5);
          if(TamagawaEncoder.XorCrcData != TamagawaEncoder.RxData[5]){
            TamagawaEncoder.XorCrcError = 1;
            Work_Alarm = 0x02;
          }
        }
        break;         
      case 0xEA:
        if(TamagawaEncoder.RxDataCnt==4){
          TamagawaEncoder.TimeoutCnt = 0;
          TamagawaEncoder.RxDataCnt = 0;
          UART_EnableRx(UART0, false);      
          TamagawaEncoder.RxID = TamagawaEncoder.RxData[0];
          TamagawaEncoder.Eeprom.ReturnAddress = TamagawaEncoder.RxData[1] & 0x7F;
          if(TamagawaEncoder.Eeprom.ReturnAddress == TamagawaEncoderPNSAddress){
            TamagawaEncoder.Eeprom.ReturnPage = TamagawaEncoder.RxData[2];
            TamagawaEncoder.Eeprom.Status.bit.Busy = (TamagawaEncoder.RxData[1] >> 7) & 0xFF;
            TamagawaEncoder.Eeprom.DataBuffer[TamagawaEncoder.Eeprom.ReturnPage][TamagawaEncoder.Eeprom.ReturnAddress] = TamagawaEncoder.RxData[2];   
          }else if(TamagawaEncoder.Eeprom.ReturnAddress < TamagawaEncoderPNSAddress){
            TamagawaEncoder.Eeprom.Status.bit.Busy = (TamagawaEncoder.RxData[1] >> 7) & 0xFF;            
            if(TamagawaEncoder.Eeprom.Status.bit.Busy == 0){
              TamagawaEncoder.Eeprom.SetAddress ++;
              TamagawaEncoder.Eeprom.DataBuffer[TamagawaEncoder.Eeprom.ReturnPage][TamagawaEncoder.Eeprom.ReturnAddress] = TamagawaEncoder.RxData[2];            
            }
          }        
          TamagawaEncoder.XorCrcData = CRC8_Check((uint8_t *)TamagawaEncoder.RxData, 3);
          if(TamagawaEncoder.XorCrcData != TamagawaEncoder.RxData[3]){
            TamagawaEncoder.XorCrcError = 1;
            Work_Alarm = 0x02;
          }        
        }  
        break;
      case 0x32:
        if(TamagawaEncoder.RxDataCnt==4){
          TamagawaEncoder.TimeoutCnt = 0;
          TamagawaEncoder.RxDataCnt = 0;
          UART_EnableRx(UART0, false);
          TamagawaEncoder.RxID = TamagawaEncoder.RxData[0];
          TamagawaEncoder.Eeprom.Address = TamagawaEncoder.RxData[1];
          TamagawaEncoder.Data = TamagawaEncoder.RxData[2];
          TamagawaEncoder.XorCrcData = CRC8_Check((uint8_t *)TamagawaEncoder.RxData, 3);
          if(TamagawaEncoder.XorCrcData != TamagawaEncoder.RxData[3]){
            TamagawaEncoder.XorCrcError = 1;
            Work_Alarm = 0x02;
          }
          TamagawaEncoder.Eeprom.ReturnAddress = TamagawaEncoder.RxData[1] & 0x7F;
          if(TamagawaEncoder.Eeprom.ReturnAddress == TamagawaEncoderPNSAddress){
            TamagawaEncoder.Eeprom.ReturnPage = TamagawaEncoder.RxData[2];
            TamagawaEncoder.Eeprom.Status.bit.Busy = (TamagawaEncoder.RxData[1] >> 7) & 0xFF;
            TamagawaEncoder.Eeprom.DataBuffer[TamagawaEncoder.Eeprom.ReturnPage][TamagawaEncoder.Eeprom.ReturnAddress] = TamagawaEncoder.RxData[2];   
          }          
          TamagawaEncoder.Flag.bit.CF32 = 0x00;
        }          
        break;
      default:
          TamagawaEncoder.RxDataCnt = 0;
          //TamagawaEncoder.TmgwTimeoutCnt = 0;
          break;              
      }
    }     
}
//==============================================================================
// TmagawaEncoderTX
//==============================================================================
void TmagawaEncoderTX(volatile uint8_t Idle, uint16_t Addr, uint8_t Data)
{
  switch(Idle){
  case 0x62:
    TamagawaEncoder.TimeoutCnt ++;
    TamagawaEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&TamagawaEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);    
    break;
  case 0x1A:
    TamagawaEncoder.TimeoutCnt ++;
    TamagawaEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&TamagawaEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;
  case 0xC2:
    TamagawaEncoder.TimeoutCnt ++;
    TamagawaEncoder.TxData[0] = Idle;                                                //CF                 
    uart0_transfer.data = (uint8_t *)&TamagawaEncoder.TxData[0];
    uart0_transfer.dataSize = 1;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;    
  case 0xEA:
    TamagawaEncoder.TimeoutCnt ++;
    TamagawaEncoder.TxData[0] = Idle;                                                //CF                 
    TamagawaEncoder.TxData[1] = Addr;
    TamagawaEncoder.TxData[2] = CRC8_Check((uint8_t *)TamagawaEncoder.TxData, 2); ;
    uart0_transfer.data = (uint8_t *)&TamagawaEncoder.TxData[0];
    uart0_transfer.dataSize = 3;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break; 
  case 0x32:
    TamagawaEncoder.TimeoutCnt ++;
    TamagawaEncoder.TxData[0] = Idle;                                                //CF                 
    TamagawaEncoder.TxData[1] = Addr;
    TamagawaEncoder.TxData[2] = Data;
    TamagawaEncoder.TxData[3] = CRC8_Check((uint8_t *)TamagawaEncoder.TxData, 3); 
    uart0_transfer.data = (uint8_t *)&TamagawaEncoder.TxData[0];
    uart0_transfer.dataSize = 4;           
    Encoder485DE();                                                          //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART_SendDMA0(&uart0_transfer);        
    break;      
  }
    if(TamagawaEncoder.TimeoutCnt > 9){
      ModBus.Error.bit.DC = 1;
      Work_Alarm = 0x01;
    }     
}

//==============================================================================
// TmagawaEncoderMultiturnReset
//==============================================================================
void TmagawaEncoderMultiturnReset(void)
{
  if(TamagawaEncoder.Cnt.CF62 < 10){
    TmagawaEncoderTX(0x62, 0x00, 0x00);
    TamagawaEncoder.Cnt.CF62 ++;
    if(TamagawaEncoder.Cnt.CF62 == 10){
      MotorEncoder.TestItem = TmagawaEncoderGetAllDataTest;
    }
  }
}
//==============================================================================
// TmagawaEncoderSingleTurnReset
//==============================================================================
void TmagawaEncoderSingleTurnReset(void)
{
  if(TamagawaEncoder.Cnt.CFC2 < 10){
    TmagawaEncoderTX(0xC2, 0x00, 0x00);
    TamagawaEncoder.Cnt.CFC2 ++;
    if(TamagawaEncoder.Cnt.CFC2 == 10){
      MotorEncoder.TestItem = TmagawaEncoderGetAllDataTest;
    }
  }
}
//==============================================================================
// DornaEncoderReadMTPDefine
//==============================================================================
void DornaEncoderReadMTPDefine(void)
{
  static uint8_t i;
  
  
  MTP.CrcDataLow = TamagawaEncoder.Eeprom.DataBuffer[0][32];
  MTP.CrcDataHigh = TamagawaEncoder.Eeprom.DataBuffer[0][33];
  MTP.CrcData = MTP.CrcDataLow | (MTP.CrcDataHigh << 8);
  for(i=32; i < 111; i++){
    MTP.CrcDataCalculate += TamagawaEncoder.Eeprom.DataBuffer[0][i];
  }
  MTP.CrcDataCalculate += 0x5AA5;
  
  MTP.EPSDriverType = TamagawaEncoder.Eeprom.DataBuffer[0][0];
  MTP.EPSMotorType = TamagawaEncoder.Eeprom.DataBuffer[0][1];
  MTP.EPSEncoderType = TamagawaEncoder.Eeprom.DataBuffer[0][2];
  MTP.MotorType = TamagawaEncoder.Eeprom.DataBuffer[0][38];
  MTP.EncoderType = TamagawaEncoder.Eeprom.DataBuffer[0][39]& 0x0F;
  MTP.VoltageType = TamagawaEncoder.Eeprom.DataBuffer[0][39]& 0xF0;
  MTP.MotorPower = TamagawaEncoder.Eeprom.DataBuffer[0][42] | (TamagawaEncoder.Eeprom.DataBuffer[0][43] << 8);
  MTP.Reserve.No8 = TamagawaEncoder.Eeprom.DataBuffer[0][44] | (TamagawaEncoder.Eeprom.DataBuffer[0][45] << 8);
  MTP.MultiTurnPosition = TamagawaEncoder.Eeprom.DataBuffer[0][46] | (TamagawaEncoder.Eeprom.DataBuffer[0][47] << 8);
  MTP.RatedSpeed = TamagawaEncoder.Eeprom.DataBuffer[0][48] * 100;
  MTP.MaximumSpeed = TamagawaEncoder.Eeprom.DataBuffer[0][49] * 100;
  MTP.NumberOfPolePairs = TamagawaEncoder.Eeprom.DataBuffer[0][50];
  MTP.OverSpeedLevel = TamagawaEncoder.Eeprom.DataBuffer[0][51];
  MTP.RatedTorque = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][52] | (TamagawaEncoder.Eeprom.DataBuffer[0][53] << 8))/100);
  MTP.MaximumTorque = TamagawaEncoder.Eeprom.DataBuffer[0][54] | (TamagawaEncoder.Eeprom.DataBuffer[0][55] << 8);
  MTP.RatedPeakCurrent = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][56] | (TamagawaEncoder.Eeprom.DataBuffer[0][57] << 8))/10);
  MTP.InstantaneousMaximumPeakCurrent = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][58] | (TamagawaEncoder.Eeprom.DataBuffer[0][59] << 8)/100));
  MTP.EMFConstant = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][60] | (TamagawaEncoder.Eeprom.DataBuffer[0][61] << 8))/10);
  MTP.RotorInertia = TamagawaEncoder.Eeprom.DataBuffer[0][62] | (TamagawaEncoder.Eeprom.DataBuffer[0][63] << 8);
  MTP.ArmatureWindingResistance = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][64] | (TamagawaEncoder.Eeprom.DataBuffer[0][65] << 8))/1000);
  MTP.ArmatureInductance = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][66] | (TamagawaEncoder.Eeprom.DataBuffer[0][67] << 8))/100);
  MTP.OverloadDetectionBaseBurrent = TamagawaEncoder.Eeprom.DataBuffer[0][68] | (TamagawaEncoder.Eeprom.DataBuffer[0][69] << 8);
  MTP.OverloadDetectionIntermediateCurrent_1 = TamagawaEncoder.Eeprom.DataBuffer[0][70] | (TamagawaEncoder.Eeprom.DataBuffer[0][71] << 8);
  MTP.OverloadDetectionIntermediateTime_1 = TamagawaEncoder.Eeprom.DataBuffer[0][72] | (TamagawaEncoder.Eeprom.DataBuffer[0][73] << 8);
  MTP.OverloadDetectionIntermediateCurrent_2 = TamagawaEncoder.Eeprom.DataBuffer[0][74] | (TamagawaEncoder.Eeprom.DataBuffer[0][75] << 8);
  MTP.OverloadDetectionIntermediateTime_2 = TamagawaEncoder.Eeprom.DataBuffer[0][76] | (TamagawaEncoder.Eeprom.DataBuffer[0][77] << 8);
  MTP.Reserve.No27 = TamagawaEncoder.Eeprom.DataBuffer[0][78] | (TamagawaEncoder.Eeprom.DataBuffer[0][79] << 8);
  MTP.Reserve.No28 = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][80] | (TamagawaEncoder.Eeprom.DataBuffer[0][81] << 8)));
  MTP.Lq0 = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][82] | (TamagawaEncoder.Eeprom.DataBuffer[0][83] << 8))/100);
  MTP.Lq1 = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][84] | (TamagawaEncoder.Eeprom.DataBuffer[0][85] << 8))/100);
  MTP.RatedTorqueIndex = TamagawaEncoder.Eeprom.DataBuffer[0][86];
  MTP.MomentOfInertiaIndex = TamagawaEncoder.Eeprom.DataBuffer[0][87];
  MTP.RatedOutputIndex = TamagawaEncoder.Eeprom.DataBuffer[0][88];
  MTP.RotationSpeedIndex = TamagawaEncoder.Eeprom.DataBuffer[0][89];
  MTP.Reserve.No35 = TamagawaEncoder.Eeprom.DataBuffer[0][90] | (TamagawaEncoder.Eeprom.DataBuffer[0][91] << 8);
  MTP.Reserve.No36 = TamagawaEncoder.Eeprom.DataBuffer[0][92] | (TamagawaEncoder.Eeprom.DataBuffer[0][93] << 8); 
  MTP.Reserve.No37 = TamagawaEncoder.Eeprom.DataBuffer[0][94] | (TamagawaEncoder.Eeprom.DataBuffer[0][95] << 8);
  MTP.Reserve.No38 = TamagawaEncoder.Eeprom.DataBuffer[0][96];
  MTP.Reserve.No39 = TamagawaEncoder.Eeprom.DataBuffer[0][97];
  MTP.PhaseLow = TamagawaEncoder.Eeprom.DataBuffer[0][98] | (TamagawaEncoder.Eeprom.DataBuffer[0][99] << 8);
  MTP.PhaseHigh = TamagawaEncoder.Eeprom.DataBuffer[0][100] | (TamagawaEncoder.Eeprom.DataBuffer[0][101] << 8);
  MTP.Reserve.No42 = TamagawaEncoder.Eeprom.DataBuffer[0][102] | (TamagawaEncoder.Eeprom.DataBuffer[0][103] << 8);
  MTP.Reserve.No43 = TamagawaEncoder.Eeprom.DataBuffer[0][104] | (TamagawaEncoder.Eeprom.DataBuffer[0][105] << 8);
  MTP.Reserve.No44 = TamagawaEncoder.Eeprom.DataBuffer[0][106] | (TamagawaEncoder.Eeprom.DataBuffer[0][107] << 8);
  MTP.Reserve.No45 = TamagawaEncoder.Eeprom.DataBuffer[0][108] | (TamagawaEncoder.Eeprom.DataBuffer[0][109] << 8);
  MTP.Reserve.No46 = TamagawaEncoder.Eeprom.DataBuffer[0][110] | (TamagawaEncoder.Eeprom.DataBuffer[0][111] << 8);  
}
//==============================================================================
// TuosidaEncoderReadMTPDefine
//==============================================================================
void TuosidaEncoderReadMTPDefine(void)
{
  //static uint8_t i;
  
  
  MTP.CrcData = TamagawaEncoder.Eeprom.DataBuffer[0][0] | (TamagawaEncoder.Eeprom.DataBuffer[0][1] << 8);
  
  MTP.CrcDataLow = (uint8_t)((RTU_CRC((uint8_t *)&TamagawaEncoder.Eeprom.DataBuffer[0][2], 112)) & 0xFF);
  MTP.CrcDataHigh = (uint8_t)((RTU_CRC((uint8_t *)&TamagawaEncoder.Eeprom.DataBuffer[0][2], 112)) >> 8) & 0xFF;  
  
  
  MTP.VoltageType = TamagawaEncoder.Eeprom.DataBuffer[0][6];
  MTP.RatedCurrent = (float)((float)TamagawaEncoder.Eeprom.DataBuffer[0][10]/100);
  MTP.OverloadCapacity = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][14] | (TamagawaEncoder.Eeprom.DataBuffer[0][15] << 8))/100);
  MTP.RatedTorque = (float)((float)TamagawaEncoder.Eeprom.DataBuffer[0][18]/100);
  MTP.RatedSpeed = TamagawaEncoder.Eeprom.DataBuffer[0][22] | (TamagawaEncoder.Eeprom.DataBuffer[0][23] << 8);
  MTP.MaximumSpeed = TamagawaEncoder.Eeprom.DataBuffer[0][26] | (TamagawaEncoder.Eeprom.DataBuffer[0][27] << 8);
  MTP.RotorInertiaFloat = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][30] | (TamagawaEncoder.Eeprom.DataBuffer[0][31] << 8))/10000000);
  MTP.OppositePotentialConstant = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][34] | (TamagawaEncoder.Eeprom.DataBuffer[0][35] << 8))/100);
  MTP.NumberOfPolePairs = TamagawaEncoder.Eeprom.DataBuffer[0][38];
  MTP.MagneticFluxAlarmLimit = (float)((float)TamagawaEncoder.Eeprom.DataBuffer[0][42]/100);
  MTP.TorqueAlarmLimit = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][46] | (TamagawaEncoder.Eeprom.DataBuffer[0][47] << 8))/100);
  MTP.TorqueOverloadThreshold = (float)((float)TamagawaEncoder.Eeprom.DataBuffer[0][50]/100);
  MTP.TwotimesOverloadTime = TamagawaEncoder.Eeprom.DataBuffer[0][54] | (TamagawaEncoder.Eeprom.DataBuffer[0][55] << 8);
  MTP.ElectricalAngle = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][74] | (TamagawaEncoder.Eeprom.DataBuffer[0][75] << 8))/100);
  MTP.CurrenLoopGain = (float)((float)(TamagawaEncoder.Eeprom.DataBuffer[0][98] | (TamagawaEncoder.Eeprom.DataBuffer[0][99] << 8))/10);
  MTP.CurrentLoopIntegrationTime = (float)((float)TamagawaEncoder.Eeprom.DataBuffer[0][102]/10);  
    
}
//==============================================================================
// TmagawaEncoderReadMTPDefine
//==============================================================================
void TmagawaEncoderReadMTPDefine(void)
{
  if(Motor.MTPType == TuosidaMotor){
    TuosidaEncoderReadMTPDefine();
  }else{
    DornaEncoderReadMTPDefine();
  }
  MotorEncoder.TestItem = TmagawaEncoderGetAllDataTest;
}
//==============================================================================
// TmagawaEncoderReadMTP
//==============================================================================
void TmagawaEncoderReadMTP(void)
{
//  if(TamagawaEncoder.Cnt.CFEA < 128){
//    TmagawaEncoderTX(0xEA, TamagawaEncoder.Cnt.CFEA, 0x00);
//    TamagawaEncoder.Cnt.CFEA ++;
//  }else{
//    MotorEncoder.TestItem = TmagawaEncoderReadMTPDefineTest;
//  }
  if(TamagawaEncoder.Eeprom.SetPage < 6){
    if(TamagawaEncoder.Eeprom.SetAddress < 127){
      TmagawaEncoderTX(0xEA, TamagawaEncoder.Eeprom.SetAddress, 0x00);      
    }else{
      TamagawaEncoder.Eeprom.SetPage ++;
      TmagawaEncoderTX(0x32, TamagawaEncoder.Eeprom.SetAddress, TamagawaEncoder.Eeprom.SetPage);
      TamagawaEncoder.Eeprom.SetAddress = 0;
    }
  }else{
    TamagawaEncoder.Eeprom.SetPage = 0;
    TamagawaEncoder.Eeprom.ReturnPage = 0;
    TamagawaEncoder.Eeprom.SetAddress = 0;
    TamagawaEncoder.Eeprom.ReturnAddress = 0;
    MotorEncoder.TestItem = TmagawaEncoderReadMTPDefineTest;
  }
//  if(TamagawaEncoder.Cnt.CFEA < 128){
//    TmagawaEncoderTX(0xEA, TamagawaEncoder.Cnt.CFEA, 0x00);
//    TamagawaEncoder.Cnt.CFEA ++;
//  }else{
//    MotorEncoder.TestItem = TmagawaEncoderReadMTPDefineTest;
//  }  
}
//==============================================================================
// TamagawaEncoderTest
//==============================================================================
__ramfunc void TamagawaEncoderTest(volatile uint8_t TestItems)
{
  switch(TestItems){
  case TmagawaEncoderStopTest:
    
    break;
  case TmagawaEncoderMultiturnTest:
    TmagawaEncoderMultiturnReset();
    break;
  case TmagawaEncoderGetAllDataTest:
    TmagawaEncoderTX(0x1A, 0x00, 0x00);
    break;
  case TmagawaEncoderReadMTPTest:
    TmagawaEncoderReadMTP();
    break;
  case TmagawaEncoderSingleTurnTest:
    TmagawaEncoderSingleTurnReset();
    break;
  case TmagawaEncoderReadMTPDefineTest:
    TmagawaEncoderReadMTPDefine();
    break;    
  }
}