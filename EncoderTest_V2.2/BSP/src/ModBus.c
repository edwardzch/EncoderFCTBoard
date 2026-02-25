#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "pit_config.h"
#include "gpio_config.h"
#include "uart_config.h"
#include "Function.h"
#include "TamagawaEncoder.h"
#include "NXPMagneticEncoder.h"
#include "MGTMagneticEncoder.h"
#include "MultiturnOpticalEncoder.h"
#include "MultiturnMagneticEncoder.h"
#include "SensAREncoder.h"
#include "ModBus.h"
#include "iap_function.h"
//==============================================================================
// DEFINES
//==============================================================================				                         

//==============================================================================
// DATA
//==============================================================================
uart_transfer_t uart1_transfer;
void ModBus_Rx03DataCollation(void);

strMod      ModBus = {0};
extern volatile uint8_t  Work_Alarm;
extern volatile uint8_t  Variable_ReadReset_Flag;
extern volatile uint8_t  Variable_Reset_Flag;
//==============================================================================
// UART1_SendDMA0
//==============================================================================
__ramfunc void UART1_SendDMA0(uart_transfer_t * p)
{
    DMA0->TCD[1].SADDR = (uint32_t)p->data;                                     //DMA source addr
    DMA0->TCD[1].DADDR = (uint32_t)&UART1->D;                                   //DMA dest   addr
    DMA0->TCD[1].CITER_ELINKNO = p->dataSize;
    DMA0->TCD[1].BITER_ELINKNO = p->dataSize;
    EDMA_EnableChannelRequest(DMA0, 1U);//DMA0->ERQ |= DMA_ERQ_ERQ1_MASK;
}
  

//==============================================================================
// Calculate RTU_CRC function
//==============================================================================
__ramfunc uint16_t RTU_CRC(volatile uint8_t * Data, uint8_t Len)
{  
  uint16_t uCRC = 0xFFFF;  
  uint8_t i = 0;
	
  while(Len--){  
	uCRC ^= *Data++;  
        for(i=0;i<8;i++){
            if(uCRC & 0x01){
                  uCRC = (uCRC >> 1) ^ 0xA001;
            }else{
                  uCRC = (uCRC >> 1);
            }
        }
  }  

  return uCRC;  
}  

// 将十六进制数组转换为以null结尾的字符串
void hexToAsciiString(uint8_t* hexArray, char* output, int length) {
    for (int i = 0; i < length; i++) {
        output[i] = (char)hexArray[i];  // 直接转换
    }
    output[length] = '\0';  // 添加字符串结束符
}
//==============================================================================
// myprintf function
//==============================================================================
void myprintf(const char *format,...)
{
    uint32_t length;
    static char SendBuff[200]; // = {0} 仅在第一次有效，用 memset 更好
    va_list args;

    // 【修复二】在每次使用前清空静态缓冲区，避免残留数据
    memset(SendBuff, 0, sizeof(SendBuff));

    va_start(args, format);
    // 【修复三】使用正确的缓冲区大小
    length = vsnprintf(SendBuff, sizeof(SendBuff), format, args);
    va_end(args);

    // vsnprintf 在缓冲区不足时，返回的是“本应写入的长度”。
    // 我们需要确保DMA发送的长度不超过缓冲区大小。
    if (length >= sizeof(SendBuff)) {
        length = sizeof(SendBuff) - 1; // 发送已截断的部分
    }

    if (length > 0) // 确保有内容才发送
    {
        SerialPort485DE();
        DMA0->TCD[1].SADDR = (uint32_t)SendBuff;
        DMA0->TCD[1].DADDR = (uint32_t)&UART1->D;
        DMA0->TCD[1].CITER_ELINKNO = length;
        DMA0->TCD[1].BITER_ELINKNO = length;
        EDMA_EnableChannelRequest(DMA0, 1U);
        for (volatile uint16_t i=0; i<30000; i++); // 这个延时也值得商榷，最好用DMA完成中断来处理
    }
}
/**************************************************************************************
* 函数名称：GetVERMrging()
* 函数功能：配置字符串合并
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void EncoderMassageCollect(void)
{
    uint8_t i;
    for(i = 0; i < 15; i++){
      SensAR.SetEncoderPN[i] = ModBus.RX.Buffer[i+17];
    }
    for(i = 0; i < 2; i++){
      SensAR.SetEncoderRev[i] = ModBus.RX.Buffer[i+45];
    }	
    for(i = 0; i < 11; i++){
      SensAR.SetEncoderSN[i] = ModBus.RX.Buffer[i+33];
    }	
}
/**************************************************************************************
* 函数名称：ModBusProtocol_ASCII_Rx()
* 函数功能：配置字符串接收
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void ModBusProtocol_ASCII_Rx(void)
{
  ModBus.RX.Buffer[ModBus.RX.DataCnt] = '\0';
  if(strcmp(ModBus.RX.Buffer, "GetPrdInfoData") == 0){
    //myprintf("getprdinfo<32>\r\nPRDr99S2A20z-01<FA>\r\n00<60>\r\nW1224-11770<4D>\r\nSSTA-00<C8>\r\n04<64>\r\nF2522_3914<41>\r\n-->");
    myprintf("%s", SensAR.GetPrdInfoData);
  }else if(strcmp(ModBus.RX.Buffer, "GetVERData") == 0){
    myprintf("%s", SensAR.GetVER);
    //myprintf("ver<4D>\r\nBoot version: 0.3S<F8>\r\nServoSense Production FW: 1.0.16u<04>\r\n-->");
  }else if(strcmp(ModBus.RX.Buffer, "EncoderPowerOff") == 0){
    EncoderPowerOff();
    Variable_Reset_Flag = 1;
  }else if(strncmp(ModBus.RX.Buffer, "SN:DL", 5) == 0){
    if(ModBus.RX.DataCnt == 19){
      MotorEncoder.HWRevData = ModBus.RX.Buffer[5] - 0x30;
    }else{
      ModBus.Error.bit.SN = 1;
    }    
  }else if(strcmp(ModBus.RX.Buffer, "GetHWRev\r\n") == 0){
//    MotorEncoder.HWRev[0] = 'H';
//    MotorEncoder.HWRev[1] = '.';
//    MotorEncoder.HWRev[2] = 'M';
//    MotorEncoder.HWRev[3] = '.';
//    MotorEncoder.HWRev[4] = '1';
//    MotorEncoder.HWRev[5] = '.';
//    MotorEncoder.HWRev[6] = '1';
//    MotorEncoder.HWRev[7] = '\0';
    myprintf("%s", MotorEncoder.HWRevBuffer);
  }else if(strcmp(ModBus.RX.Buffer, "GetFWRev\r\n") == 0){
//    MotorEncoder.FWRev[0] = '2';
//    MotorEncoder.FWRev[1] = '.';
//    MotorEncoder.FWRev[2] = '1';
//    MotorEncoder.FWRev[3] = '.';
//    MotorEncoder.FWRev[4] = '1';
//    MotorEncoder.FWRev[5] = '.';
//    MotorEncoder.FWRev[6] = 'R';
//    MotorEncoder.FWRev[7] = '\0';
    myprintf("%s", MotorEncoder.FWRevBuffer);
  }else if(strcmp(ModBus.RX.Buffer, "Firmware Update\r\n") == 0){
      IAP_RequestUpdate();
  }else if(strcmp(ModBus.RX.Buffer, "Firmware version\r\n") == 0){
      myprintf("KV30_APP_FCT_%0.1f\r\n", FWVersion);
  }else{			
    //EncoderMassageCollect();
    ModBus.Error.bit.SN = 1;//移除对sensar信息收集，改成对sn的判断
  }
  UART_EnableRx(UART1, true); 
}
//==============================================================================
// ModBus_RTU
//==============================================================================
void ModBus_MasterRTU(void)
{
  
}

//==============================================================================
// ModBus_RTU
//==============================================================================
__ramfunc void ModBus_RTU(void)
{
    if(ModBus.RX.Buffer[ModBus.RX.DataCnt-1] == '\n' && ModBus.RX.Buffer[ModBus.RX.DataCnt-2] == '\r'){
      ModBusProtocol_ASCII_Rx();
    }else{
      ModBus.ADR = ModBus.RX.Buffer[0];
      ModBus.CMD = ModBus.RX.Buffer[1]; 
      if(ModBus.RX.DataCnt> 3 && ModBus.ADR == ModBusAddress){
        switch(ModBus.CMD){
        case 0x03:
          if(ModBus.RX.DataCnt == 8){
            //UART1_Len = 0;
            ModBus.TimeoutCnt = 0;         
            ModBus_Rx03();
            //ModBus.FLAG.bit.Busy0x03 = 0x01; 
          }
          break;
        case 0x06:
          if(ModBus.RX.DataCnt == 8){
            //UART1_Len = 0;
            ModBus.TimeoutCnt = 0;          
            ModBus_Rx06();          
            //ModBus.FLAG.bit.Busy0x06 = 0x01;  
          }
          break;
        case 0x10:
          if(ModBus.RX.DataCnt > 5){
            ModBus.RX.DataCountHigh = ModBus.RX.Buffer[4];
            ModBus.RX.DataCountLow = ModBus.RX.Buffer[5];
            ModBus.RX.DataSize = ((ModBus.RX.DataCountHigh << 8) | ModBus.RX.DataCountLow) & 0xFFFF;
            if(ModBus.RX.DataCnt == (ModBus.RX.DataSize + 9)){
              //UART1_Len = 0;
              ModBus.TimeoutCnt = 0;
              UART_EnableRx(UART1, false);   
              //ModBus.FLAG.bit.Busy0x10 = 0x01;            
            }
          }
          break;
        default:
          ModBus.RX.DataCnt = 0;
          ModBus.RX.Buffer[0] = 0;
          ModBus.RX.Buffer[1] = 0;
          break;
        }
      }else if(ModBus.RX.DataCnt> 3 && ModBus.ADR == 0x02){
        switch(ModBus.CMD){
        case 0x03:
          if(ModBus.FLAG.bit.DriverTx == 1){
            ModBus.RX.CRCLow = (uint8_t)((RTU_CRC((uint8_t *)ModBus.RX.Buffer, 7)) & 0xFF);
            ModBus.RX.CRCHigh = (uint8_t)((RTU_CRC((uint8_t *)ModBus.RX.Buffer, 7)) >> 8) & 0xFF; 
            if(ModBus.RX.Buffer[7] == ModBus.RX.CRCLow && ModBus.RX.Buffer[8] == ModBus.RX.CRCHigh){
              Motor.FctPosition = (ModBus.RX.Buffer[5] << 24) | (ModBus.RX.Buffer[6] << 16) | (ModBus.RX.Buffer[3] << 8) | ModBus.RX.Buffer[4];
            }
            ModBus.FLAG.bit.DriverTx = 0;
          }else{
            ModBus_Rx03DataCollation();
            if(ModBus.RX.Buffer[6] == ModBus.RX.CRCLow && ModBus.RX.Buffer[7] == ModBus.RX.CRCHigh){
              if(ModBus.RX.DataAddr == 0xE164){
                ModBus.FLAG.bit.DriverTx = 1;
              }    
            }
          }
          break;
        case 0x06:
          
          break;
        case 0x10:
          
          break;
        default:
          ModBus.RX.DataCnt = 0;
          ModBus.RX.Buffer[0] = 0;
          ModBus.RX.Buffer[1] = 0;        
          break;
        }
      }      
    }
}

void ModBus_FCT(void)
{
  switch(ModBus.CMD){
  case 0x03:
    if(ModBus.FLAG.bit.Busy0x03){
      ModBus.FLAG.bit.Busy0x03 = 0;
      ModBus_Rx03();
    }    
    break;
  case 0x06:
    if(ModBus.FLAG.bit.Busy0x06){
      ModBus.FLAG.bit.Busy0x06 = 0;
      ModBus_Rx06();
    }
    break;
  case 0x10:
    if(ModBus.FLAG.bit.Busy0x10){
      //ModBus_Rx10();
    }
    break;
  }
}
void ModBus_Tx03(volatile uint8_t DriveAddr, uint16_t DataAddr, uint16_t DataLen)
{
    ModBus.TimeoutCnt ++;  
    ModBus.TX.Buffer[0] = DriveAddr;
    ModBus.TX.Buffer[1] = 0x03;  
    ModBus.TX.Buffer[2] = (uint8_t)((DataAddr >> 8) & 0xFF);
    ModBus.TX.Buffer[3] = (uint8_t)(DataAddr & 0xFF);
    ModBus.TX.DataAddr = DataAddr;
    ModBus.TX.Buffer[4] = (uint8_t)((DataLen >> 8) & 0xFF);
    ModBus.TX.Buffer[5] = (uint8_t)(DataLen & 0xFF);
    ModBus.TX.Buffer[6] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 6)) & 0xFF);
    ModBus.TX.Buffer[7] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 6)) >> 8) & 0xFF;  
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 8;  
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer); 
    if(ModBus.TimeoutCnt > 50){
      Work_Alarm = 0x07;
    }       
}
void ModBus_Tx06(volatile uint8_t DriveAddr, uint16_t DataAddr, uint16_t Data)
{
    ModBus.TimeoutCnt ++;
    ModBus.TX.Buffer[0] = DriveAddr;
    ModBus.TX.Buffer[1] = 0x06;  
    ModBus.TX.Buffer[2] = (uint8_t)((DataAddr >> 8) & 0xFF);
    ModBus.TX.Buffer[3] = (uint8_t)(DataAddr & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)((Data >> 8) & 0xFF);
    ModBus.TX.Buffer[5] = (uint8_t)(Data & 0xFF);
    ModBus.TX.Buffer[6] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 6)) & 0xFF);
    ModBus.TX.Buffer[7] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 6)) >> 8) & 0xFF;  
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 8;  
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);
    if(ModBus.TimeoutCnt > 50){
      Work_Alarm = 0x07;
    }       
}
void ModBus_Tx10(volatile uint8_t DriveAddr, uint16_t DataAddr, uint16_t DataLen, uint16_t * DataBuf)
{
    static uint8_t ByteLen = 0;
    uint8_t j =0;
    ModBus.TimeoutCnt ++;    
    ModBus.TX.Buffer[0] = DriveAddr;
    ModBus.TX.Buffer[1] = 0x10;
    ModBus.TX.Buffer[2] = (uint8_t)((DataAddr >> 8) & 0xFF);
    ModBus.TX.Buffer[3] = (uint8_t)(DataAddr & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)((DataLen >> 8) & 0xFF);
    ModBus.TX.Buffer[5] = (uint8_t)(DataLen & 0xFF); 
    ByteLen = DataLen << 1;
    ModBus.TX.Buffer[6] = ByteLen;
    for (volatile uint8_t i=0; i<DataLen; i++){
      ModBus.TX.Buffer[7+j] = (uint8_t)((DataBuf[i] >> 8) &0x00FF);
      ModBus.TX.Buffer[8+j] = (uint8_t)(DataBuf[i] & 0x00FF); 
      j += 2;
    }
    ModBus.TX.Buffer[7+ByteLen] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, (7+ByteLen))) & 0xFF);
    ModBus.TX.Buffer[8+ByteLen] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, (7+ByteLen))) >> 8) & 0xFF; 
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 9+ByteLen;  
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t k=0; k<5; k++);
    UART1_SendDMA0(&uart1_transfer);  
    if(ModBus.TimeoutCnt > 50){
      Work_Alarm = 0x07;
    }       
}
void ModBus_ReturnTx03(volatile uint16_t ReturnData, uint16_t ReturnDataLen)
{
    UART_EnableRx(UART1, false);  
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD;            
    ModBus.TX.Buffer[2] = ReturnDataLen;
    ModBus.TX.Buffer[3] = (uint8_t)((ReturnData >> 8) & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)(ReturnData & 0xFF);
    ModBus.TX.Buffer[ReturnDataLen + 3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, ReturnDataLen + 3)) & 0xFF);
    ModBus.TX.Buffer[ReturnDataLen + 4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, ReturnDataLen + 3)) >> 8) & 0xFF;
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = ReturnDataLen + 5; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);       
}
void ModBus_Return32bitTx03(volatile uint32_t ReturnData, uint16_t ReturnDataLen)
{
    UART_EnableRx(UART1, false);  
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD;            
    ModBus.TX.Buffer[2] = ReturnDataLen;
    ModBus.TX.Buffer[3] = (uint8_t)((ReturnData & 0x0000FF00) >> 8);
    ModBus.TX.Buffer[4] = (uint8_t)(ReturnData & 0x000000FF);   
    ModBus.TX.Buffer[5] = (uint8_t)((ReturnData >> 24) & 0xFF);
    ModBus.TX.Buffer[6] = (uint8_t)((ReturnData & 0x00FF0000) >> 16);    
    ModBus.TX.Buffer[ReturnDataLen + 3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, ReturnDataLen + 3)) & 0xFF);
    ModBus.TX.Buffer[ReturnDataLen + 4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, ReturnDataLen + 3)) >> 8) & 0xFF;
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = ReturnDataLen + 5; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);       
}
void ModBus_ReturnTx06(void)
{
    UART_EnableRx(UART1, false);  
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD;  
    ModBus.TX.Buffer[2] = ModBus.RX.DataAddrHigh;
    ModBus.TX.Buffer[3] = ModBus.RX.DataAddrLow;              
    ModBus.TX.Buffer[4] = ModBus.RX.DataHigh;
    ModBus.TX.Buffer[5] = ModBus.RX.DataLow; 
    ModBus.TX.Buffer[6] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 6)) & 0xFF);
    ModBus.TX.Buffer[7] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 6)) >> 8) & 0xFF;
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 8; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);  
}
void ModBus_Rx03DataCollation(void)
{
    ModBus.RX.DataAddrHigh = ModBus.RX.Buffer[2];
    ModBus.RX.DataAddrLow = ModBus.RX.Buffer[3];      
    ModBus.RX.DataAddr = ((ModBus.RX.DataAddrHigh << 8) | ModBus.RX.DataAddrLow) & 0xFFFF;
    ModBus.RX.DataCountHigh = ModBus.RX.Buffer[4];
    ModBus.RX.DataCountLow = ModBus.RX.Buffer[5];
    ModBus.RX.DataSize = ((ModBus.RX.DataCountHigh << 8) | ModBus.RX.DataCountLow) & 0xFFFF;
    ModBus.TX.DataSize = (uint8_t)(ModBus.RX.DataSize << 1);
    ModBus.RX.CRCLow = (uint8_t)((RTU_CRC((uint8_t *)ModBus.RX.Buffer, 6)) & 0xFF);
    ModBus.RX.CRCHigh = (uint8_t)((RTU_CRC((uint8_t *)ModBus.RX.Buffer, 6)) >> 8) & 0xFF;   
}
void ModBus_Rx06DataCollation(void)
{
    ModBus.RX.DataAddrHigh = ModBus.RX.Buffer[2];
    ModBus.RX.DataAddrLow = ModBus.RX.Buffer[3];      
    ModBus.RX.DataAddr = ((ModBus.RX.DataAddrHigh << 8) | ModBus.RX.DataAddrLow) & 0xFFFF;
    ModBus.RX.DataHigh = ModBus.RX.Buffer[4];
    ModBus.RX.DataLow = ModBus.RX.Buffer[5];
    ModBus.RX.Data = ((ModBus.RX.DataHigh << 8) | ModBus.RX.DataLow) & 0xFFFF;    
    ModBus.RX.CRCLow = (uint8_t)((RTU_CRC((uint8_t *)ModBus.RX.Buffer, 6)) & 0xFF);
    ModBus.RX.CRCHigh = (uint8_t)((RTU_CRC((uint8_t *)ModBus.RX.Buffer, 6)) >> 8) & 0xFF;   
}

void ModBus_ReturnEeprom(void)
{
    uart1_transfer.data = (uint8_t *)&TamagawaEncoder.Eeprom.DataBuffer[0][0];
    uart1_transfer.dataSize = 128; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);       
}
void ModBus_ReturnEepromPage0(void)
{
    uart1_transfer.data = (uint8_t *)&TamagawaEncoder.Eeprom.DataBuffer[0][0];
    uart1_transfer.dataSize = 128; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);       
}
void ModBus_ReturnEepromPage2(void)
{
    uart1_transfer.data = (uint8_t *)&TamagawaEncoder.Eeprom.DataBuffer[2][0];
    uart1_transfer.dataSize = 128; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);       
}
void EncoderTypeSet(uint8_t SetType, uint32_t SetEncoderBaudRate ,uint32_t SetCycle)
{
    Motor.EncoderType = SetType;
    Init_UART0(SetEncoderBaudRate);
    MotorEncoder.PitCycle = SetCycle;
    MotorEncoder.SamplingCycle = SetSamplingCycle/MotorEncoder.PitCycle;
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, MotorEncoder.PitCycle);//1us=48,设置通讯周期为500us 2KHz Cycle*48
    PIT_StartTimer(PIT, kPIT_Chnl_0);  
    ModBus.RX.Data = 0;   
}

void ClientEncoderTypeSet(uint8_t mtpType, uint8_t SetType, uint32_t SetEncoderBaudRate ,uint16_t SetCycle)
{
    Motor.MTPType = mtpType;
    Motor.EncoderType = SetType;
    Init_UART0(SetEncoderBaudRate);
    MotorEncoder.PitCycle = SetCycle;
    MotorEncoder.SamplingCycle = SetSamplingCycle/MotorEncoder.PitCycle;
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, MotorEncoder.PitCycle);//1us=48,设置通讯周期为500us 2KHz Cycle*48
    PIT_StartTimer(PIT, kPIT_Chnl_0);  
    ModBus.RX.Data = 0;   
}
void BasicReadings(void)
{
  switch(ModBus.RX.DataAddr){
  case 0x0000://获取测试板软件版本
      ModBus_ReturnTx03(EncoderTestFW, ModBus.TX.DataSize);  
    break;
  case 0x0001://获取测试版硬件版本
     ModBus_ReturnTx03(EncoderTestHW, ModBus.TX.DataSize);
    break;
  case 0x0002://获取编码器的报警
      //ModBus_ReturnTx03(AbsoluteEncoder.EncError.all, ModBus.TX.DataSize);
    break;
  case 0x0003://获取编码器的转速
      Motor.Flag.bit.ReadyTest = 0x00;
      //ModBus_ReturnTx03(AbsoluteEncoder.Speed, ModBus.TX.DataSize);    
    break;
  case 0x0004:
    myprintf("chui hui is good\r\n");
    break;    
  }
}
void TuosidaMTPprintf(void)
{
  myprintf("MTP的值为:\r\n");
  myprintf("额定电压:%d\n",MTP.VoltageType);
  myprintf("额定电流:%.2f\n",MTP.RatedCurrent);
  myprintf("过载能力:%.2f\n",MTP.OverloadCapacity);
  myprintf("额定转矩:%.2f\n",MTP.RatedTorque);
  myprintf("额定转速:%d\n",MTP.RatedSpeed);
  myprintf("最大转速:%d\n",MTP.MaximumSpeed);
  myprintf("转子惯量:%.8f\n",MTP.RotorInertiaFloat);
  myprintf("相反电势常数:%.2f\n",MTP.OppositePotentialConstant);
  myprintf("极对数:%d\n",MTP.NumberOfPolePairs);
  myprintf("磁通报警限值:%.2f\n",MTP.MagneticFluxAlarmLimit);
  myprintf("转矩报警限值:%.2f\n",MTP.TorqueAlarmLimit);
  myprintf("转矩过载阈值:%.2f\n",MTP.TorqueOverloadThreshold);
  myprintf("2倍过载时间:%d\n",MTP.TwotimesOverloadTime);
  myprintf("电角度:%.2f\n",MTP.ElectricalAngle);
  myprintf("电流环增益:%.2f\n",MTP.CurrenLoopGain);
  myprintf("电流环积分时间:%.2f\n",MTP.CurrentLoopIntegrationTime);
}

void BasicSettings(void)
{
  switch(ModBus.RX.DataAddr){
    case 0x0000://配置编码器断电      
      if(ModBus.RX.Data == 1){
        EncoderPowerOff();//编码器断电
        PIT_StopTimer(PIT, kPIT_Chnl_0); 
        Variable_ReadReset_Flag = 0x01;
        MotorEncoder.Flag.bit.PowerOn = 0;
        ModBus.RX.Data = 0;
      }else if(ModBus.RX.Data == 2){
        PIT_StopTimer(PIT, kPIT_Chnl_0); 
        Variable_ReadReset_Flag = 0x01;
        MotorEncoder.Flag.bit.PowerOn = 0;
        ModBus.RX.Data = 0;
      }else if(ModBus.RX.Data == 3){
        MotorEncoder.TestItem = TmagawaEncoderStopTest;
        EncoderPowerOff();//编码器断电
        MotorEncoder.Flag.bit.PowerOn = 0;
        ModBus.RX.Data = 0;
      }   
      ModBus_ReturnTx06();    
      break;
    case 0x0001://配置编码器上电  
      if(ModBus.RX.Data == 1){
        PIT_StartTimer(PIT, kPIT_Chnl_0); 
        EncoderPowerOn();//编码器上电
        MotorEncoder.Flag.bit.PowerOn = 1;
        ModBus.RX.Data = 0;
      }else if(ModBus.RX.Data == 2){
        EncoderPowerOff();//编码器断电
        PIT_StopTimer(PIT, kPIT_Chnl_0); 
        MotorEncoder.Flag.bit.PowerOn = 0;
        ModBus.RX.Data = 0;        
      }else if(ModBus.RX.Data == 3){
        PIT_StartTimer(PIT, kPIT_Chnl_0); 
        MotorEncoder.Flag.bit.PowerOn = 1;
        ModBus.RX.Data = 0;
      }else if(ModBus.RX.Data == 4){
        EncoderPowerOn();//编码器上电
        MotorEncoder.Flag.bit.PowerOn = 1;
        ModBus.RX.Data = 0;        
      }
      ModBus_ReturnTx06();    
      break;
    case 0x0002://停止和编码器通讯
      if(ModBus.RX.Data == 1){
//        Variable_ReadReset_Flag = 0x01;
        PIT_StopTimer(PIT, kPIT_Chnl_0); 
        ModBus.RX.Data = 0;
      }               
      ModBus_ReturnTx06();    
      break;
    case 0x0003://重新和编码器通讯并设定通讯周期 最大65535
      MotorEncoder.PitCycle = ModBus.RX.Data;
      MotorEncoder.SamplingCycle = SetSamplingCycle/MotorEncoder.PitCycle;
      PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, MotorEncoder.PitCycle);//1us=48,设置通讯周期为500us 2KHz Cycle*48      
      PIT_StartTimer(PIT, kPIT_Chnl_0); 
      ModBus.RX.Data = 0;             
      ModBus_ReturnTx06();    
      break;
    case 0x0004://设置编码器类型
      switch(ModBus.RX.Data){
        case MULTITURNMagneticEncoder://1
          EncoderTypeSet(MULTITURNMagneticEncoder, TamagawaEncoderBaudRate, 48000);               
          break;
        case MULTITURNOpticalEncoder://2
          EncoderTypeSet(MULTITURNOpticalEncoder, TamagawaEncoderBaudRate, 24000);                   
          break;
        case MGTMagneticEncoder://3
          EncoderTypeSet(MGTMagneticEncoder, TamagawaEncoderBaudRate, 6000);               
          break;
        case NXPMagneticEncoder://4
          EncoderTypeSet(NXPMagneticEncoder, TamagawaEncoderBaudRate, 6000);                  
          break;
        case TAMAGAWAEncoder://5
          EncoderTypeSet(TAMAGAWAEncoder, TamagawaEncoderBaudRate, 6000);
          break;
        case SENSAREncoder://6
          EncoderTypeSet(SENSAREncoder, SensAREncoderBaudRate, 24000);
          break; 
        case TuosidaMotor://7
          ClientEncoderTypeSet(TuosidaMotor,TAMAGAWAEncoder, TamagawaEncoderBaudRate, 6000);
          break;  
        case 8://2
          EncoderTypeSet(MULTITURNOpticalEncoder, TamagawaEncoderBaudRate, 6000);  //8KHz                 
          break;          
        }
      EnableIRQ(UART0_RX_TX_IRQn);
      ModBus_ReturnTx06();         
      break;
    case 0x0005://准备检测速度波动
      if(ModBus.RX.Data == 1){
        MotorEncoder.TestItem = MgtMagneticEncoderSpeedTest;           
        ModBus.RX.Data = 0;
      }             
      ModBus_ReturnTx06();    
      break;
    case 0x0006://写入测试年
      MotorEncoder.TestYear = (uint8_t)ModBus.RX.Data;  
      if(Motor.EncoderType == MGTMagneticEncoder){
        MotorEncoder.TestDate = MotorEncoder.TestYear;      
        MotorEncoder.TestDateCnt ++;
        MotorEncoder.TestItem = MgtMagneticEncoderWriteDateTest;         
      }    
      MotorEncoder.Flag.bit.Year = 1;
      MotorEncoder.Flag.bit.Moon = 0;
      MotorEncoder.Flag.bit.Day = 0;
      MotorEncoder.Flag.bit.Hour = 0;
      ModBus.RX.Data = 0;                   
      ModBus_ReturnTx06();    
      break;
    case 0x0007://写入测试月
      MotorEncoder.TestMoon = (uint8_t)ModBus.RX.Data;  
      if(Motor.EncoderType == MGTMagneticEncoder){
        MotorEncoder.TestDate = MotorEncoder.TestMoon;      
        MotorEncoder.TestDateCnt ++;
        MotorEncoder.TestItem = MgtMagneticEncoderWriteDateTest;         
      }
      MotorEncoder.Flag.bit.Year = 0;
      MotorEncoder.Flag.bit.Moon = 1;
      MotorEncoder.Flag.bit.Day = 0;
      MotorEncoder.Flag.bit.Hour = 0;
      ModBus.RX.Data = 0;            
      ModBus_ReturnTx06();    
      break;
    case 0x0008://写入测试日
      MotorEncoder.TestDay = (uint8_t)ModBus.RX.Data;
      if(Motor.EncoderType == MGTMagneticEncoder){
        MotorEncoder.TestDate = MotorEncoder.TestDay;      
        MotorEncoder.TestDateCnt ++;
        MotorEncoder.TestItem = MgtMagneticEncoderWriteDateTest;         
      }
      MotorEncoder.Flag.bit.Year = 0;
      MotorEncoder.Flag.bit.Moon = 0;
      MotorEncoder.Flag.bit.Day = 1;
      MotorEncoder.Flag.bit.Hour = 0;
      ModBus.RX.Data = 0;            
      ModBus_ReturnTx06();      
      break;
    case 0x0009://写入测试时
      MotorEncoder.TestHour = (uint8_t)ModBus.RX.Data; 
      switch(Motor.EncoderType){
      case MGTMagneticEncoder:
        MotorEncoder.TestDate = MotorEncoder.TestHour;      
        MotorEncoder.TestDateCnt ++;        
        MotorEncoder.TestItem = MgtMagneticEncoderWriteDateTest;         
        break;
      case MULTITURNMagneticEncoder:
        MotorEncoder.TestItem = MultiturnMagneticEncoderWriteDateTest; 
        break;
      case MULTITURNOpticalEncoder:
        MotorEncoder.TestItem = MultiturnOpticalEncoderWriteDateTest; 
        break;        
      }
      MotorEncoder.Flag.bit.Year = 0;
      MotorEncoder.Flag.bit.Moon = 0;
      MotorEncoder.Flag.bit.Day = 0;
      MotorEncoder.Flag.bit.Hour = 1;
      ModBus.RX.Data = 0;          
      ModBus_ReturnTx06();    
      break; 
  default:
    Communication_Address_Error();
    break;
  }
}

void ModBus_Rx03(void)
{
    ModBus_Rx03DataCollation();
     
    if(ModBus.RX.Buffer[6] != ModBus.RX.CRCLow || ModBus.RX.Buffer[7] != ModBus.RX.CRCHigh){
      ModBus_Crc_Error();      
    }else{
        if(ModBus.Error.bit.DC){
          Encoder_Timeout();             
        }else if(ModBus.Error.bit.EC){
          Encoder_CrcError();                          
        }else if(ModBus.Error.bit.SN){
          SNDigitsAreIncorrect();
        }
//        else if(AbsoluteEncoder.EncoderErr.bit.CE == 0x01 || AbsoluteEncoder.Status.bit.ea0 == 0x01){
//          Encoder_CeError();
//        }
        else{
          if(ModBus.RX.DataAddr < 0x100){
            BasicReadings();
          }else{
             switch(Motor.EncoderType){
              case MULTITURNMagneticEncoder:
                switch(ModBus.RX.DataAddr){
                case 0x0200://读取hall测试结果
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Hall.Result, ModBus.TX.DataSize);
                  break;
                case 0x0201://读取转速
                  ModBus_ReturnTx03(Motor.ActualSpeed, ModBus.TX.DataSize);
                  break;
                case 0x0202://读取初始化结果
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Status0x6D, ModBus.TX.DataSize);
                  break;                  
                case 0x0203://读取测试结果        
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[2][0], ModBus.TX.DataSize);
                  break;
                case 0x0204://读取编码器状态          
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Status.all, ModBus.TX.DataSize);
                  break;                   
                case 0x0205://读取编码器报警
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Error.all, ModBus.TX.DataSize);
                  break;
                case 0x0206://读取编码器主版本号
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[3][20], ModBus.TX.DataSize);//1
                  break;  
                case 0x0207://读取编码器子版本号
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[3][21], ModBus.TX.DataSize);//0
                  break;  
                case 0x0208://读取编码器修订版本号
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[3][22], ModBus.TX.DataSize);//4
                  break; 
                case 0x0209://读取编码器版本阶段
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[3][23], ModBus.TX.DataSize);//82
                  break;         
                case 0x020A://读取电池电压
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.BatteryVoltage, ModBus.TX.DataSize);
                  break;   
                case 0x020B://读取FCT年
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[0x02][0x68], ModBus.TX.DataSize);
                  break;   
                case 0x020C://读取FCT月
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[0x02][0x69], ModBus.TX.DataSize);
                  break;
                case 0x020D://读取FCT日
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[0x02][0x6A], ModBus.TX.DataSize);
                  break;
                case 0x020E://读取FCT时
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[0x02][0x6B], ModBus.TX.DataSize);
                  break;      
                case 0x020F://读取编码器分辨率
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.ReadResolutionID, ModBus.TX.DataSize);
                  break;    
                case 0x0210://读取MT6835寄存器的值
                  ModBus_ReturnTx03(MT6835Addr.TestRegData, ModBus.TX.DataSize);
                  break;      
                case 0x0211:
                  ModBus_ReturnTx03(MotorEncoder.PitCnt, ModBus.TX.DataSize);
                  break;
                case 0x0212:
                  ModBus_ReturnTx03((MultiturnMagneticEncoder.Hall.Buffer[3]<<12)|(MultiturnMagneticEncoder.Hall.Buffer[2]<<8)|(MultiturnMagneticEncoder.Hall.Buffer[1]<<4)|(MultiturnMagneticEncoder.Hall.Buffer[0]) , ModBus.TX.DataSize);     
                  break;
                case 0x0213://读取温度报警值
                  ModBus_ReturnTx03(MultiturnMagneticEncoder.Eeprom.DataBuffer[0x03][0x0A], ModBus.TX.DataSize);
                  break;                  
                default:
                  Communication_Address_Error();
                  break;                  
                }               
                break;
              case MULTITURNOpticalEncoder:
                switch(ModBus.RX.DataAddr){
                case 0x0300://读取MTAB的值
                  ModBus_ReturnTx03(PNH2612.MTABSector, ModBus.TX.DataSize);
                  break;
                case 0x0301://读取霍尔的值
                  ModBus_ReturnTx03(PNH2612.HallSector, ModBus.TX.DataSize);
                  break;
                case 0x0302://读取初始化结果
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Status0x6D, ModBus.TX.DataSize);
                  break;                  
                case 0x0303://读取测试结果        
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[2][0], ModBus.TX.DataSize);
                  break;
                case 0x0304://读取编码器状态          
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Status.all, ModBus.TX.DataSize);
                  break;                   
                case 0x0305://读取编码器报警
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Error.all, ModBus.TX.DataSize);
                  break;
                case 0x0306://读取编码器主版本号
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[0x03][0x14], ModBus.TX.DataSize);//1
                  break;  
                case 0x0307://读取编码器子版本号
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[0x03][0x15], ModBus.TX.DataSize);//0
                  break;  
                case 0x0308://读取编码器修订版本号
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[0x03][0x16], ModBus.TX.DataSize);//4
                  break; 
                case 0x0309://读取编码器版本阶段
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[0x03][0x17], ModBus.TX.DataSize);//82
                  break;         
                case 0x030A://读取电池电压
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.BatteryVoltage, ModBus.TX.DataSize);
                  break;  
                case 0x030B://读取温度
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Temperature, ModBus.TX.DataSize);
                  break;
                case 0x030C://读取FCT年
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[0x02][0x68], ModBus.TX.DataSize);
                  break;   
                case 0x030D://读取FCT月
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[0x02][0x69], ModBus.TX.DataSize);
                  break;
                case 0x030E://读取FCT日
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[0x02][0x6A], ModBus.TX.DataSize);
                  break;
                case 0x030F://读取FCT时
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Eeprom.DataBuffer[0x02][0x6B], ModBus.TX.DataSize);
                  break; 
                case 0x0310://读取MSinMax值
                  ModBus_ReturnTx03(PNH2612.M.SinDataMax, ModBus.TX.DataSize);
                  break;
                case 0x0311://读取MCosMax值
                  ModBus_ReturnTx03(PNH2612.M.CosDataMax, ModBus.TX.DataSize);
                  break;
                case 0x0312://读取NSinMax值
                  ModBus_ReturnTx03(PNH2612.N.SinDataMax, ModBus.TX.DataSize);
                  break;   
                case 0x0313://读取NCosMax值
                  ModBus_ReturnTx03(PNH2612.N.CosDataMax, ModBus.TX.DataSize);
                  break; 
                case 0x0314://读取SSinMax值
                  ModBus_ReturnTx03(PNH2612.S.SinDataMax, ModBus.TX.DataSize);
                  break; 
                case 0x0315://读取SCosMax值
                  ModBus_ReturnTx03(PNH2612.S.CosDataMax, ModBus.TX.DataSize);
                  break; 
                case 0x0316://读取MSin值
                  ModBus_ReturnTx03(PNH2612.M.SinData, ModBus.TX.DataSize);
                  break;
                case 0x0317://读取MCos值
                  ModBus_ReturnTx03(PNH2612.M.CosData, ModBus.TX.DataSize);
                  break;
                case 0x0318://读取NSin值
                  ModBus_ReturnTx03(PNH2612.N.SinData, ModBus.TX.DataSize);
                  break;   
                case 0x0319://读取NCos值
                  ModBus_ReturnTx03(PNH2612.N.CosData, ModBus.TX.DataSize);
                  break; 
                case 0x031A://读取SSin值
                  ModBus_ReturnTx03(PNH2612.S.SinData, ModBus.TX.DataSize);
                  break; 
                case 0x031B://读取SCos值
                  ModBus_ReturnTx03(PNH2612.S.CosData, ModBus.TX.DataSize);
                  break; 
                case 0x031C://读取光强度测试结果
                  ModBus_ReturnTx03(PNH2612.LEDTestDACResult, ModBus.TX.DataSize);
                  break;
                case 0x031D://读取内部报警1
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.InternalAlarm1, ModBus.TX.DataSize);
                  break;
                case 0x031E://读取内部报警2
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.InternalAlarm2, ModBus.TX.DataSize);
                  break;
                case 0x031F://读取内部报警3
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.InternalAlarm3, ModBus.TX.DataSize);
                  break;
                case 0x0320://读取hall扇区结果
                  MultiturnOpticalEncoder.Hall.Sector = ((MultiturnOpticalEncoder.Hall.Buffer[0] << 12) | (MultiturnOpticalEncoder.Hall.Buffer[1] << 8) | (MultiturnOpticalEncoder.Hall.Buffer[2] << 4) | (MultiturnOpticalEncoder.Hall.Buffer[3])) & 0xFFFF; 
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Hall.Sector, ModBus.TX.DataSize);
                  break;
                case 0x0321://读取MTAB扇区结果
                  MultiturnOpticalEncoder.MTAB.Sector = ((MultiturnOpticalEncoder.MTAB.Buffer[0] << 12) | (MultiturnOpticalEncoder.MTAB.Buffer[1] << 8) | (MultiturnOpticalEncoder.MTAB.Buffer[2] << 4) | (MultiturnOpticalEncoder.MTAB.Buffer[3])) & 0xFFFF; 
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.MTAB.Sector, ModBus.TX.DataSize);
                  break;                  
                case 0x0322://读取扇区测试结果,合格为3
                  ModBus_ReturnTx03((MultiturnOpticalEncoder.Hall.Result << 1) | MultiturnOpticalEncoder.MTAB.Result, ModBus.TX.DataSize);
                  break;
                case 0x0323://读取MSinCos差值
                  ModBus_ReturnTx03(abs(PNH2612.M.SinData - PNH2612.M.CosData), ModBus.TX.DataSize);
                  break;
                case 0x0324://读取NSinCos差值
                  ModBus_ReturnTx03(abs(PNH2612.N.SinData - PNH2612.N.CosData), ModBus.TX.DataSize);
                  break;
                case 0x0325://读取SSinCos差值
                  ModBus_ReturnTx03(abs(PNH2612.S.SinData - PNH2612.S.CosData), ModBus.TX.DataSize);
                  break; 
                case 0x0326:
                  ModBus_ReturnTx03(PNH2612.M.FeedbackSpeed, ModBus.TX.DataSize);
                  break;
                case 0x0327:
                  ModBus_ReturnTx03(PNH2612.N.FeedbackSpeed, ModBus.TX.DataSize);
                  break;
                case 0x0328:
                  ModBus_ReturnTx03(PNH2612.S.FeedbackSpeed, ModBus.TX.DataSize);
                  break;       
                case 0x0329:
                  ModBus_ReturnTx03(PNH2612.M.MarkPositionDifference, ModBus.TX.DataSize);
                  break;
                case 0x032A:
                  ModBus_Return32bitTx03(abs(Motor.FctPos_MPos_Diff[1] - Motor.FctPos_MPos_Diff[0]), ModBus.TX.DataSize);
                  break; 
                case 0x032B:
                  ModBus_ReturnTx03(PNH2612.MNSAnalogResult, ModBus.TX.DataSize);
                  break;  
                case 0x032C://读取DAC值
                  ModBus_ReturnTx03(PNH2612.LEDAdjustDAC, ModBus.TX.DataSize);
                  break;
                case 0x032D://读取HALLCheck: 发生的错误类型是什么（2=跳变，3=抖动，4=步数错）。
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Hall.Check, ModBus.TX.DataSize);
                  break;                     
                case 0x032E://读取HALL扇区跨区突变个数如果很大，可能是干扰或码盘损坏
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Hall.GlitchCnt, ModBus.TX.DataSize);
                  break;   
                case 0x032F://读取HALL扇区逻辑反转个数（0->1->0）。如果很大，可能是机械装配松动或临界区不稳定。
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Hall.JitterCnt, ModBus.TX.DataSize);
                  break;                     
                case 0x0330://读取HALL扇区是否转满一圈如果是 1，说明转了一圈但 Cnt 不是 4（可能是 5, 6 等）
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.Hall.CountErrCnt, ModBus.TX.DataSize);
                  break;                  
                case 0x0331://读取MTABCheck: 发生的错误类型是什么（2=跳变，3=抖动，4=步数错）。
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.MTAB.Check, ModBus.TX.DataSize);
                  break;                     
                case 0x0332://读取MTAB扇区跨区突变个数如果很大，可能是干扰或码盘损坏
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.MTAB.GlitchCnt, ModBus.TX.DataSize);
                  break;   
                case 0x0333://读取MTAB扇区逻辑反转个数（0->1->0）。如果很大，可能是机械装配松动或临界区不稳定。
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.MTAB.JitterCnt, ModBus.TX.DataSize);
                  break;                     
                case 0x0334://读取MTAB扇区是否转满一圈如果是 1，说明转了一圈但 Cnt 不是 4（可能是 5, 6 等）
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.MTAB.CountErrCnt, ModBus.TX.DataSize);
                  break;    
                case 0x0335://读取MTAB扇区是否转满一圈如果是 1，说明转了一圈但 Cnt 不是 4（可能是 5, 6 等）
                  ModBus_ReturnTx03(MultiturnOpticalEncoder.MTAB.CountErrCnt, ModBus.TX.DataSize);
                  break; 
                case 0x0336://读取同时判断是否介于2000到2100之间，用于验证灯有没有亮  在这步之后0x0302://开启6路模拟信号最大值检测//十字联轴不在这里检测
                  ModBus_ReturnTx03(PNH2612.MNSAnalogSynchronousCheckResult, ModBus.TX.DataSize); 
                  break;               
                default:
                  Communication_Address_Error();
                  break;              
                }                        
                break;
              case MGTMagneticEncoder:
                switch(ModBus.RX.DataAddr){
                case 0x0400://判断eeprom所有数据是否读取完毕
                  ModBus_ReturnTx03(MotorEncoder.TestItem, ModBus.TX.DataSize);//MotorEncoder.TestItem = 0代表读取完毕了
                  break;
                case 0x0401://读取转速
                  ModBus_ReturnTx03(Motor.FilterSpeed, ModBus.TX.DataSize);
                  break;
                case 0x0402:
                  ModBus_ReturnTx03(MgtMagneticEncoder.Eeprom.DataBuffer[0][0x6F], ModBus.TX.DataSize);
                  break;
                case 0x0403:
                  ModBus_ReturnTx03(MgtMagneticEncoder.Eeprom.DataBuffer[0][0x70], ModBus.TX.DataSize);
                  break;
                case 0x0404://读取软件版本 105版本
                  ModBus_ReturnTx03(MgtMagneticEncoder.FirmwareVersion, ModBus.TX.DataSize);
                  break;  
                case 0x0405://读取向编码器发送0xEA的返回值
                  ModBus_ReturnTx03(MgtMagneticEncoder.Eeprom.DataBuffer[MgtMagneticEncoder.Eeprom.ReturnPage][MgtMagneticEncoder.Eeprom.ReturnAddress], ModBus.TX.DataSize);
                  break;
                case 0x0406://读取向编码器发送0xA2的返回值
                  ModBus_ReturnTx03(MgtMagneticEncoder.Eeprom.Data, ModBus.TX.DataSize);
                  break;                  
                case 0x0407://读取软件版本 106往后
                  ModBus_Return32bitTx03(MgtMagneticEncoder.Firmware, ModBus.TX.DataSize);
                  break;
                case 0x0408://读取编码器分辨率
                  ModBus_ReturnTx03(MgtMagneticEncoder.ResolutionID, ModBus.TX.DataSize);
                  break;                  
                default:
                  Communication_Address_Error();
                  break;                  
                }
                break;
              case NXPMagneticEncoder:
                switch(ModBus.RX.DataAddr){
                case 0x0500://判断eeprom所有数据是否读取完毕
                  ModBus_ReturnTx03(MotorEncoder.TestItem, ModBus.TX.DataSize);//MotorEncoder.TestItem = 0代表读取完毕了
                  break;
                case 0x0501://读取转速
                  ModBus_ReturnTx03(Motor.FilterSpeed, ModBus.TX.DataSize);
                  break;
                case 0x0502:
                  ModBus_ReturnTx03(NxpMagneticEncoder.Eeprom.DataBuffer[0x6F], ModBus.TX.DataSize);
                  break;
                case 0x0503:
                  ModBus_ReturnTx03(NxpMagneticEncoder.Eeprom.DataBuffer[0x70], ModBus.TX.DataSize);
                  break;
                case 0x0504://读取软件版本
                  ModBus_ReturnTx03(NxpMagneticEncoder.FirmwareVersion, ModBus.TX.DataSize);
                  break;  
                default:
                  Communication_Address_Error();
                  break;                  
                }                     
                break;
              case TAMAGAWAEncoder:
                switch(ModBus.RX.DataAddr){
                  case 0x0600://获取编码器的ID
                    ModBus_ReturnTx03(TamagawaEncoder.ResolutionID, ModBus.TX.DataSize);
                    break;
                  case 0x0601://获取编码器状态
                    ModBus_ReturnTx03(TamagawaEncoder.Status.all, ModBus.TX.DataSize);
                    break;
                  case 0x0602://获取编码器报警
                    ModBus_ReturnTx03(TamagawaEncoder.Error.all, ModBus.TX.DataSize);
                    break;
                  case 0x0603://获取编码器多圈数据
                    ModBus_ReturnTx03(TamagawaEncoder.MultiTurnPosition, ModBus.TX.DataSize);
                    break;  
                  case 0x0604://获取MTP
                    ModBus_ReturnEeprom();
                    break;
                  case 0x0605://获取编码器单圈数据
                    ModBus_Return32bitTx03(TamagawaEncoder.SingleTurnPosition, ModBus.TX.DataSize);
                    break;  
                  case 0x0606://打印MTP
                    if(Motor.MTPType == TuosidaMotor){
                      TuosidaMTPprintf();
                    }
                    break; 
                  case 0x0607://获取编码器相位偏差
                    ModBus_ReturnTx03(TamagawaEncoder.PhaseAngle, ModBus.TX.DataSize);
                    break;
                  case 0x0608:
                    ModBus_ReturnEepromPage0();
                    break;
                  case 0x0609:
                    ModBus_ReturnEepromPage2();
                    break;                    
                default:
                  Communication_Address_Error();
                  break;                   
                }
                break;  
              case SENSAREncoder:
                switch(ModBus.RX.DataAddr){
                  case 0x0700://获取编码器的ID
                    //ModBus_ReturnTx03(TamagawaEncoder.ResolutionID, ModBus.TX.DataSize);
                    break;
                  case 0x0701://获取获取hall1 幅值
                    ModBus_ReturnTx03(SensAR.Hall_1, ModBus.TX.DataSize);
                    break;
                  case 0x0702://获取获取hall2 幅值
                    ModBus_ReturnTx03(SensAR.Hall_2, ModBus.TX.DataSize);
                    break;
                  case 0x0703://获取获取hall3 幅值
                    ModBus_ReturnTx03(SensAR.Hall_3, ModBus.TX.DataSize);
                    break; 
                  case 0x0704://获取获取hall4 幅值
                    ModBus_ReturnTx03(SensAR.Hall_4, ModBus.TX.DataSize);
                    break;
                  case 0x0705://获取获取hall5 幅值
                    ModBus_ReturnTx03(SensAR.Hall_5, ModBus.TX.DataSize);
                    break;
                  case 0x0706://获取获取hall6 幅值
                    ModBus_ReturnTx03(SensAR.Hall_6, ModBus.TX.DataSize);
                    break; 
                  case 0x0707://获取获取hall7 幅值
                    ModBus_ReturnTx03(SensAR.Hall_7, ModBus.TX.DataSize);
                    break;                     
                default:
                  Communication_Address_Error();
                  break;                   
                }
                break;                    
            }           
          }          
        }
    }  
}

void ModBus_Rx06(void)
{
    ModBus_Rx06DataCollation();
    if(ModBus.RX.Buffer[6] != ModBus.RX.CRCLow || ModBus.RX.Buffer[7] != ModBus.RX.CRCHigh){
        ModBus_Crc_Error();          
    }else{
        if(ModBus.Error.bit.DC){
          if(ModBus.RX.DataAddr == 0x0000){
            if(ModBus.RX.Data == 1){
              EncoderPowerOff();//编码器断电
              PIT_StopTimer(PIT, kPIT_Chnl_0);   
              Variable_ReadReset_Flag = 0x01;
              ModBus.RX.Data = 0;
            }
            ModBus_ReturnTx06();             
          }else{
            Encoder_Timeout();  
          }             
        }else if(ModBus.Error.bit.EC){
          if(ModBus.RX.DataAddr == 0x0000){
            if(ModBus.RX.Data == 1){
              EncoderPowerOff();//编码器断电
              PIT_StopTimer(PIT, kPIT_Chnl_0);   
              Variable_ReadReset_Flag = 0x01;
              ModBus.RX.Data = 0;
            }
            ModBus_ReturnTx06();             
          }else{
            Encoder_CrcError();  
          }                       
        }else if(ModBus.Error.bit.SN){
          SNDigitsAreIncorrect();
        }
//        else if(AbsoluteEncoder.EncoderErr.bit.CE == 0x01 || AbsoluteEncoder.Status.bit.ea0 == 0x01){
//          Encoder_CeError();
//        }
        else{ 
          if(ModBus.RX.DataAddr < 0x0100){
            BasicSettings();
          }else{
            switch(Motor.EncoderType){
              case MULTITURNMagneticEncoder:
                switch(ModBus.RX.DataAddr){
                case 0x0200://读取eeprom前4页数据（4*127）
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderReadAllEepromTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0201://编码器初始化
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderInitializeTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0202://测试hall的值
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderHallTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break; 
                case 0x0203://写入测试结果
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderWriteResultTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;                   
                case 0x0204://设置读取编码器所有信息
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderGetAllDataTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                   
                  break;
                case 0x0205://设置多圈报警清除
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderMultiturnTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break; 
                case 0x0206://设置电池电压和温度读取
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderTBTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();      
                  break; 
                case 0x0207://打开内部协议
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderOIPTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0208://关闭内部协议
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderCIPTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;                     
                case 0x0209://设置编码器分辨率
                    MotorEncoder.SetResolutionID = ModBus.RX.Data;
                    MotorEncoder.TestItem = MultiturnMagneticEncoderSetResolutionTest;                        
                    ModBus.RX.Data = 0;           
                    ModBus_ReturnTx06();                   
                  break;   
                case 0x020A://设置读取编码器分辨率
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderReadResolutionTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                   
                  break; 
                case 0x020B://设置写入硬件版本
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderWriteHWRevTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                   
                  break;  
                case 0x020C://设置写入MT6835寄存器内容
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnMagneticEncoderWriteRegisterTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                   
                  break;                           
                default:
                  Communication_Address_Error();
                  break;                  
                }                
                break;
              case MULTITURNOpticalEncoder:
                switch(ModBus.RX.DataAddr){
                case 0x0300://设置读取eeprom前4页数据（4*127）
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderReadAllEepromTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0301://编码器初始化
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderInitializeTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0302://开启6路模拟信号最大值检测//十字联轴不在这里检测
                    if(ModBus.RX.Data == 1){
                      MultiturnOpticalEncoder.Flag.bit.AMD = 1;
                      MotorEncoder.TestItem = MultiturnOpticalEncoderAnalogDataTest;                          
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break; 
                case 0x0303://写入测试结果
                    if(ModBus.RX.Data == 1){
                      MultiturnOpticalEncoder.TestResult = ModBus.RX.Data;
                      MotorEncoder.TestItem = MultiturnOpticalEncoderWriteResultTest;                         
                      ModBus.RX.Data = 0;
                    }else if(ModBus.RX.Data == 9){
                      MultiturnOpticalEncoder.TestResult = ModBus.RX.Data;                      
                      MotorEncoder.TestItem = MultiturnOpticalEncoderWriteResultTest; 
                      ModBus.RX.Data = 0;                      
                    }
                    ModBus_ReturnTx06();                    
                  break;                   
                case 0x0304://设置读取编码器所有信息
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderGetAllDataTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                   
                  break;
                case 0x0305://设置多圈报警清除
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderMultiturnTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break; 
                case 0x0306://设置电池电压和温度读取
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderTBTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();      
                  break; 
                case 0x0307://打开内部协议
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderOIPTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0308://关闭内部协议
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderCIPTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;  
                case 0x0309://设置DAC                   
                    PNH2612.LEDTestDAC = ModBus.RX.Data;
                    MultiturnOpticalEncoder.Flag.bit.AMD = 0;
                    MotorEncoder.TestItem = MultiturnOpticalEncoderDACTest;                    
                    ModBus.RX.Data = 0;                                
                    ModBus_ReturnTx06();                    
                  break;
                case 0x030A://设置光强度测试
                    if(ModBus.RX.Data == 1){
                      MultiturnOpticalEncoder.Flag.bit.LedDac = 1;   
                      MotorEncoder.TestItem = MultiturnOpticalEncoderLightIntensityTest;                                             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                
                  break;  
                case 0x030B://设置读取内部报警
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderReadInternalAlarmTest;                          
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;
                case 0x030C://设置RMRNRS初始值
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderRMRNRSInitializeTest;                          
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;
                case 0x030D://设置写入硬件版本
                    MotorEncoder.HWRevData = ModBus.RX.Data;
                    MotorEncoder.TestItem = MultiturnOpticalEncoderWriteHWRevTest;             
                    ModBus.RX.Data = 0;            
                    ModBus_ReturnTx06();  
                  break;
                case 0x030E://设置读取MTABHALL的扇区1 2 3 4对应0° 90° 180° 270°
                    MultiturnOpticalEncoder.Flag.bit.MTABHALL = ModBus.RX.Data;         
                    ModBus.RX.Data = 0;            
                    ModBus_ReturnTx06();  
                  break; 
                case 0x030F://设置MTABHALL扇区检测
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderSectorTest;                          
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();      
                  break;     
                case 0x0310://设置读取MNS三码道位置
                   if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderMNSPositionTest;                          
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;
                case 0x0311://设置记录M位置
                   if(ModBus.RX.Data == 1){
                      PNH2612.M.PositionFlag ++;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;
                case 0x0312://设置获取MNS码道模拟量最大值检测结果
                   if(ModBus.RX.Data == 1){
                      MultiturnOpticalEncoder.Flag.bit.AMD = 2;
                      MotorEncoder.TestItem = MultiturnOpticalEncoderMNSAnalogMaxCheckTest;                          
                      ModBus.RX.Data = 0;
                    }   
                   ModBus_ReturnTx06(); 
                  break;  
                case 0x0313://设置获取MNS 模拟信号0x85
                   if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MultiturnOpticalEncoderReadAnalogDataTest;                          
                      ModBus.RX.Data = 0;
                    }   
                   ModBus_ReturnTx06(); 
                  break;                   
                default:
                  Communication_Address_Error();
                  break;                  
                }                         
                break;
              case MGTMagneticEncoder:                
                switch(ModBus.RX.DataAddr){
                case 0x0400://读取eeprom所有数据（7*127）
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderReadAllEepromTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0401://编码器初始化
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderInitializeTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0402://编码器进行速度检测
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderSpeedTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                   
                  break;
                case 0x0403://设置读取软件版本 105版本
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderFirmwareVersionTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;
                case 0x0404://测试eeprom全部地址写入
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderWriteAllEepromTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break; 
                case 0x0405://读取所有数据0
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderGetAllDataTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();   
                  break;
                case 0x0406://0x32指令写eeprom
                    MgtMagneticEncoder.Eeprom.SetAddress = ModBus.RX.DataHigh;//高位是地址或者是0x7F
                    MgtMagneticEncoder.Eeprom.Data = ModBus.RX.DataLow;//低位是数据或者是页数
                    MotorEncoder.TestItem = MgtMagneticEncoderCommand0x32Test;
                    ModBus.RX.Data = 0;
                    ModBus_ReturnTx06();                  
                  break;  
                case 0x0407://0xEA指令读eeprom
                    MgtMagneticEncoder.Eeprom.SetAddress = ModBus.RX.DataHigh;//高位是地址或者是0x7F
                    MotorEncoder.TestItem = MgtMagneticEncoderCommand0xEATest;      
                    ModBus.RX.Data = 0;
                    ModBus_ReturnTx06();                  
                  break;
                case 0x0408://打开内部协议
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderOIPTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0409://关闭内部协议
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderCIPTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;                  
                case 0x040A://0x7A指令写eeprom 传输地址
                    MgtMagneticEncoder.Eeprom.Address = ModBus.RX.Data;
                    ModBus.RX.Data = 0;
                    ModBus_ReturnTx06();                   
                  break;
                case 0x040B:////0x7A指令写eeprom 传输数据
                    MgtMagneticEncoder.Eeprom.Data = ModBus.RX.DataHigh;//高位是数据
                    MotorEncoder.TestItem = MgtMagneticEncoderCommand0x7ATest;                    
                    ModBus.RX.Data = 0;
                    ModBus_ReturnTx06();                       
                  break;
                case 0x040C://0xA2指令读eeprom
                    MgtMagneticEncoder.Eeprom.Address = ModBus.RX.Data;
                    MotorEncoder.TestItem = MgtMagneticEncoderCommand0xA2Test;        
                    ModBus.RX.Data = 0;
                    ModBus_ReturnTx06();                    
                  break;       
                case 0x040D://设置读取软件版本
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderFirmwareTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;
                case 0x040E://设置编码器分辨率
                    MotorEncoder.SetResolutionID = ModBus.RX.Data;
                    MotorEncoder.TestItem = MgtMagneticEncoderSetResolutionTest;       
                    ModBus.RX.Data = 0;             
                    ModBus_ReturnTx06();                    
                  break; 
                case 0x040F://设置读取编码器分辨率
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderReadResolutionTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;  
                case 0x0410://设置写入硬件版本
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.HWRevAddr = 0x030D;  
                      MgtMagneticEncoder.Eeprom.Status.bit.Write = 1;
                      MotorEncoder.TestItem = MgtMagneticEncoderWriteHWRevTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break; 
                case 0x0411://设置读取硬件版本
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.HWRevAddr = 0x030D;  
                      MotorEncoder.TestItem = MgtMagneticEncoderReadHWRevTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;  
                case 0x0412://设置编码器通讯超时测试
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = MgtMagneticEncoderTimeoutTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;                   
                default:
                  Communication_Address_Error();
                  break;                  
                }
                break;
              case NXPMagneticEncoder:
                switch(ModBus.RX.DataAddr){
                case 0x0500://读取eeprom所有数据（7*127）
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = NxpMagneticEncoderReadAllEepromTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0501://编码器初始化
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = NxpMagneticEncoderInitializeTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                  break;
                case 0x0502://编码器进行速度检测
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = NxpMagneticEncoderSpeedTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                   
                  break;
                case 0x0503://设置读取软件版本
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = NxpMagneticEncoderFirmwareVersionTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;
                case 0x0504://测试eeprom全部地址写入
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = NxpMagneticEncoderWriteAllEepromTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                    
                  break;                  
                default:
                  Communication_Address_Error();
                  break;                  
                }                     
                break;
              case TAMAGAWAEncoder:
                switch(ModBus.RX.DataAddr){
                  case 0x0600://设置多圈报警复位
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = TmagawaEncoderMultiturnTest;             
                      ModBus.RX.Data = 0;
                    }             
                    ModBus_ReturnTx06();                     
                    break;
                  case 0x0601://设置获取编码器所有信息
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = TmagawaEncoderGetAllDataTest;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06(); 
                    break;
                  case 0x0602://设置获取MTP
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = TmagawaEncoderReadMTPTest;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06(); 
                    break;                    
                  case 0x0603://设置单圈位置复位成0
                    if(ModBus.RX.Data == 1){
                      MotorEncoder.TestItem = TmagawaEncoderSingleTurnTest;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break;
                    
                default:
                  Communication_Address_Error();
                  break;                   
                }
                break;
              case SENSAREncoder:
                switch(ModBus.RX.DataAddr){
                  case 0x0700://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestFlag = 1;
                      SensAR.CycleFlag = 1;
                      SensARDataInitialization(); 
                      //EncoderPowerOn();//编码器上电
                    }             
                    ModBus_ReturnTx06();                     
                    break;
                  case 0x0701://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensAREnterTest;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06(); 
                    break;
                  case 0x0702://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARgetprdinfoTest;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break;
                  case 0x0703://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARgethallvalue1Test;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break; 
                  case 0x0704://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARgethallvalue2Test;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break;  
                  case 0x0705://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARgethallvalue3Test;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break;     
                  case 0x0706://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARgethallvalue4Test;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break; 
                  case 0x0707://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARgethallvalue5Test;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break; 
                  case 0x0708://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARgethallvalue6Test;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break; 
                  case 0x0709://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARgethallvalue7Test;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break;
                  case 0x070A://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARsetprdinfoTest;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break;  
                  case 0x070B://
                    if(ModBus.RX.Data == 1){
                      SensAR.TestContent = SensARverTest;
                      SensAR.CycleFlag = 1;
                      SensAR.Flag.bit.Enter = 1;
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break; 
                  case 0x070C://
                    if(ModBus.RX.Data == 1){
                      EncoderPowerOff();//编码器下电
                      ModBus.RX.Data = 0;
                    }
                    ModBus_ReturnTx06();                     
                    break;                      
                default:
                  Communication_Address_Error();
                  break;                   
                }              
                break;
            default:
              NotSetEncoderType();
              break;
            }
          }
        }
    }
}

//==============================================================================
//Encoder_Timeout
//void Encoder_Timeout(void)
//==============================================================================
void Encoder_Timeout(void)
{   
    Work_Alarm = 0x01;
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD | 0x80;            
    ModBus.TX.Buffer[2] = Work_Alarm | 0x50;
    ModBus.TX.Buffer[3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) >> 8) & 0xFF;  
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 5;  
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);    
}
//==============================================================================
//Encoder_CrcError
//void Encoder_CrcError(void)
//==============================================================================
void Encoder_CrcError(void)
{
    Work_Alarm = 0x02;
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD | 0x80;            
    ModBus.TX.Buffer[2] = Work_Alarm | 0x50;
    ModBus.TX.Buffer[3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) >> 8) & 0xFF;  
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 5;  
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);   
}
//==============================================================================
//Encoder_CeError
//void Encoder_CeError(void)
//==============================================================================
//void Encoder_CeError(void)
//{
//    Work_Alarm = 0x03;
//    ModBus.TX.Buffer[0] = ModBus.ADR;
//    ModBus.TX.Buffer[1] = ModBus.CMD | 0x80;            
//    ModBus.TX.Buffer[2] = Work_Alarm;
//    ModBus.TX.Buffer[3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) & 0xFF);
//    ModBus.TX.Buffer[4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) >> 8) & 0xFF;  
//    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
//    uart1_transfer.dataSize = 5;  
//    SerialPort485DE();                                                       //Write enabled
//    for (volatile uint8_t i=0; i<5; i++);
//    UART1_SendDMA0(&uart1_transfer);   
//}
//==============================================================================
//The communication address is not used
//void Communication_Address_Error(void)
//==============================================================================
void Communication_Address_Error(void)
{
    Work_Alarm = 0x03;
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD | 0x80;            
    ModBus.TX.Buffer[2] = Work_Alarm | 0x50;
    ModBus.TX.Buffer[3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) >> 8) & 0xFF;  
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 5; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer); 
}
//==============================================================================
//ModBus CRC Error
//void ModBus_Crc_Error(void)
//==============================================================================
void ModBus_Crc_Error(void)
{
    Work_Alarm = 0x04;
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD | 0x80;            
    ModBus.TX.Buffer[2] = Work_Alarm | 0x50;
    ModBus.TX.Buffer[3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) >> 8) & 0xFF;  
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 5; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer);   
}
//==============================================================================
//The Encoder type is not setted
//void NotSetEncoderType(void)
//==============================================================================
void NotSetEncoderType(void)
{
    Work_Alarm = 0x05;
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD | 0x80;            
    ModBus.TX.Buffer[2] = Work_Alarm | 0x50;
    ModBus.TX.Buffer[3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) >> 8) & 0xFF;  
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 5; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer); 
}
//==============================================================================
//SN位数错误
//void SNDigitsAreIncorrect(void)
//==============================================================================
void SNDigitsAreIncorrect(void)
{
    Work_Alarm = 0x06;
    ModBus.TX.Buffer[0] = ModBus.ADR;
    ModBus.TX.Buffer[1] = ModBus.CMD | 0x80;            
    ModBus.TX.Buffer[2] = Work_Alarm | 0x50;
    ModBus.TX.Buffer[3] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) & 0xFF);
    ModBus.TX.Buffer[4] = (uint8_t)((RTU_CRC((uint8_t *)ModBus.TX.Buffer, 3)) >> 8) & 0xFF;  
    uart1_transfer.data = (uint8_t *)&ModBus.TX.Buffer[0];
    uart1_transfer.dataSize = 5; 
    SerialPort485DE();                                                       //Write enabled
    for (volatile uint8_t i=0; i<5; i++);
    UART1_SendDMA0(&uart1_transfer); 
}
