/****************************************************************************************
  * @file     SensAR_function.c
  * @brief    SensAR编码器配置
  *           
****************************************************************************************/
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "SensAREncoder.h"
#include "Function.h"

strSensAR   SensAR = {0};
/**************************************************************************************
* 函数名称：SensAR_Crc()
* 函数功能：配置SensAR_Crc
* 输入参量：Data 需要校验的数据
            Len  需要校验数据个数
* 输出参量：uCRC CRC值
***************************************************************************************/
uint8_t SensAR_Crc(volatile char * Data, uint8_t Len)
{  
    uint8_t uCRC = 0;  
          
    while(Len--){  
                  uCRC += *Data++;  
    }  

    return uCRC;  
}

/**************************************************************************************
* 函数名称：SensARDataInitialization()
* 函数功能：配置一些空格、双引号等
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void SensARDataInitialization(void)
{
    SensAR.setprdinfo[0] = 's';
    SensAR.setprdinfo[1] = 'e';
    SensAR.setprdinfo[2] = 't';
    SensAR.setprdinfo[3] = 'p';
    SensAR.setprdinfo[4] = 'r';
    SensAR.setprdinfo[5] = 'd';
    SensAR.setprdinfo[6] = 'i';
    SensAR.setprdinfo[7] = 'n';
    SensAR.setprdinfo[8] = 'f';
    SensAR.setprdinfo[9] = 'o';
    SensAR.setprdinfo[10] = ' ';
    SensAR.setprdinfo[11] = '"';		

    SensAR.ReadEncoderPN[15] = 0x20;
    SensAR.ReadEncoderPN[16] = 0x22;
    SensAR.ReadEncoderRev[2] = 0x20;
    SensAR.ReadEncoderRev[3] = 0x22;	
    SensAR.ReadEncoderSN[11] = 0x20;
    SensAR.ReadEncoderSN[12] = 0x22;
    SensAR.SetEncoderPN[15] = 0x20;
    SensAR.SetEncoderPN[16] = 0x22;
    SensAR.SetEncoderRev[2] = 0x20;
    SensAR.SetEncoderRev[3] = 0x22;	
    SensAR.SetEncoderSN[11] = 0x20;
    SensAR.SetEncoderSN[12] = 0x22;	
    SensAR.ReadPCBAPN[7] = 0x20;
    SensAR.ReadPCBAPN[8] = 0x22;
    SensAR.ReadPCBARev[2] = 0x20;
    SensAR.ReadPCBARev[3] = 0x22;	
    SensAR.StringCRC[0] = 0x3C;
    SensAR.StringCRC[3] = 0x3E;
    SensAR.StringCRC[4] = 0x0D;
}
/**************************************************************************************
* 函数名称：PrdInfoCollectMessage()
* 函数功能：配置PrdInfoCollectMessage
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void PrdInfoCollectMessage(void)
{
  uint8_t i;

//  for(i = 0; i < 15; i++){
//    SensAR.ReadEncoderPN[i] = SensAR.RxData[i+17];
//  }
//
//  for(i = 0; i < 2; i++){
//    SensAR.ReadEncoderRev[i] = SensAR.RxData[i+38];
//  }	
//
//  for(i = 0; i < 11; i++){
//    SensAR.ReadEncoderSN[i] = SensAR.RxData[i+46];
//  }

  for(i = 0; i < 7; i++){
    SensAR.ReadPCBAPN[i] = SensAR.RxData[(SensAR.RxDataCnt-40)+i];
  }

  for(i = 0; i < 2; i++){
    SensAR.ReadPCBARev[i] = SensAR.RxData[(SensAR.RxDataCnt-27)+i];
  }

  for(i = 0; i < 10; i++){
    SensAR.ReadPCBASN[i] = SensAR.RxData[(SensAR.RxDataCnt-19)+i];
  }	
}
/**************************************************************************************
* 函数名称：SetStringMrging()
* 函数功能：配置字符串合并
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void SetStringMrging(void)
{
  uint8_t i;
  for(i = 0; i < 12; i++){
    SensAR.SetPrdInfoData[i] = SensAR.setprdinfo[i];
  }		
  for(i = 0; i < 17; i++){
    SensAR.SetPrdInfoData[i+12] = SensAR.SetEncoderPN[i];
  }	
  for(i = 0; i < 4; i++){
    SensAR.SetPrdInfoData[i+29] = SensAR.SetEncoderRev[i];
  }	
  for(i = 0; i < 13; i++){
    SensAR.SetPrdInfoData[i+33] = SensAR.SetEncoderSN[i];
  }
  for(i = 0; i < 9; i++){
    SensAR.SetPrdInfoData[i+46] = SensAR.ReadPCBAPN[i];
  }
  for(i = 0; i < 4; i++){
    SensAR.SetPrdInfoData[i+55] = SensAR.ReadPCBARev[i];
  }
  for(i = 0; i < 10; i++){
    SensAR.SetPrdInfoData[i+59] = SensAR.ReadPCBASN[i];
  }	
}
/**************************************************************************************
* 函数名称：GetPrdInfoDataMrging()
* 函数功能：配置字符串合并
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void GetPrdInfoDataMrging(void)
{
  uint8_t i;
  for(i = 0; i < 108; i++){
    SensAR.GetPrdInfoData[i] = SensAR.RxData[i+1];
  }			
}
/**************************************************************************************
* 函数名称：GetVERMrging()
* 函数功能：配置字符串合并
* 输入参量：无
* 输出参量：无
***************************************************************************************/
void GetVERMrging(void)
{
    uint8_t i;
    for(i = 0; i < 75; i++){
      SensAR.GetVER[i] = SensAR.RxData[i+1];
    }			
}
/**************************************************************************************
* 函数名称：SensARTest()
* 函数功能：配置SensARTest
* 输入参量：p->DataSize 数据个数
* 输出参量：无
***************************************************************************************/
void SensARTest(uint8_t TestContent)
{
  if(SensAR.Flag.bit.Enter == 1){
    SensAR.CycleFlag = 0;
    EncoderPrintf("\r");
  }else{
    switch(TestContent){
      case SensARStopTest:
              
      break;
      case SensAREnterTest:
//			SensAR.TestContent = SensARStopTest;
//		  SensAR.Flag.bit.Enter = 1;
//		  EncoderPrintf("\r");
      break;
      case SensARgetprdinfoTest:		
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.getprdinfo = 1;
          SensAR.DataString = "getprdinfo";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");		
      break;
      case SensARgethallvalue1Test:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.gethallvalue1 = 1;
          SensAR.DataString = "gethallvalue 1";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");				
      break;
      case SensARgethallvalue2Test:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.gethallvalue2 = 1;
          SensAR.DataString = "gethallvalue 2";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");				
      break;
      case SensARgethallvalue3Test:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.gethallvalue3 = 1;
          SensAR.DataString = "gethallvalue 3";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");		
      break;
      case SensARgethallvalue4Test:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.gethallvalue4 = 1;
          SensAR.DataString = "gethallvalue 4";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");		
      break;
      case SensARgethallvalue5Test:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.gethallvalue5 = 1;
          SensAR.DataString = "gethallvalue 5";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");			
      break;
      case SensARgethallvalue6Test:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.gethallvalue6 = 1;
          SensAR.DataString = "gethallvalue 6";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");
      break;	
      case SensARgethallvalue7Test:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.gethallvalue7 = 1;
          SensAR.DataString = "gethallvalue 7";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");	
      break;	
      case SensARsetprdinfoTest:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.setprdinfo = 1;
          SetStringMrging();
          SensAR.DataCnt = strlen(SensAR.SetPrdInfoData);
          SensAR.CrcData = SensAR_Crc(SensAR.SetPrdInfoData, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.SetPrdInfoData, "<", SensAR.CrcData, ">", "\r");
      break;			
      case SensARverTest:
          SensAR.TestContent = SensARStopTest;
          SensAR.Flag.bit.ver = 1;
          SensAR.DataString = "ver";
          SensAR.DataCnt = strlen(SensAR.DataString);
          SensAR.CrcData = SensAR_Crc(SensAR.DataString, SensAR.DataCnt);
          EncoderPrintf("%s""%s""%02X""%s""%s", SensAR.DataString, "<", SensAR.CrcData, ">", "\r");		
      break;			
    }		
  }
}
///**************************************************************************************
//* 函数名称：HallValueCalculate
//* 函数功能：配置HallValueCalculate
//* 输入参量：ReceiveCnt 接收个数
//* 输出参量：无
//***************************************************************************************/
uint16_t HallValueCalculate(uint8_t ReceiveCnt)
{
  uint16_t HallValue = 0;
  if(ReceiveCnt >= 30){
    switch(ReceiveCnt){
      case 31:
              HallValue = (SensAR.RxData[21] - '0');
      break;
      case 32:
              HallValue = (SensAR.RxData[21] - '0') * 10 + (SensAR.RxData[22] - '0');
      break;
      case 33:
              HallValue = (SensAR.RxData[21] - '0') * 100 + (SensAR.RxData[22] - '0') * 10 + (SensAR.RxData[23] - '0');
      break;
      case 34:
              HallValue = (SensAR.RxData[21] - '0') * 1000 + (SensAR.RxData[22] - '0') * 100 + (SensAR.RxData[23] - '0') * 10 + (SensAR.RxData[24] - '0');
      break;
      case 35:
              HallValue = (SensAR.RxData[21] - '0') * 10000 + (SensAR.RxData[22] - '0') * 1000 + (SensAR.RxData[23] - '0') * 100 + (SensAR.RxData[24] - '0') * 10 + (SensAR.RxData[25] - '0');
      break;
    }
  }	
  return HallValue;
}
///**************************************************************************************
//* 函数名称：SensAR_RX
//* 函数功能：配置SensAR_RX
//* 输入参量：TestData_ID 测试标志
//* 输出参量：无
//***************************************************************************************/
void SensAR_RX(void)
{	
  SensAR.RxData[SensAR.RxDataCnt++] = UART0->D; 
  
  switch(SensAR.Flag.all){		
    case SensAREnterDataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.Flag.all = 0;			
          SensAR.RxDataCnt = 0;	
          SensAR.CycleFlag = 1;				
        }	
    break;
    case SensARgetprdinfoDataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          PrdInfoCollectMessage();
          GetPrdInfoDataMrging();
          SensAR.Flag.all = 0;
          SensAR.RxDataCnt = 0;					
        }	
    break;
    case SensARgethallvalue1DataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.Hall_1 = HallValueCalculate(SensAR.RxDataCnt);
          SensAR.Flag.all = 0;				
          SensAR.RxDataCnt = 0;	
        }					    
    break;
    case SensARgethallvalue2DataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.Hall_2 = HallValueCalculate(SensAR.RxDataCnt);
          SensAR.RxDataCnt = 0;
          SensAR.Flag.all = 0;				
        }			
    break;
    case SensARgethallvalue3DataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.Hall_3 = HallValueCalculate(SensAR.RxDataCnt);
          SensAR.RxDataCnt = 0;
          SensAR.Flag.all = 0;				
        }				
    break;
    case SensARgethallvalue4DataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.Hall_4 = HallValueCalculate(SensAR.RxDataCnt);
          SensAR.RxDataCnt = 0;
          SensAR.Flag.all = 0;				
        }					
    break;
    case SensARgethallvalue5DataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.Hall_5 = HallValueCalculate(SensAR.RxDataCnt);
          SensAR.RxDataCnt = 0;
          SensAR.Flag.all = 0;				
        }			
    break;
    case SensARgethallvalue6DataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.Hall_6 = HallValueCalculate(SensAR.RxDataCnt);
          SensAR.RxDataCnt = 0;
          SensAR.Flag.all = 0;				
        }				
    break;
    case SensARgethallvalue7DataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.Hall_7 = HallValueCalculate(SensAR.RxDataCnt);
          SensAR.RxDataCnt = 0;
          SensAR.Flag.all = 0;				
        }			
    break;	
    case SensARsetprdinfoDataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          SensAR.RxDataCnt = 0;
          SensAR.Flag.all = 0;				
        }	
    break;		
    case SensARverDataID:
        if(SensAR.RxData[SensAR.RxDataCnt-3] == '-' && SensAR.RxData[SensAR.RxDataCnt-2] == '-' && SensAR.RxData[SensAR.RxDataCnt-1] == '>'){
          GetVERMrging();
          SensAR.RxDataCnt = 0;
          SensAR.Flag.all = 0;				
        }
    break;					
    default:
      SensAR.RxDataCnt = 0;
    break;
  }
}

