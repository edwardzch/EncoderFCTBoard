#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "gpio_config.h"
#include "pit_config.h"
#include "uart_config.h"
#include "Function.h"
#include "MGTMagneticEncoder.h"
#include "NXPMagneticEncoder.h"
#include "MultiturnMagneticEncoder.h"
#include "MultiturnOpticalEncoder.h"
#include "TamagawaEncoder.h"
#include "SensAREncoder.h"
#include "ModBus.h"
//==============================================================================
// DEFINES
//==============================================================================
//#define POWERON_STANDBY_TIME 					1800                                      //1.8s

//==============================================================================
// DATA
//==============================================================================
void Bubble_Sort(void);
strServoMotor     Motor = {0};
strServoEncoder   MotorEncoder = {0};
strDisplay        Display = {0};
strMT6835Addr     MT6835Addr = {0};
uart_transfer_t   uart0_transfer;

volatile uint8_t   Variable_Reset_Flag = 0;


extern volatile uint8_t  Work_Alarm;
extern volatile uint32_t PITCnt;
extern volatile uint8_t  PCBAFctResult;
extern volatile uint8_t  Variable_ReadReset_Flag;
extern volatile uint8_t  Enc_PowerOn_Flag;
//==============================================================================
// UART_SendDMA0
//==============================================================================
__ramfunc void UART_SendDMA0(uart_transfer_t * p)
{
    DMA0->TCD[0].SADDR = (uint32_t)p->data;                                     //DMA source addr
    DMA0->TCD[0].DADDR = (uint32_t)&UART0->D;                                   //DMA dest   addr
    DMA0->TCD[0].CITER_ELINKNO = p->dataSize;
    DMA0->TCD[0].BITER_ELINKNO = p->dataSize;
    DMA0->ERQ |= DMA_ERQ_ERQ0_MASK;
}
/**************************************************************************************
* 函数名称：MyEncoderPrintf
* 函数功能：配置MyEncoderPrintf
* 输入参量：*format 发送的字符串内容
* 输出参量：uCRC CRC值
***************************************************************************************/
void EncoderPrintf(const char *format,...)
{
    uint32_t length;
    static char EncoderSendBuff[200] = {0};
    va_list args;

    va_start(args, format);
    length = vsnprintf((char*)EncoderSendBuff, sizeof(EncoderSendBuff)+1, (char*)format, args);
    va_end(args);
    
    Encoder485DE();
    
    DMA0->TCD[0].SADDR = (uint32_t)EncoderSendBuff;                             //DMA source addr
    DMA0->TCD[0].DADDR = (uint32_t)&UART0->D;                                   //DMA dest   addr
    DMA0->TCD[0].CITER_ELINKNO = length;
    DMA0->TCD[0].BITER_ELINKNO = length;
    DMA0->ERQ |= DMA_ERQ_ERQ0_MASK;
}
//==============================================================================
// UART_ReceiveDMA0
//==============================================================================
__ramfunc void UART_ReceiveDMA0(uart_transfer_t * p)
{
    DMA0->TCD[3].SADDR = (uint32_t)&UART0->D;                                   //DMA source addr
    DMA0->TCD[3].DADDR = (uint32_t)p->data;                                     //DMA dest   addr
    DMA0->TCD[3].CITER_ELINKNO = p->dataSize;
    DMA0->TCD[3].BITER_ELINKNO = p->dataSize;
    DMA0->ERQ |= DMA_ERQ_ERQ3_MASK;
}

//==============================================================================
// Calculate CRC8 function
//==============================================================================
__ramfunc uint8_t CRC8_Check(volatile uint8_t * Data, uint16_t Len)
{  
    uint8_t uCRC = 0;  
          
    while(Len--){  
                  uCRC ^= *Data++;  
    }  

    return uCRC;  
}  
void  Variable_Reset(void)
{
  if(Variable_Reset_Flag){
    Variable_ReadReset_Flag = 0;
    Variable_Reset_Flag = 0;
    Work_Alarm = 0;
    PITCnt = 0;    
    SerialPort485RE();
    Encoder485RE();
    memset(&PNH2612, 0, sizeof(strICPNH2612));
    memset(&TamagawaEncoder, 0, sizeof(strTmgwEnc));
    memset(&SensAR, 0, sizeof(strSensAR));
    memset(&MultiturnMagneticEncoder, 0, sizeof(strMULMGTEnc));
    memset(&NxpMagneticEncoder, 0, sizeof(strNXPEnc));
    memset(&MgtMagneticEncoder, 0, sizeof(strMGTEnc)); 
    memset(&MultiturnOpticalEncoder, 0, sizeof(strMulOptEnc)); 
    memset(&MotorEncoder, 0, sizeof(strServoEncoder)); 
    memset(&Motor, 0, sizeof(strServoMotor));       
    memset(&ModBus, 0, sizeof(strMod)); 
//    PIT_StopTimer(PIT, kPIT_Chnl_0);   
  }

}
                      


void Motor_SpeedTest(volatile uint8_t Encoder_ID, volatile uint32_t SingleTurnPosition)
{

    if(Motor.StartTime < 50){
      Motor.StartTime ++;
      Motor.DifBuffer[1] = Motor.DifBuffer[0];
      Motor.DifBuffer[0] = SingleTurnPosition; 
      Motor.Difference = Motor.DifBuffer[0] - Motor.DifBuffer[1];
    }else{
      Motor.DifBuffer[1] = Motor.DifBuffer[0];
      Motor.DifBuffer[0] = SingleTurnPosition; 
      Motor.Difference = Motor.DifBuffer[0] - Motor.DifBuffer[1];    
      if(Encoder_ID == 17){
        if(Motor.Difference < -65536){
          Motor.Difference += 131072;
        }else if(Motor.Difference > 65536){
          Motor.Difference = 131072 - Motor.Difference;
        }
        if(Motor.EncoderType >= MGTMagneticEncoder){
          Motor.ActualSpeed = (Motor.Difference * 1875) >> 9;//Motor_Spd = (Motor.Difference >> 17) / (125/60000000);        
        }else{
          Motor.ActualSpeed = (Motor.Difference * 1875) >> 12;//Motor_Spd = (Motor.Difference >> 17) / (125/60000000);        
        }
      }else if(Encoder_ID == 23){
        if(Motor.Difference < -4194304){
          Motor.Difference += 8388608;
        }else if(Motor.Difference > 4194304){
          Motor.Difference = 8388608 - Motor.Difference;
        }
        if(Motor.EncoderType >= MGTMagneticEncoder){
          Motor.ActualSpeed = (Motor.Difference * 1875) >> 15;//Motor_Spd = (Motor.Difference >> 23) / (62.5/60000000);          
        }else{
          Motor.ActualSpeed = (Motor.Difference * 1875) >> 18;//Motor_Spd = (Motor.Difference >> 23) / (62.5/60000000);          
        }
      }
      if(MotorEncoder.SamplingCycleCnt < MotorEncoder.SamplingCycle){
        MotorEncoder.SamplingCycleCnt ++;
      }else{
        MotorEncoder.SamplingCycleCnt = 0;
        Motor.SpeedBuffer[BufferCnt-1] = Motor.ActualSpeed;
        for(volatile uint8_t i=0; i<(BufferCnt-1); i++){
          Motor.SpeedBuffer[i] = Motor.SpeedBuffer[i+1];
  //            Motor.AllData += Motor_SpdBuf[i];
        }
        if(Motor.Cnt <= BufferCnt){
          Motor.Cnt ++;
        }else{
          Bubble_Sort();
          Motor.FilterSpeed = (Motor.SpeedBuffer[Check_Cnt-1] + Motor.SpeedBuffer[Check_Cnt]) >> 1;
          //Motor_SpdBuf_All = 0;
        }        
      }

    }        
}

void Bubble_Sort(void)
{          
   for(int i=0;i<(BufferCnt-1);i++)
   {//冒泡
        //相连两个数的索引是利用内部循环
        for(int index=0;index<(BufferCnt-1-i);index++)
        {
             //排序 
             if(Motor.SpeedBuffer[index]>Motor.SpeedBuffer[index+1])
             {
                 int temp=Motor.SpeedBuffer[index];
                 Motor.SpeedBuffer[index]=Motor.SpeedBuffer[index+1];
                 Motor.SpeedBuffer[index+1]=temp;
             }
        } 
   } 
}


