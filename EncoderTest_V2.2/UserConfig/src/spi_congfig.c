#include "fsl_common.h"
#include "spi.h"
#include "smartabs.h"
#include "ioport.h"

//==============================================================================
// DEFINES
//==============================================================================
#define SPI_BaudRate   10000000U
extern volatile uint8_t   SPITxData[16], SPIRxData[16];
extern volatile strSMART  SMARTABS; 
void Init_SPI0(void)
{
    dspi_master_handle_t g_m_handle;
    dspi_master_config_t masterConfig;
    
//------------------------------------------------------------------------------ 
// Clock enabled
//------------------------------------------------------------------------------ 
    CLOCK_EnableClock(kCLOCK_PortC);  
//------------------------------------------------------------------------------ 
// SPI0 pinmux function
// PIN25--PTC4 : SPI0_PCS0 (ALT2)
// PIN26--PTC5 : SPI0_SCK (ALT2)
// PIN27--PTC6 : SPI0_SOUT (ALT2)
// PIN28--PTC7 : SPI0_SIN (ALT2)    
    
//------------------------------------------------------------------------------ 
    PORT_SetPinMux(PORTC, 4U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTC, 5U, kPORT_MuxAlt2);      
    PORT_SetPinMux(PORTC, 6U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTC, 7U, kPORT_MuxAlt2);
   
//------------------------------------------------------------------------------ 
// Configrate SPI0 for master
//------------------------------------------------------------------------------ 
    
    masterConfig.whichCtar               = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate     = SPI_BaudRate;
    masterConfig.ctarConfig.bitsPerFrame = 8U;
    masterConfig.ctarConfig.cpol         = kDSPI_ClockPolarityActiveLow;
    masterConfig.ctarConfig.cpha         = kDSPI_ClockPhaseSecondEdge;
    masterConfig.ctarConfig.direction    = kDSPI_MsbFirst;

    masterConfig.ctarConfig.pcsToSckDelayInNanoSec        = 1000000000U / SPI_BaudRate;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / SPI_BaudRate;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / SPI_BaudRate;

    masterConfig.whichPcs           = kDSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK        = false;
    masterConfig.enableRxFifoOverWrite      = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint                = kDSPI_SckToSin0Clock;
    
    DSPI_MasterInit(SPI0, &masterConfig, CLOCK_GetFreq(DSPI0_CLK_SRC));  
    DSPI_MasterTransferCreateHandle(SPI0, &g_m_handle, NULL, NULL);

//    DSPI_EnableInterrupts(SPI0, kDSPI_EndOfQueueInterruptEnable);
//    NVIC_SetPriority(SPI0_IRQn, 2);
//    EnableIRQ(SPI0_IRQn);
    

}
__ramfunc status_t MT6835_SPI_Tx(uint8_t* masterTxData, uint8_t* masterRxData, uint8_t transfer_dataSize)
{
  dspi_transfer_t masterXfer;  
  status_t ret;
     
  masterXfer.txData      = masterTxData;                                        //发给从机数据
  masterXfer.rxData      = masterRxData;                                        //从机发送过来的数据
  masterXfer.dataSize    = transfer_dataSize;                                   //从机发送数据个数                    
  masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous ;

  ret = DSPI_MasterTransferBlocking(SPI0, &masterXfer);     
  return ret;
} 
__ramfunc void SPI_Encoder(void)
{
//      if(MT6835.CAL){
//        switch(MT6835.ZERO_Flag){  
//        case 0x00:
//          SPITxData[0] = 0x60;
//          SPITxData[1] = 0x0D;
//          SPITxData[2] = 0x01;                                                    //先关闭温补
//          for(volatile int i=3;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }  
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//            MT6835.ZERO_Flag = 0x01;
//          }          
//          break;
//        case 0x01:
//          SPITxData[0] = 0x30;
//          SPITxData[1] = 0x0D;
//          for(volatile int i=2;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }               
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){ 
//            MT6835.WarmTonic_STATUS = SPIRxData[2]>>6;
//          }           
//          if(MT6835.WarmTonic_STATUS == 0x03){
//            MT6835.ZERO_Flag = 0x02;
//          }
//          break;
//        case 0x02:
//          SPITxData[0] = 0x31;
//          SPITxData[1] = 0x13;                                                    //0x113地址，最高两位，代表自校准状态                                                   
//          for(volatile int i=2;i<3;i++){
//            SPITxData[i] = 0xFF;
//          }
//          CAL_DE();                                                               //开启自校准
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//            MT6835.CAL_STATUS = SPIRxData[2]>>6;
//          }
//          if(MT6835.CAL_STATUS == 0x03){                                          //自校准完成
//            CAL_RE();
//            #if (ENCODER==Increment)           
//            MT6835.ZERO_Flag = 0x03;
//            #endif          
//          }
//          break;
//      case 0x03:
//          SPITxData[0] = 0x60;        //ZERO_POS[11:0]清零                
//          SPITxData[1] = 0x09;
//          SPITxData[2] = 0x00;     
////          for(volatile int i=3;i<16;i++){
////            SPITxData[i] = 0xFF;
////          }
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//            MT6835.ZERO_Flag = 0x04;            
//          }        
//        break; 
//      case 0x04:
//          SPITxData[0] = 0x60;        //ZERO_POS[11:0]清零                
//          SPITxData[1] = 0x0A;
//          SPITxData[2] = 0x01;     
////          for(volatile int i=3;i<16;i++){
////            SPITxData[i] = 0xFF;
////          }
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//            MT6835.ZERO_Flag = 0x05;            
//          }        
//        break; 
//      case 0x05:
//          SPITxData[0] = 0xA0;
//          SPITxData[1] = 0x03;
//          for(volatile int i=2;i<5;i++){
//            SPITxData[i] = 0xFF;
//          }          
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 6)==kStatus_Success){
//            MT6835.ZERO_Flag = 0x06;
//            MT6835.ANGLE2 = SPIRxData[2];                                       //ANGLE[20:13];
//            MT6835.ANGLE1 = SPIRxData[3];                                       //ANGLE[12:5];
//            MT6835.ANGLE0 = SPIRxData[4]>>3;                                    //ANGLE[4:0],STATUS[2:0];
//            SMARTABS.Angle = ((MT6835.ANGLE2<<13)|(MT6835.ANGLE1<<5)|MT6835.ANGLE0);
//            MT6835.Zero_Pos = (SMARTABS.Angle + 0x72B0)>>9;                     //零位寄存器清零后，读取位置角度，然后角度加上5.04°，这个偏置角度转换成12bit的角度写到零位寄存器
//          }        
//        break;
//      case 0x06:
//          SPITxData[0] = 0x60;                                                  //写偏置角度
//          SPITxData[1] = 0x09;
//          SPITxData[2] = MT6835.Zero_Pos >> 4;                                  //零位寄存器高8位                   
////          for(volatile int i=3;i<16;i++){
////            SPITxData[i] = 0xFF;
////          }               
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){  
//            MT6835.ZERO_Flag = 0x07;
//          }        
//        break;        
//      case 0x07:
//          SPITxData[0] = 0x60;                                                  //写偏置角度
//          SPITxData[1] = 0x0A;
//          SPITxData[2] = (((MT6835.Zero_Pos & 0x0F)<<4)|0x01);                  //零位寄存器低4位
////          for(volatile int i=3;i<16;i++){
////            SPITxData[i] = 0xFF;
////          }               
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){  
//              MT6835.ZERO_Flag = 0x08;
//          }       
//        break;  
//      case 0x08: 
//          SPITxData[0] = 0x60;
//          SPITxData[1] = 0x0D;
//          SPITxData[2] = 0xC1;                                                  //开启温补
//          for(volatile int i=3;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }  
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//            MT6835.ZERO_Flag = 0x01;
//          }         
//        break;
//        default:
//          break;        
//        }           
//      }
      

      
//      if(MT6835.WarmTonic){
//          SPITxData[0] = 0x60;
//          SPITxData[1] = 0x0D;
//          SPITxData[2] = 0x01;  
////          for(volatile int i=3;i<16;i++){
////            SPITxData[i] = 0xFF;
////          }  
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//            MT6835.WarmTonic = 0;
//          }        
//      }
//      
//      
//      if(MT6835.ZERO_Flag == 100){
//          MT6835.ZERO_Flag = 1;
//      }else if(MT6835.WarmTonic == 2){
//          MT6835.WarmTonic = 3;
//      }else if(MT6835.CAL){
//        SPITxData[0] = 0x31;
//        SPITxData[1] = 0x13;
//        SPIRxData[0] = 0x00;                                                    //0x113地址，最高两位，代表自校准状态
//        for(volatile int i=2;i<16;i++){
//          SPITxData[i] = 0xFF;
//        }
//        CAL_DE();                                                               //开启自校准
//        if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//          MT6835.CAL_STATUS = SPIRxData[2]>>6;
//        }
//        if(MT6835.CAL_STATUS == 0x03){
//          MT6835.CAL = 0;
//          CAL_RE();
//          MT6835.ZERO_Flag = 1;
//        }        
//      }else if(MT6835.WarmTonic == 1){
//          SPITxData[0] = 0x60;
//          SPITxData[1] = 0x0D;
//          SPITxData[2] = 0x01;  
//          for(volatile int i=3;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }  
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//            MT6835.ZERO_Flag = 100;
//            MT6835.WarmTonic = 0;
//          }          
//      }else if(MT6835.ZERO_Flag){
//          if(MT6835.ZERO_Cal == 0){
//            SPITxData[0] = 0x60;        //ZERO_POS[11:0]清零                
//            SPITxData[1] = 0x09;
//            SPITxData[2] = 0x00;     
//            for(volatile int i=3;i<16;i++){
//              SPITxData[i] = 0xFF;
//            }
//            if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//              MT6835.ZERO_Cal = 1;            
//            }
//          }else if(MT6835.ZERO_Cal == 1){
//            SPITxData[0] = 0x60;        //ZERO_POS[11:0]清零                
//            SPITxData[1] = 0x0A;
//            SPITxData[2] = 0x01;     
//            for(volatile int i=3;i<16;i++){
//              SPITxData[i] = 0xFF;
//            }
//            if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){
//              MT6835.ZERO_Cal = 2;            
//            }            
//          }else if(MT6835.ZERO_Cal == 2){
//            SPITxData[0] = 0xA0;
//            SPITxData[1] = 0x03;
//            for(volatile int i=2;i<16;i++){
//              SPITxData[i] = 0xFF;
//            }          
//            if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 6)==kStatus_Success){
//              MT6835.ZERO_Cal = 3;
//              MT6835.ANGLE2 = SPIRxData[2];                                       //ANGLE[20:13];
//              MT6835.ANGLE1 = SPIRxData[3];                                       //ANGLE[12:5];
//              MT6835.ANGLE0 = SPIRxData[4]>>3;                                    //ANGLE[4:0],STATUS[2:0];
//              SMARTABS.Angle = ((MT6835.ANGLE2<<13)|(MT6835.ANGLE1<<5)|MT6835.ANGLE0);
//              MT6835.Zero_Pos = (SMARTABS.Angle + 0x72B0)>>9;                        //零位寄存器清零后，读取位置角度，然后角度加上5.04°，这个偏置角度转换成12bit的角度写到零位寄存器
////              SMARTABS.Zero_Angle = SMARTABS.Angle>>9;
//            }
//          }else if(MT6835.ZERO_Cal == 3){
//              SPITxData[0] = 0x60;                                              //写偏置角度
//              SPITxData[1] = 0x09;
//              SPITxData[2] = MT6835.Zero_Pos >> 4;  
//              for(volatile int i=3;i<16;i++){
//                SPITxData[i] = 0xFF;
//              }               
//              if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){  
//                MT6835.ZERO_Cal = 4;
//              }
//          }else if(MT6835.ZERO_Cal == 4){
//              SPITxData[0] = 0x60;                                              //写偏置角度
//              SPITxData[1] = 0x0A;
//              SPITxData[2] = (((MT6835.Zero_Pos & 0x0F)<<4)|0x01);  
//              for(volatile int i=3;i<16;i++){
//                SPITxData[i] = 0xFF;
//              }               
//              if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){  
//                  MT6835.ZERO_Cal = 0;
//                  MT6835.ZERO_Flag = 0;
//                  MT6835.WarmTonic = 2;
//              }            
//          }
//      }else if(MT6835.EEPROM == 2){
//          SPITxData[0] = 0x31;
//          SPITxData[1] = 0x12; 
//          for(volatile int i=2;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){ 
////            MT6838cnt ++;
//            MT6835.NVM_STA = SPIRxData[2]>>4;
//            if(MT6835.NVM_STA == 1){
//              MT6835.EEPROM = 0;
//              MT6835.NVM_STA = 0;
//              SPITxData[0] = 0x30;
//              SPITxData[1] = 0x0D;
//              for(volatile int i=2;i<16;i++){
//                SPITxData[i] = 0xFF;
//              }               
//              if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){ 
//                MT6835.WarmTonic_STATUS = SPIRxData[2]>>6;
//              }              
// //             MT6835.WarmTonic = 1;
//            }
//          }           
//          
//      }else if(MT6835.WarmTonic == 3){
////          SPITxData[0] = 0x30;
////          SPITxData[1] = 0x0D;
////          for(volatile int i=2;i<16;i++){
////            SPITxData[i] = 0xFF;
////          }               
////          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){ 
////            MT6835.WarmTonic_STATUS = SPIRxData[3]>>6;
////          }
//          SPITxData[0] = 0x60;
//          SPITxData[1] = 0x0D;
//          SPITxData[2] = 0xC1;  
//          for(volatile int i=3;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }               
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){  
//              if(MT6835.EEPROM == 0){
//                MT6835.EEPROM = 1;
//                SPITxData[0] = 0xC0;                                              //写EEPROM
//                SPITxData[1] = 0x00; 
//                for(volatile int i=2;i<16;i++){
//                  SPITxData[i] = 0xFF;
//                }              
//                if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){            
//                  if(SPIRxData[2] == 0x55){
//                    MT6835.EEPROM = 2;
//                    MT6835.WarmTonic = 0;
//                  }
//                }            
//            }                        
//          }        
//      }else if(MT6835.Z_STATUS == 1){
//          SPITxData[0] = 0x30;
//          SPITxData[1] = 0x0A;
//          for(volatile int i=2;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }               
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){  
//            MT6835.Z_STATUS = 2;
//            MT6835.Z_EDGE = (SPIRxData[2]&0x08)>>3;
//            MT6835.Z_PUL_WID = SPIRxData[2]&0x07;  
//          }        
//      }else if(MT6835.Z_STATUS == 2){
//          SPITxData[0] = 0x30;
//          SPITxData[1] = 0x0B;
//          for(volatile int i=2;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }               
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){  
//            MT6835.Z_STATUS = 0;
//            MT6835.Z_PHASE = SPIRxData[2]>>6;  
//          }        
//      }else if(MT6835.Z_STATUS == 3){
//          SPITxData[0] = 0x60;
//          SPITxData[1] = 0x0B;
//          SPITxData[2] = 0x44;
//          for(volatile int i=3;i<16;i++){
//            SPITxData[i] = 0xFF;
//          }               
//          if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){  
//            if(MT6835.EEPROM == 0){
//              MT6835.EEPROM = 1;
//              SPITxData[0] = 0xC0;                                              //写EEPROM
//              SPITxData[1] = 0x00; 
//              for(volatile int i=2;i<16;i++){
//                SPITxData[i] = 0xFF;
//              }              
//              if(MT6835_SPI_Tx((uint8_t *)SPITxData, (uint8_t *)SPIRxData, 3)==kStatus_Success){            
//                if(SPIRxData[2] == 0x55){
//                  MT6835.Z_STATUS = 0;
//                  MT6835.EEPROM = 2;
//                }
//              }            
//          }         
//        }
//      }  
}