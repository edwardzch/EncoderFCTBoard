#include "fsl_common.h"
#include "adc.h"


//==============================================================================
// Variables
//==============================================================================


//==============================================================================
// Code
//==============================================================================
//void Init_ADC0_1(void)
//{                    
//    adc16_config_t adc16_config;
//    adc16_channel_config_t adc16_channel_config; 
//  
////------------------------------------------------------------------------------ 
//// Clock Enabled
////------------------------------------------------------------------------------ 
//    CLOCK_EnableClock(kCLOCK_Adc0);
//    CLOCK_EnableClock(kCLOCK_Adc1);  
//
////------------------------------------------------------------------------------ 
//// ADC0, ADC1 pinmux function
//// PIN03--PTE16 : ADC0_DP1
//// PIN04--PTE17 : ADC0_DM1
//// PIN05--PTE18 : ADC1_DP1
//// PIN06--PTE19 : ADC1_DM1
////------------------------------------------------------------------------------ 
//    PORT_SetPinMux(PORTE, 16U, kPORT_PinDisabledOrAnalog);
//    PORT_SetPinMux(PORTE, 17U, kPORT_PinDisabledOrAnalog);
//    PORT_SetPinMux(PORTE, 18U, kPORT_PinDisabledOrAnalog);
//    PORT_SetPinMux(PORTE, 19U, kPORT_PinDisabledOrAnalog);
//
////------------------------------------------------------------------------------ 
//// ADC0&1 Init
////------------------------------------------------------------------------------ 
//    adc16_config.referenceVoltageSource     = kADC16_ReferenceVoltageSourceVref;//For external pins pair of VrefH and VrefL.
//    adc16_config.clockSource                = kADC16_ClockSourceAlt0;           //Use bus clock
//    adc16_config.enableAsynchronousClock    = false;
//    adc16_config.clockDivider               = kADC16_ClockDivider4;             //ADC clock = 48/4 = 12MHz
//    adc16_config.resolution                 = kADC16_ResolutionDF16Bit;         //Differential 16 bits mode
//    adc16_config.longSampleMode             = kADC16_LongSampleDisabled;
//    adc16_config.enableHighSpeed            = false;
//    adc16_config.enableLowPower             = false;
//    adc16_config.enableContinuousConversion = false;
//    ADC16_Init(ADC0, &adc16_config);
//    ADC16_Init(ADC1, &adc16_config);
//
//    ADC16_EnableHardwareTrigger(ADC0, true);                                    // Enable hardware trigger
//    ADC16_SetHardwareAverage(ADC0,kADC16_HardwareAverageCount16);               // Enable hardware 32 samples averaged 
//    ADC16_EnableHardwareTrigger(ADC1, true); 
//    ADC16_SetHardwareAverage(ADC1,kADC16_HardwareAverageCount16);
//  
//    //Manual compensation 
//    ADC0->OFS = 0;
//    ADC1->OFS = 0;
//  
//    //ADC16_DoAutoCalibration(ADC0);     
//    //ADC16_DoAutoCalibration(ADC1);
//
////------------------------------------------------------------------------------ 
//// ADC0&1 Group Configuration
////------------------------------------------------------------------------------ 
//    //ADC0 Channel Group 0
//    adc16_channel_config.channelNumber = 1;                                     //ADC0->Channel 1 is COS signal
//    adc16_channel_config.enableInterruptOnConversionCompleted = true;           //Enable interrupt on Conversion Completed 
//    adc16_channel_config.enableDifferentialConversion = true;                   //differential 16-bit mode
//    ADC16_SetChannelConfig(ADC0, 0, &adc16_channel_config);
//    //ADC1 Channel Group 0  
//    adc16_channel_config.channelNumber = 1;                                     //ADC1->Channel 1 is SIN signal
//    adc16_channel_config.enableInterruptOnConversionCompleted = true;           //Enable interrupt on Conversion Completed
//    adc16_channel_config.enableDifferentialConversion = true;                   //differential 16-bit mode
//    ADC16_SetChannelConfig(ADC1, 0, &adc16_channel_config);
//
////------------------------------------------------------------------------------ 
//// Enable Interrupt
////------------------------------------------------------------------------------ 
//    NVIC_SetPriority(ADC0_IRQn, 2);
//    EnableIRQ(ADC0_IRQn);
//    NVIC_SetPriority(ADC1_IRQn, 2);
//    EnableIRQ(ADC1_IRQn);                                                       //ADC0 interrupt occurs before ADC1 interrupt
//}
//
//
void Init_PDB0(void)
{                    
    pdb_config_t pdb_config;
//    pdb_adc_pretrigger_config_t pdb_adc_pretrigger_config; 

//------------------------------------------------------------------------------ 
// Clock Enabled
//------------------------------------------------------------------------------ 
    CLOCK_EnableClock(kCLOCK_Pdb0);  

//------------------------------------------------------------------------------ 
// PDB0 Init
//------------------------------------------------------------------------------ 
    pdb_config.loadValueMode = kPDB_LoadValueImmediately;
    pdb_config.prescalerDivider = kPDB_PrescalerDivider1;
    pdb_config.dividerMultiplicationFactor = kPDB_DividerMultiplicationFactor1;
    pdb_config.triggerInputSource = kPDB_TriggerSoftware;
    pdb_config.enableContinuousMode = false;
    PDB_Init(PDB0, &pdb_config);
    
//------------------------------------------------------------------------------ 
// Configure the PDB0 period, PDB0 clock = 48MHz
//------------------------------------------------------------------------------ 
    PDB_SetModulusValue(PDB0, ModulusValue);                                    //Config automatic clear time, 1ms
//    PDB_SetCounterDelayValue(PDB0, DelayValue);                                 //Config PDB interupt time, delay 50us
//    PDB_EnableInterrupts(PDB0, kPDB_DelayInterruptEnable);                      //Enable delay interrupt 
  
//------------------------------------------------------------------------------ 
// Configure the PDB pretrigger
//------------------------------------------------------------------------------ 
//    pdb_adc_pretrigger_config.enablePreTriggerMask = true;
//    pdb_adc_pretrigger_config.enableOutputMask = true;
//    pdb_adc_pretrigger_config.enableBackToBackOperationMask = false;
//    PDB_SetADCPreTriggerConfig(PDB0, kPDB_ADCTriggerChannel0, &pdb_adc_pretrigger_config); //PDB0->Channel 0 for ADC0
//    PDB_SetADCPreTriggerConfig(PDB0, kPDB_ADCTriggerChannel1, &pdb_adc_pretrigger_config); //PDB0->Channel 1 for ADC1
    
//    PDB_SetADCPreTriggerDelayValue(PDB0, kPDB_ADCTriggerChannel0, kPDB_ADCPreTrigger0, 0);
//    PDB_SetADCPreTriggerDelayValue(PDB0, kPDB_ADCTriggerChannel1, kPDB_ADCPreTrigger0, 0);
    
//    PDB_DoLoadValues(PDB0);

//------------------------------------------------------------------------------ 
// Enable Interrupt
//------------------------------------------------------------------------------ 
    PDB_EnableInterrupts(PDB0, kPDB_DelayInterruptEnable);                      //Enable delay interrupt 
    NVIC_SetPriority(PDB0_IRQn, 3);
    EnableIRQ(PDB0_IRQn);   
}

