#ifndef _ADC_H_
#define _ADC_H_

#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_ftm.h"
#include "fsl_adc16.h"
#include "fsl_pdb.h"


//==============================================================================
// Definitions
//==============================================================================
//#define Period          (62)                                                    //100us
//#define ConverTime      (62)//(47 + 7 + 8)                                      //16 Conver Time (46.6us) + Calculate Time (6.8us) + Idle Time (6us)
//#define ConverTime    (39)//(24 + 7 + 8)                                      // 8 Conver Time (24.0us) + Calculate Time (6.8us) + Idle Time (6us)
#define ModulusValue    (1000 * 48)                                             //1ms = 1000*48MHz
#define DelayValue      (40 * 48)                                               //Request -> 50us -> 485RE, min delay is 48 * 48   
#define TiggerValue     0//((Period - ConverTime) * 48)


//==============================================================================
// Functions
//==============================================================================
void Init_ADC0_1(void);
void Init_PDB0(void);


#endif /* _ADC_H_ */
