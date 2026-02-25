#ifndef _GPIO_CONFIG_H_
#define _GPIO_CONFIG_H_

#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

//==============================================================================
// Definitions
//==============================================================================
#define DO1L()                          (GPIOD->PCOR = 0x00000008)
#define DO1H()                          (GPIOD->PSOR = 0x00000008)
#define DO2L()                          (GPIOD->PCOR = 0x00000010)
#define DO2H()                          (GPIOD->PSOR = 0x00000010)
#define DO3L()                          (GPIOD->PCOR = 0x00000020)
#define DO3H()                          (GPIOD->PSOR = 0x00000020)
#define DO4L()                          (GPIOD->PCOR = 0x00000040)
#define DO4H()                          (GPIOD->PSOR = 0x00000040)
#define DO5L()                          (GPIOD->PCOR = 0x00000080)
#define DO5H()                          (GPIOD->PSOR = 0x00000080)
#define DO6L()                          (GPIOE->PCOR = 0x01000000)
#define DO6H()                          (GPIOE->PSOR = 0x01000000)
#define Ddig2L()                        (GPIOB->PCOR = 0x00010000)
#define Ddig2H()                        (GPIOB->PSOR = 0x00010000)
#define Ddig1L()                        (GPIOB->PCOR = 0x00020000)
#define Ddig1H()                        (GPIOB->PSOR = 0x00020000)
#define DisplayClear()                  (GPIOC->PCOR = 0x000000FF)
#define DaL()                           (GPIOC->PCOR = 0x00000001)
#define DaH()                           (GPIOC->PSOR = 0x00000001)
#define DbL()                           (GPIOC->PCOR = 0x00000002)
#define DbH()                           (GPIOC->PSOR = 0x00000002)
#define DcL()                           (GPIOC->PCOR = 0x00000004)
#define DcH()                           (GPIOC->PSOR = 0x00000004)
#define DdL()                           (GPIOC->PCOR = 0x00000008)
#define DdH()                           (GPIOC->PSOR = 0x00000008)
#define DeL()                           (GPIOC->PCOR = 0x00000010)
#define DeH()                           (GPIOC->PSOR = 0x00000010)
#define DfL()                           (GPIOC->PCOR = 0x00000020)
#define DfH()                           (GPIOC->PSOR = 0x00000020)
#define DgL()                           (GPIOC->PCOR = 0x00000040)
#define DgH()                           (GPIOC->PSOR = 0x00000040)
#define DdpL()                          (GPIOC->PCOR = 0x00000080)
#define DdpH()                          (GPIOC->PSOR = 0x00000080)
#define Display0()                      (GPIOC->PSOR = 0x0000003F)
#define Display1()                      (GPIOC->PSOR = 0x00000006)
#define Display2()                      (GPIOC->PSOR = 0x0000005B)
#define Display3()                      (GPIOC->PSOR = 0x0000004F)
#define Display4()                      (GPIOC->PSOR = 0x00000066)
#define Display5()                      (GPIOC->PSOR = 0x0000006D)
#define Display6()                      (GPIOC->PSOR = 0x0000007D)
#define Display7()                      (GPIOC->PSOR = 0x00000007)
#define Display8()                      (GPIOC->PSOR = 0x0000007F)
#define Display9()                      (GPIOC->PSOR = 0x0000006F)
#define DisplayA()                      (GPIOC->PSOR = 0x00000077)
#define DisplayB()                      (GPIOC->PSOR = 0x0000007C)
#define DisplayC()                      (GPIOC->PSOR = 0x00000039)
#define DisplayD()                      (GPIOC->PSOR = 0x0000005E)
#define DisplayE()                      (GPIOC->PSOR = 0x00000079)
#define DisplayF()                      (GPIOC->PSOR = 0x00000071)
#define DisplayH()                      (GPIOC->PSOR = 0x00000076)
#define DisplayL()                      (GPIOC->PSOR = 0x00000038)
#define DisplayP()                      (GPIOC->PSOR = 0x00000073)
#define DisplayO()                      (GPIOC->PSOR = 0x0000005C)

#define Display0Dot()                   (GPIOC->PSOR = 0x000000BF)
#define Display1Dot()                   (GPIOC->PSOR = 0x00000086)
#define Display2Dot()                   (GPIOC->PSOR = 0x000000DB)
#define Display3Dot()                   (GPIOC->PSOR = 0x000000CF)
#define Display4Dot()                   (GPIOC->PSOR = 0x000000E6)
#define Display5Dot()                   (GPIOC->PSOR = 0x000000ED)
#define Display6Dot()                   (GPIOC->PSOR = 0x000000FD)
#define Display7Dot()                   (GPIOC->PSOR = 0x00000087)
#define Display8Dot()                   (GPIOC->PSOR = 0x000000FF)
#define Display9Dot()                   (GPIOC->PSOR = 0x000000EF)
#define DisplayADot()                   (GPIOC->PSOR = 0x000000F7)
#define DisplayBDot()                   (GPIOC->PSOR = 0x000000FC)
#define DisplayCDot()                   (GPIOC->PSOR = 0x000000B9)
#define DisplayDDot()                   (GPIOC->PSOR = 0x000000DE)
#define DisplayEDot()                   (GPIOC->PSOR = 0x000000F9)
#define DisplayFDot()                   (GPIOC->PSOR = 0x000000F1)
#define DisplayHDot()                   (GPIOC->PSOR = 0x000000F6)
#define DisplayLDot()                   (GPIOC->PSOR = 0x000000B8)
#define DisplayPDot()                   (GPIOC->PSOR = 0x000000F3)
#define DisplayODot()                   (GPIOC->PSOR = 0x000000DC)



#define EncoderPowerOn()             DO6H()
#define EncoderPowerOff()            DO6L()
//==============================================================================
// DEFINES
//==============================================================================   
//#define READ_DI1()                 (GPIOE->PDIR & 0x80000) 
//#define READ_DI2()                 (GPIOE->PDIR & 0x80000) 
//#define READ_DI3()                 (GPIOE->PDIR & 0x0004) 
//#define READ_DI4()                 (GPIOE->PDIR & 0x0008) 
//==============================================================================
// Functions
//==============================================================================
void Init_GPIO(void);
void IOConnected(void);
void IODisconnected(void);
#endif /* _GPIO_CONFIG_H_ */
