#include "fsl_common.h"
#include "gpio_config.h"




//==============================================================================
// DATA
//==============================================================================
extern volatile uint8_t  Enc_PowerOn_Flag;

//==============================================================================
// CODE
//==============================================================================
void Init_GPIO(void)
{
//------------------------------------------------------------------------------ 
// Clock enabled
//------------------------------------------------------------------------------ 
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);
    
//------------------------------------------------------------------------------ 
// INPUT
// PIN03--PTE16 : DI1
// PIN04--PTE17 : DI2    
// PIN05--PTE18 : DI3
// PIN06--PTE19 : DI4    
//------------------------------------------------------------------------------ 
//    PORT_SetPinMux(PORTE, 16U, kPORT_MuxAsGpio);
//    PORT_SetPinMux(PORTE, 17U, kPORT_MuxAsGpio);    
//    PORT_SetPinMux(PORTE, 18U, kPORT_MuxAsGpio);    
//    PORT_SetPinMux(PORTE, 19U, kPORT_MuxAsGpio);    
//    GPIOE->PDDR &= 0xFFFEFFFF;                                                  //Pin is configured as general-purpose input    
//    GPIOE->PDDR &= 0xFFFDFFFF;                                                  //Pin is configured as general-purpose input
//    GPIOE->PDDR &= 0xFFFBFFFF;                                                  //Pin is configured as general-purpose input
//    GPIOE->PDDR &= 0xFFF7FFFF;                                                  //Pin is configured as general-purpose input
    const port_pin_config_t DI_config = {/* Internal pull-up resistor is enabled */
                                                   kPORT_PullUp,
                                                   /* Fast slew rate is configured */
                                                   kPORT_FastSlewRate,
                                                   /* Passive filter is disabled */
                                                   kPORT_PassiveFilterDisable,
                                                   /* Open drain is disabled */
                                                   kPORT_OpenDrainDisable,
                                                   /* Low drive strength is configured */
                                                   kPORT_LowDriveStrength,
                                                   /* Pin is configured as PTA4 */
                                                   kPORT_MuxAsGpio,
                                                   /* Pin Control Register fields [15:0] are not locked */
                                                   kPORT_UnlockRegister};
    PORT_SetPinConfig(PORTE, 16U, &DI_config);
    PORT_SetPinConfig(PORTE, 17U, &DI_config);
    PORT_SetPinConfig(PORTE, 18U, &DI_config);
    PORT_SetPinConfig(PORTE, 19U, &DI_config);   
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
    };
    GPIO_PinInit(GPIOE, 16U, &sw_config);  
    GPIO_PinInit(GPIOE, 17U, &sw_config);
    GPIO_PinInit(GPIOE, 18U, &sw_config);
    GPIO_PinInit(GPIOE, 19U, &sw_config);

// OUTPUT 
// PIN47--PTD6 : DO1
// PIN48--PTD7 : DO2    
// PIN15--PTE24 : DO3    
    
// PIN31--PTB16 : Ddig2
// PIN32--PTB17 : Ddig1
// PIN33--PTC0 : Da
// PIN34--PTC1 : Db 
// PIN35--PTC2 : Dc  
// PIN36--PTC3 : Dd    
// PIN37--PTC4 : De 
// PIN38--PTC5 : Df
// PIN39--PTC6 : Dg
// PIN40--PTC7 : Ddp       
    
//------------------------------------------------------------------------------ 
    PORT_SetPinMux(PORTD, 3U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTD, 4U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTD, 5U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTD, 6U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTD, 7U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTE, 24U, kPORT_MuxAsGpio);

    
    PORT_SetPinMux(PORTB, 16U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTB, 17U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 1U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 2U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 3U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 4U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 5U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 6U, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTC, 7U, kPORT_MuxAsGpio);
    
//    GPIOD->PDDR |= 0x00000040;                                                  //Pin is configured as general-purpose output
//    GPIOD->PDDR |= 0x00000080;                                                  //Pin is configured as general-purpose output
//    GPIOE->PDDR |= 0x01000000;                                                  //Pin is configured as general-purpose output
//
//    GPIOB->PDDR |= 0x00010000;                                                  //Pin is configured as general-purpose output
//    GPIOB->PDDR |= 0x00020000;                                                  //Pin is configured as general-purpose output
//    GPIOC->PDDR |= 0x00000001;                                                  //Pin is configured as general-purpose output
//    GPIOC->PDDR |= 0x00000002;                                                  //Pin is configured as general-purpose output
//    GPIOC->PDDR |= 0x00000004;                                                  //Pin is configured as general-purpose output
//    GPIOC->PDDR |= 0x00000008;                                                  //Pin is configured as general-purpose output
//    GPIOC->PDDR |= 0x00000010;                                                  //Pin is configured as general-purpose output
//    GPIOC->PDDR |= 0x00000020;                                                  //Pin is configured as general-purpose output    
//    GPIOC->PDDR |= 0x00000040;                                                  //Pin is configured as general-purpose output    
//    GPIOC->PDDR |= 0x00000080;                                                  //Pin is configured as general-purpose output    

    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };
    GPIO_PinInit(GPIOD, 3U, &led_config);   
    GPIO_PinInit(GPIOD, 4U, &led_config);   
    GPIO_PinInit(GPIOD, 5U, &led_config);     
    GPIO_PinInit(GPIOD, 6U, &led_config);   
    GPIO_PinInit(GPIOD, 7U, &led_config);
    GPIO_PinInit(GPIOE, 24U, &led_config);
    GPIO_PinInit(GPIOB, 16U, &led_config);
    GPIO_PinInit(GPIOB, 17U, &led_config);
    GPIO_PinInit(GPIOC, 0U, &led_config);
    GPIO_PinInit(GPIOC, 1U, &led_config);
    GPIO_PinInit(GPIOC, 2U, &led_config);
    GPIO_PinInit(GPIOC, 3U, &led_config);
    GPIO_PinInit(GPIOC, 4U, &led_config);
    GPIO_PinInit(GPIOC, 5U, &led_config);
    GPIO_PinInit(GPIOC, 6U, &led_config);
    GPIO_PinInit(GPIOC, 7U, &led_config);
    DO1L();                                                                      //默认低电平，
    DO2L();                                                                      //默认低电平，
    DO3L();                                                                      //默认低电平，
    DO4L();                                                                      //默认低电平，
    DO5L();                                                                      //默认低电平，
    DO6L();                                                                      //默认低电平，
//    Enc_PowerOn_Flag = 1;
//    SMARTABS.FlagPit = 1;
    PORT_SetPinInterruptConfig(PORTE, 16U, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(PORTE, 17U, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(PORTE, 18U, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(PORTE, 19U, kPORT_InterruptFallingEdge);
    NVIC_SetPriority(PORTE_IRQn, 0);
    EnableIRQ(PORTE_IRQn);       
         
    
}
void IOConnected(void)
{
    DO1H();//
    DO2H();//    
    DO3H();//    
    DO4H();//
    DO5H();//  
}
void IODisconnected(void)
{
    DO1L();//
    DO2L();//    
    DO3L();//    
    DO4L();//
    DO5L();//    
}

