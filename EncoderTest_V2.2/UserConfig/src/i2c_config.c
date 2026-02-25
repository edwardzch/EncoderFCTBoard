#include "fsl_common.h"
#include "eeprom.h"


//==============================================================================
// DEFINES
//==============================================================================
#define I2C_BaudRate   100000U


//==============================================================================
// DATA
//==============================================================================


//==============================================================================
// CODE
//==============================================================================
void Init_I2C(void)
{
    port_pin_config_t port_pin_config;
    i2c_master_config_t i2c_master_config;
  
//------------------------------------------------------------------------------ 
// Clock enabled
//------------------------------------------------------------------------------ 
    CLOCK_EnableClock(kCLOCK_PortE);

//------------------------------------------------------------------------------ 
// I2C0 pinmux function
// PIN10--PTE24 : I2C0_SCL (ALT5)
// PIN11--PTE25 : I2C0_SDA (ALT5)
//------------------------------------------------------------------------------ 
    PORT_SetPinMux(PORTE, 24U, kPORT_MuxAlt5);
    PORT_SetPinMux(PORTE, 25U, kPORT_MuxAlt5);    
    
    // PTE24 (pin 10) is configured as I2C0_SCL
    port_pin_config.pullSelect = kPORT_PullUp;                                  //Internal pull-up resistor is enabled
    port_pin_config.slewRate = kPORT_FastSlewRate;                              //Fast slew rate is configured
    port_pin_config.passiveFilterEnable = kPORT_PassiveFilterDisable;           //Passive filter is disabled
    port_pin_config.openDrainEnable = kPORT_OpenDrainEnable;                    //Open drain is enabled
    port_pin_config.driveStrength =kPORT_LowDriveStrength;                      //Low drive strength is configured
    port_pin_config.mux = kPORT_MuxAlt5;                                        //Pin is configured as I2C0_SCL
    port_pin_config.lockRegister = kPORT_UnlockRegister;                        //Pin Control Register fields [15:0] are not locked
    PORT_SetPinConfig(PORTE, 24U, &port_pin_config);
    
    // PTE25 (pin 11) is configured as I2C0_SDA 
    port_pin_config.pullSelect = kPORT_PullDisable;                             //Internal pull-up resistor is enabled
    port_pin_config.slewRate = kPORT_FastSlewRate;                              //Fast slew rate is configured
    port_pin_config.passiveFilterEnable = kPORT_PassiveFilterDisable;           //Passive filter is disabled
    port_pin_config.openDrainEnable = kPORT_OpenDrainEnable;                    //Open drain is enabled
    port_pin_config.driveStrength = kPORT_LowDriveStrength;                     //Low drive strength is configured
    port_pin_config.mux = kPORT_MuxAlt5;                                        //Pin is configured as I2C0_SCL
    port_pin_config.lockRegister = kPORT_UnlockRegister;                        //Pin Control Register fields [15:0] are not locked
    PORT_SetPinConfig(PORTE, 25U, &port_pin_config);    

//------------------------------------------------------------------------------ 
// Configrate I2C0 for master
//------------------------------------------------------------------------------ 
    I2C_MasterGetDefaultConfig(&i2c_master_config);  
    i2c_master_config.baudRate_Bps = I2C_BaudRate;                              //100KBps
    I2C_MasterInit(I2C0, &i2c_master_config, CLOCK_GetFreq(I2C0_CLK_SRC));  
}

__ramfunc status_t I2C_WR_AT24LC256(uint8_t* master_txBuff, uint16_t Address, uint16_t nDataLength)
{
  i2c_master_transfer_t i2c_master_transfer;  
  status_t ret;
  
  i2c_master_transfer.slaveAddress   = 0x50;
  i2c_master_transfer.direction      = kI2C_Write;                              //1st: 0x50 | R/nW 
  i2c_master_transfer.subaddress     = (uint32_t)Address;                       //2nd: Address Byte 1 (Address takes up 2 Bytes) 
  i2c_master_transfer.subaddressSize = 2;                                       //3rd: Affress Byte 0 
  i2c_master_transfer.data           = master_txBuff;                           //4th: Data 1
  i2c_master_transfer.dataSize       = nDataLength;                             //...: Date n     
  i2c_master_transfer.flags          = kI2C_TransferDefaultFlag;
  ret = I2C_MasterTransferBlocking(I2C0, &i2c_master_transfer);     
  return ret;
}  

__ramfunc status_t I2C_RD_AT24LC256(uint8_t* master_rxBuff, uint16_t Address, uint16_t nDataLength)
{
  i2c_master_transfer_t i2c_master_transfer;  
  status_t ret;
  
  i2c_master_transfer.slaveAddress   = 0x50;
  i2c_master_transfer.direction      = kI2C_Read;                               //0x50 | R/nW  
  i2c_master_transfer.subaddress     = (uint32_t)Address;                       //2nd: Address Byte 1 (Address takes up 2 Bytes) 
  i2c_master_transfer.subaddressSize = 2;                                       //3rd: Affress Byte 0 
  i2c_master_transfer.data           = master_rxBuff;                           //4th: Data 1
  i2c_master_transfer.dataSize       = nDataLength;                             //...: Date n     
  i2c_master_transfer.flags          = kI2C_TransferDefaultFlag;
  ret = I2C_MasterTransferBlocking(I2C0, &i2c_master_transfer);  
  return ret;
} 


