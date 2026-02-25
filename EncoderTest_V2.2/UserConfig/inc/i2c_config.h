#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "fsl_port.h"
#include "fsl_i2c.h"
#include "fsl_i2c_edma.h"


//==============================================================================
// Definitions
//==============================================================================


//==============================================================================
// Functions
//==============================================================================
void Init_I2C(void);
__ramfunc status_t I2C_WR_AT24LC256(uint8_t* master_txBuff, uint16_t Address, uint16_t nDataLength);
__ramfunc status_t I2C_RD_AT24LC256(uint8_t* master_rxBuff, uint16_t Address, uint16_t nDataLength);


#endif /* _EEPROM_H_ */
