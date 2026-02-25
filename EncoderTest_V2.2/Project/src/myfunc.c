#include "myfunc.h"

//==============================================================================
// DEFINES
//==============================================================================


//==============================================================================
// DATA
//==============================================================================
  


//==============================================================================
// Calculate CRC8 function
//==============================================================================
//Check table
__ramfunc uint8_t CRC8_Check(volatile uint8_t * Data, uint8_t Len)
{  
  uint8_t uCRC = 0;  
	
  while(Len--){  
		uCRC ^= *Data++;  
  }  

  return uCRC;  
}  

//Calculation
//__ramfunc uint8_t CRC8_Calcu(volatile uint8_t * Data, uint8_t Len)
//{
//  uint8_t uCRC = 0;
//  uint8_t i = 0;
//	
//	if(Len<=0){
//		return 0;
//	}
//	Len--;
//  while(Len--){
//		uCRC ^= *Data++;
//    for(i=0; i<8; i++){
//      if(uCRC&0x01) uCRC = (uCRC>>1)^0x80;
//      else          uCRC = (uCRC>>1);
//    }
//  }
//  return uCRC;
//}




