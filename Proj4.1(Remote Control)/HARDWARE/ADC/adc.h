#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"

#define Battery_Ch 6

void Dly_us(void);
 void RD_TSL(void) ;
u16 Get_Adc(u8 ch); 
void  ccd_Init(void);
#endif 












