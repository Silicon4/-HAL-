#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"

#define RXBUFFERSIZE   1	 					//�����С
extern uint8_t aRxBuffer[RXBUFFERSIZE];			//HAL��USART����Buffer
extern UART_HandleTypeDef huart3;

void MX_USART3_UART_Init(void);


#endif

