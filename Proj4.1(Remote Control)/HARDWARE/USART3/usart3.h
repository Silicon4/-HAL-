#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"

#define RXBUFFERSIZE   1	 					//缓存大小
extern uint8_t aRxBuffer[RXBUFFERSIZE];			//HAL库USART接收Buffer
extern UART_HandleTypeDef huart3;

void MX_USART3_UART_Init(void);


#endif

