#include "us015.h"
#include "led.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
// �����ж�   �ֳ�������������   PB1---Trig 
////////////////////////////////////////////////////////////////////////////////// 

void Trig_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_Initure.Pin	=	GPIO_PIN_1;			//PB1
	GPIO_Initure.Mode	=	GPIO_MODE_OUTPUT_PP;  	//�������,���ڴ���������
	GPIO_Initure.Pull	=	GPIO_PULLDOWN;		  	//����
	GPIO_Initure.Speed	=	GPIO_SPEED_FREQ_HIGH;			//����
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
}

void ch1_capture(void)
{	
			TRIG1=1;
			delay_us(20);
			TRIG1=0;
}


