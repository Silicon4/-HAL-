#include "exti.h"


void MiniBalance_EXTI_Init(void)
{ 
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_Initure.Pin = GPIO_PIN_12;
	GPIO_Initure.Mode = GPIO_MODE_IT_FALLING;		//�ж���������
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Initure.Pull = GPIO_PULLUP;				//��������
	
	HAL_GPIO_Init(GPIOA, &GPIO_Initure);
	
	HAL_NVIC_SetPriority(EXTI15_10_IRQn,2,0);       //��ռ���ȼ�Ϊ12�������ȼ�Ϊ2
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);             //ʹ���ж���12
}


