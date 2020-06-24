#include "exti.h"


void MiniBalance_EXTI_Init(void)
{ 
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_Initure.Pin = GPIO_PIN_12;
	GPIO_Initure.Mode = GPIO_MODE_IT_FALLING;		//中断下拉触发
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Initure.Pull = GPIO_PULLUP;				//上拉输入
	
	HAL_GPIO_Init(GPIOA, &GPIO_Initure);
	
	HAL_NVIC_SetPriority(EXTI15_10_IRQn,2,0);       //抢占优先级为12，子优先级为2
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);             //使能中断线12
}


