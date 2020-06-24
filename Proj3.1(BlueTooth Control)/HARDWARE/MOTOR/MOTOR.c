#include "MOTOR.h"

TIM_HandleTypeDef 	TIM1_Handler = {0};	  	//定时器1句柄 
TIM_OC_InitTypeDef 	TIM1_CH14Handler = {0};	//定时器1通道14句柄


void MiniBalance_Motor_Init(void)		//完成4个控制引脚的初始化
{
	GPIO_InitTypeDef GPIO_Initure = {0};	//声明初始化结构体
	__HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟

	GPIO_Initure.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;	//端口配置
	GPIO_Initure.Pull = GPIO_NOPULL;
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;	  						//推挽输出
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;					 //50M
	
	HAL_GPIO_Init(GPIOB, &GPIO_Initure);				  //根据设定参数初始化GPIOB 
}


//TIM1 PWM初始化 
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void MiniBalance_PWM_Init(u16 arr,u16 psc)		//完成两个PWM输出引脚的初始化
{
	TIM1_Handler.Instance=TIM1;							//定时器1
	TIM1_Handler.Init.Prescaler=psc;					//定时器分频
	TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;	//向上计数模式
	TIM1_Handler.Init.Period=arr;		 				//自动重装载值
	TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&TIM1_Handler);	  			 //初始化PWM
	
	TIM1_CH14Handler.OCMode=TIM_OCMODE_PWM1;		 	//模式选择PWM1
	TIM1_CH14Handler.Pulse= 0;						//设置比较值,初始占空比设为0
	TIM1_CH14Handler.OCPolarity=TIM_OCPOLARITY_HIGH; 	//输出极性 高电平有效 
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH14Handler,TIM_CHANNEL_1);//初始化TIM1，PWM通道1、4
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH14Handler,TIM_CHANNEL_4);//初始化TIM1，PWM通道1、4
	
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1);	//开启PWM通道1、4
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_4);	//开启PWM通道1、4
}

//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用 这里并不使用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM1)		//？
	{
		__HAL_RCC_TIM1_CLK_ENABLE();            //使能TIM1时钟
		HAL_NVIC_SetPriority(TIM1_UP_IRQn,2,2); //设置更新(计数溢出)中断，抢占优先级2，子优先级2
		HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);          //开启ITM1更新中断
	}
}

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	
    if(htim->Instance==TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();			//使能TIM1时钟
		__HAL_AFIO_REMAP_TIM1_PARTIAL();		//TIM1通道引脚部分重映射使能
		__HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
		
		GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_11;           	//PA8 PA11
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出,用于输出PWM
		GPIO_Initure.Pull=GPIO_PULLDOWN;          //下拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
	}
}

//定时器3中断服务函数
void TIM1_IRQHandler(void)
{
    //HAL_TIM_IRQHandler(&TIM1_Handler);		//调用HAL库定时器中断处理函数
}
