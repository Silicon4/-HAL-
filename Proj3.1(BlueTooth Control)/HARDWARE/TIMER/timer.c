#include "timer.h"

TIM_HandleTypeDef htim3;      //定时器3句柄
/**************************************************************************
函数功能：定时器3通道3输入捕获初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
使用寄存器和库函数两种方式进行初始化
**************************************************************************/
void TIM3_Cap_Init(u16 arr,u16 psc)
{
//	RCC->APB1ENR|=1<<1;     //TIM3时钟使能     
//	RCC->APB2ENR|=1<<3;    	//使能PORTB时钟   	 
//	GPIOB->CRL&=0XFFFFFF00; 
//	GPIOB->CRL|=0X00000028;//  	PB.0 输入 PB.1输出

//	TIM3->ARR=arr;  		//设定计数器自动重装值   
//	TIM3->PSC=psc;  		//预分频器 
//	TIM3->CCMR2|=1<<0;	//选择输入端 
//	TIM3->CCMR2|=0<<4; 	// 配置输入滤波器 不滤波
//	TIM3->CCMR2|=0<<2; 	//配置输入分频,不分频

//	TIM3->CCER|=0<<9; 	//上升沿捕获
//	TIM3->CCER|=1<<8; 	//允许捕获计数器的值到捕获寄存器中

//	TIM3->DIER|=1<<3;   //允许捕获中断				
//	TIM3->DIER|=1<<0;   //允许更新中断	
//	TIM3->CR1|=0x01;    //使能定时器3
//	MY_NVIC_Init(1,3,TIM3_IRQn,1);
	TIM_IC_InitTypeDef TIM3_CH3_IC_Initure;	//定时器初始化句柄
    
    htim3.Instance=TIM3;                          //选择定时器：定时器3
    htim3.Init.Prescaler=psc;                  //预分频器：72分频,1M的计数频率,1um周期
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;    //计数方向：向上计数
    htim3.Init.Period=arr;                      //自动装载值：最大计时10ms
    htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频：不分频
    HAL_TIM_IC_Init(&htim3);						//进行初始化
    
    //初始化TIM1输入捕获参数
    TIM3_CH3_IC_Initure.ICPolarity=TIM_ICPOLARITY_RISING;    //捕获极性：上升沿捕获
    TIM3_CH3_IC_Initure.ICSelection=TIM_ICSELECTION_DIRECTTI;//交错映射：直接映射到TI3上
    TIM3_CH3_IC_Initure.ICPrescaler=TIM_ICPSC_DIV1;          //输入捕获分频：不分频
    TIM3_CH3_IC_Initure.ICFilter=0x03;                       //硬件滤波：设置8个定时器时钟周期滤波
    HAL_TIM_IC_ConfigChannel(&htim3,&TIM3_CH3_IC_Initure,TIM_CHANNEL_3);//通道选择：配置TIM4通道3
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);   //开始捕获TIM3的通道3
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);   //使能更新中断
}

//Input Capture 输入捕获MSP函数
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_0;            //PB0
    GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;  	//复用输入
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_1;            //PB1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM3_IRQn,1,3); 	//设置中断优先级，抢占优先级1，子优先级3
    HAL_NVIC_EnableIRQ(TIM3_IRQn);       	//开启ITM4中断
}
/**************************************************************************
函数功能：超声波接收回波函数
入口参数：无
返回  值：无
**************************************************************************/
u16 TIM3CH3_CAPTURE_STA,TIM3CH3_CAPTURE_VAL;
void Read_Distane(void)
{   
	 PBout(1)=1;
	 delay_us(15);  
	 PBout(1)=0;	
			if(TIM3CH3_CAPTURE_STA&0X80)//成功捕获到了一次高电平
		{
			Distance=TIM3CH3_CAPTURE_STA&0X3F;
			Distance*=65536;					        //溢出时间总和
			Distance+=TIM3CH3_CAPTURE_VAL;		//得到总的高电平时间
			Distance=Distance*170/1000;
		//	printf("%d \r\n",Distance);
			TIM3CH3_CAPTURE_STA=0;			//开启下一次捕获
		}				
}
/**************************************************************************
函数功能：超声波回波脉宽读取中断
入口参数：无
返回  值：无
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM3->SR;
	if((TIM3CH3_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(tsr&0X01)//溢出
		{	    
			if(TIM3CH3_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM3CH3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM3CH3_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM3CH3_CAPTURE_VAL=0XFFFF;
				}
				else TIM3CH3_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x08)//捕获3发生捕获事件
		{	
			if(TIM3CH3_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{		
				TIM3CH3_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
				TIM3CH3_CAPTURE_VAL=TIM3->CCR3;	//获取当前的捕获值.
				TIM3->CCER&=~(1<<9);			//CC1P=0 设置为上升沿捕获
			}
			else  								//还未开始,第一次捕获上升沿
			{
				TIM3CH3_CAPTURE_STA=0;			//清空
				TIM3CH3_CAPTURE_VAL=0;
				TIM3CH3_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM3->CNT=0;					//计数器清空
				TIM3->CCER|=1<<9; 				//CC1P=1 设置为下降沿捕获
			}		    
		}			     	    					   
	}
	TIM3->SR=0;//清除中断标志位 	     
}
