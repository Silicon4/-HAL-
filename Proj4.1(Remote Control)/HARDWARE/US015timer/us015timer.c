#include "us015timer.h"

//  PC8---Echo     TIM3 CH3 


//TIM3  捕获初始化
TIM_HandleTypeDef TIM3_Handler;         //定时器3句柄

//定时器3通道1输入捕获配置
//arr：自动重装值(TIM3是16位的!!)
//psc：时钟预分频数
void TIM3_CH3_Cap_Init(u32 arr,u16 psc)
{  
    TIM_IC_InitTypeDef TIM3_CH3Config;  
    
    TIM3_Handler.Instance=TIM3;                          //通用定时器3
    TIM3_Handler.Init.Prescaler=psc;                     //分频系数
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM3_Handler.Init.Period=arr;                        //自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
   	TIM3_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;//使能自动重载
    HAL_TIM_IC_Init(&TIM3_Handler);						//初始化输入捕获时基参数
    
    TIM3_CH3Config.ICPolarity=TIM_ICPOLARITY_RISING;    //上升沿捕获
    TIM3_CH3Config.ICSelection=TIM_ICSELECTION_DIRECTTI;//映射到TI1上
    TIM3_CH3Config.ICPrescaler=TIM_ICPSC_DIV1;          //配置输入分频，不分频
    TIM3_CH3Config.ICFilter=0;                          //配置输入滤波器，不滤波
    HAL_TIM_IC_ConfigChannel(&TIM3_Handler,&TIM3_CH3Config,TIM_CHANNEL_3);//配置TIM3通道3
	
    HAL_TIM_IC_Start_IT(&TIM3_Handler,TIM_CHANNEL_3);   //开启TIM3的捕获通道3，并且开启捕获中断
    __HAL_TIM_ENABLE_IT(&TIM3_Handler,TIM_IT_UPDATE);   //使能更新中断
	
	HAL_NVIC_SetPriority(TIM3_IRQn,0,0);    //设置中断优先级，抢占优先级2，子优先级0
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断通道  
}

//定时器2底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_IC_Init()调用
//htim:定时器2句柄
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
    __HAL_AFIO_REMAP_TIM3_ENABLE();	
	__HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOC时钟
	
    GPIO_Initure.Pin=GPIO_PIN_0;            //PB0
    GPIO_Initure.Mode=GPIO_MODE_AF_INPUT; 	//复用推挽输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM3_IRQn,0,0);    //设置中断优先级，抢占优先级3，子优先级0
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断通道  
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数
u8  TIM3CH3_CAPTURE_STA=0;							//输入捕获状态		    				
u16	TIM3CH3_CAPTURE_VAL;							  //输入捕获值(TIM3是16位)

//定时器2中断服务函数
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
											}else TIM3CH3_CAPTURE_STA++;
										}	 
								}
						   	if(tsr&0x08)//捕获3发生捕获事件
				    	{	
											if(TIM3CH3_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
											{	  			
											TIM3CH3_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
											TIM3CH3_CAPTURE_VAL=TIM3->CCR3;	//获取当前的捕获值.
											TIM3->CCER&=~(1<<9);			//CC1P=0 设置为上升沿捕获
									  	}else  								//还未开始,第一次捕获上升沿
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
 
//定时器更新中断（计数溢出）中断处理回调函数， 该函数在HAL_TIM_IRQHandler中会被调用,实现多个计数器满装载的计数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//更新中断（溢出）发生时执行
{
	
}

//定时器输入捕获中断处理回调函数，该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{
	
}

/**************************************************************************
函数功能：超声波接收回波函数
入口参数：无
返回  值：无
**************************************************************************/
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
			TIM3CH3_CAPTURE_STA=0;			//开启下一次捕获
		}				
}
