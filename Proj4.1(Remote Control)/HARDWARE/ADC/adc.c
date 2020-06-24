#include "adc.h"
#define TSL_SI	PAout(7)	 //SI	7
#define TSL_CLK	 PAout(2)	 //CLK 2

ADC_HandleTypeDef hadc;		//ADC句柄

/**************************************************************************
函数功能：延时
入口参数：无
返回	值：无
**************************************************************************/
void Dly_us(void)
{
	 int ii;	
	 for(ii=0;ii<10;ii++);		
}
/**************************************************************************
函数功能：CCD数据采集
入口参数：无
返回	值：无
**************************************************************************/
 void RD_TSL(void) 
{
 u8 i=0,tslp=0;
	TSL_CLK=1;
	TSL_SI=0; 
	Dly_us();
		
	TSL_SI=1; 
	TSL_CLK=0;
	Dly_us();
		
	TSL_CLK=1;
	TSL_SI=0;
	Dly_us(); 
	for(i=0;i<128;i++)
	{ 
	TSL_CLK=0;
	Dly_us();	//调节曝光时间
	ADV[tslp]=(Get_Adc(3))>>4;
	++tslp;
	TSL_CLK=1;
	 Dly_us();
	}
}
/**************************************************************************
函数功能：AD采样
入口参数：ADC1 的通道
返回	值：AD转换结果
**************************************************************************/
u16 Get_Adc(u8 ch)	 
{
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	
	ADC1_ChanConf.Channel=ch;									 //通道
	ADC1_ChanConf.Rank=1;										 //第1个序列，序列1
	ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;		//采样时间				 
	HAL_ADC_ConfigChannel(&hadc,&ADC1_ChanConf);		//通道配置
	HAL_ADC_Start(&hadc);								 //开启ADC
	HAL_ADC_PollForConversion(&hadc,1);				//轮询转换
	return (u16)HAL_ADC_GetValue(&hadc);				//返回最近一次ADC1规则组的转换结果

//	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
//	ADC1->SQR3|=ch;							
//	ADC1->CR2|=1<<22;		 //启动规则转换通道 
//	while(!(ADC1->SR&1<<1));//等待转换结束	 		 
//	return ADC1->DR;		//返回adc值
}

/**************************************************************************
函数功能：线性CCD初始化
入口参数：无
返回	值：无
备注：这个函数采用两种方式，纯寄存器操作和纯HAL库操作
**************************************************************************/
void	ccd_Init(void)
{	
//先初始化IO口
// 	RCC->APB2ENR|=1<<2;	//使能PORTA口时钟 
//	GPIOA->CRL&=0XFFFF0FFF;//PA3 anolog输入 	 
	GPIO_InitTypeDef GPIO_Initure;
 	__HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	
	
	GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
	GPIO_Initure.Mode=GPIO_MODE_ANALOG;	 //模拟
	GPIO_Initure.Pull=GPIO_NOPULL;			//不带上下拉
	
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	
//	GPIOA->CRL&=0X0FFFF0FF;//PA2	 7 
//	GPIOA->CRL|=0X20000200;//Pa2	 7 推挽输出 2MHZ
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_7;			//PA2 7
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;	//推挽输出
	GPIO_Initure.Pull=GPIO_PULLDOWN;			//不带上下拉
	
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
//	//通道10/11设置			 
//	RCC->APB2ENR|=1<<9;	//ADC1时钟使能		
//	RCC->APB2RSTR|=1<<9;	 //ADC1复位
//	RCC->APB2RSTR&=~(1<<9);//复位结束		
//	RCC->CFGR&=~(3<<14);	 //分频因子清零
//	//SYSCLK/DIV6=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
//	//否则将导致ADC准确度下降!
//	RCC->CFGR|=2<<14;
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;
	__HAL_RCC_ADC1_CLK_ENABLE();			//使能ADC1时钟
	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;			//ADC外设时钟
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//分频因子6时钟为72M/6=12MHz
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);					//设置ADC时钟

	hadc.Instance=ADC1;
	hadc.Init.DataAlign=ADC_DATAALIGN_RIGHT;			 //右对齐
	hadc.Init.ScanConvMode=DISABLE;						//非扫描模式
	hadc.Init.ContinuousConvMode=DISABLE;				//关闭连续转换
	hadc.Init.NbrOfConversion=1;						 //1个转换在规则序列中 也就是只转换规则序列1 
	hadc.Init.DiscontinuousConvMode=DISABLE;			 //禁止不连续采样模式
	hadc.Init.NbrOfDiscConversion=0;					 //不连续采样通道数为0
	hadc.Init.ExternalTrigConv=ADC_SOFTWARE_START;		 //软件触发
	HAL_ADC_Init(&hadc);									//初始化
	
	//ADC1->CR1&=0XF0FFFF;	 //工作模式清零
	//ADC1->CR1|=0<<16;		//独立工作模式
	//ADC1->CR1&=~(1<<8);	//非扫描模式		
	//ADC1->CR2&=~(1<<1);	//单次转换模式
	//ADC1->CR2&=~(7<<17);
	//ADC1->CR2|=7<<17;		 //软件控制转换	
	ADC1->CR2|=1<<20;		//使用用外部触发(SWSTART)我发现HAL初始化之后外部触发没有使能，无奈手动使能
	//ADC1->CR2&=~(1<<11);	 //右对齐	 
	//ADC1->SQR1&=~(0XF<<20);
	//ADC1->SQR1&=0<<20;	 //1个转换在规则序列中 也就是只转换规则序列1
	//设置通道7的采样时间
	
//	ADC1->SMPR2&=0XFFFF0FFF;//通道3采样时间清空
//	ADC1->SMPR2|=7<<9;		//通道3	239.5周期,提高采样时间可以提高精确度
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	
	ADC1_ChanConf.Channel=3;									 //通道
	ADC1_ChanConf.Rank=1;										 //第1个序列，序列1
	ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;		//采样时间				 
	HAL_ADC_ConfigChannel(&hadc,&ADC1_ChanConf);				//通道配置	

// 	ADC1->CR2|=1<<0;		//ADC上电
//	ADC1->CR2|=1<<3;		//使能复位校准，软件置位，硬件清零
//	while(ADC1->CR2&1<<3);	//等待校准结束 		 
//	ADC1->CR2|=1<<2;		//使能AD校准，软件置位，硬件清零 
//	while(ADC1->CR2&1<<2);	//等待校准结束
	HAL_ADCEx_Calibration_Start(&hadc);					 //校准ADC
	HAL_Delay(1);
}
//下面的MSP初始化不需要了，所有的引脚初始化都在前面的CCD初始化函数里

//void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
//{
//	GPIO_InitTypeDef GPIO_Initure;
//	__HAL_RCC_ADC1_CLK_ENABLE();			//使能ADC1时钟
//	__HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
//	
//	GPIO_Initure.Pin=GPIO_PIN_3;			//PA1
//	GPIO_Initure.Mode=GPIO_MODE_ANALOG;	 //模拟
//	GPIO_Initure.Pull=GPIO_NOPULL;			//不带上下拉
//	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
//}
