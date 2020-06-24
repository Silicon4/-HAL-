#include "timer.h"
#include "delay.h"


TIM_HandleTypeDef htim3;      //定时器3句柄

void Remote_Init(void)
{  
    TIM_IC_InitTypeDef TIM3_CH3_IC_Initure;	//定时器初始化句柄
    
    htim3.Instance=TIM3;                          //选择定时器：定时器3
    htim3.Init.Prescaler=(72-1);                  //预分频器：72分频,1M的计数频率,1um周期
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;    //计数方向：向上计数
    htim3.Init.Period=10000;                      //自动装载值：最大计时10ms
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

    HAL_NVIC_SetPriority(TIM3_IRQn,1,3); 	//设置中断优先级，抢占优先级1，子优先级3
    HAL_NVIC_EnableIRQ(TIM3_IRQn);       	//开启ITM4中断
}

//遥控器接收状态
//[7]:收到了引导码标志
//[6]:得到了一个按键的所有信息
//[5]:保留	
//[4]:标记上升沿是否已经被捕获								   
//[3:0]:溢出计时器
u8 	RmtSta=0;	  	  
u16 Dval;		//上升沿期间,计数器增加的值
u32 RmtRec=0;	//红外接收到的数据	   		    
u8  RmtCnt=0;	//按键按下的次数

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);//HAL库定时器处理函数
} 

//定时器更新（溢出）中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
 		if(RmtSta&0x80)//检测第[7]位，如果收到引导码
		{
			RmtSta&=~0X10;						//清零第[5]位,清除上升沿捕获标志
			if((RmtSta&0X0F)==0X00)RmtSta|=1<<6;//标记已经完成一次按键的键值信息采集
			if((RmtSta&0X0F)<14)RmtSta++;
			else
			{
				RmtSta&=~(0x80);//清空引导标识
				RmtSta&=~(0x0F);//清空溢出计数器
			}						 	   	
		}	
	}
}

//定时器输入捕获中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{
	if(htim->Instance==TIM3)
	{
		if(DATA_PIN)//读取引脚状态，如果引脚高电平，说明是上升沿捕获
		{
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3);   					//清除捕获极性
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);//配置捕获极性，设置为下降沿捕获
			__HAL_TIM_SET_COUNTER(&htim3,0);		//清空定时器计数值	  
		  	RmtSta|=0X10;							//设置第[4]位，标记上升沿已经被捕获
		}else //下降沿捕获
		{
			Dval=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);//读取输入捕获的计数值
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3);   //清除捕获极性
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);//配置捕获极性，设置为上升沿捕获
			if(RmtSta&0X10)							//完成一次上升沿捕获 
			{
 				if(RmtSta&0X80)//接收到了引导码
				{
					
					if(Dval>300&&Dval<800)			//收到逻辑0,560us
					{
						RmtRec<<=1;
						RmtRec|=0;   
					}
					else if(Dval>1400&&Dval<1800)	//收到逻辑1,1680us
					{
						RmtRec<<=1;
						RmtRec|=1;
					}
					else if(Dval>2200&&Dval<2600)	//收到重复码,2.5ms
					{
						RmtCnt++; 					//按键次数增加1次
						RmtSta&=~(0x0F);			//清空溢出计数器		
					}
 				}
				else if(Dval>4200&&Dval<4700)		//收到起始码,4.5ms
				{
					RmtSta|=0x80;					//设置第[7]位，表示成功接收到了引导码
					RmtCnt=0;						//清除按键次数计数器
				}						 
			}
			RmtSta&=~(0x10);						//清除上升沿捕获标志
		}
	}
}

u8 Remote_Scan(void)
{        
	u8 sta=0;       
	u8 temp1,temp2;
	if(RmtSta&(1<<6))//得到一个按键的所有信息了
	{ 
	    temp1=RmtRec>>24;			//得到地址码
	    temp2=(RmtRec>>16)&0xff;	//得到地址反码 
 	    if((temp1==(u8)~temp2)&&temp1==REMOTE_ID)//检验遥控地址码,遥控器发送的地址码要和这里宏定义的相同才匹配 
	    { 
	        temp1=RmtRec>>8;
	        temp2=RmtRec; 	
	        if(temp1==(u8)~temp2)sta=temp1;//键值正确	 
		}   
		if((sta==0)||((RmtSta&0X80)==0))//按键数据错误/遥控已经没有按下了
		{
		 	RmtSta&=~(1<<6);//清除接收到有效按键标识
			RmtCnt=0;		//清除按键次数计数器
		}
	}
    return sta;
}

void read_yaokong(void)
{
	u8 key;
	key = Remote_Scan();
	switch (key)
	{
		case 24: Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0; break;	//前进
		case 74: Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0; break;	//后退
		case 16: Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0; break;	//左转
		case 90: Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1; break;	//右转
		case 56: Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0; break;	//停止
		case 162: mode_flag = 0; break;	//遥控模式
		case 98: mode_flag = 1; break;	//CCD循迹模式
		case 226: mode_flag = 2; break;	//超声波避障模式
	}
}
