#include "timer.h"

TIM_HandleTypeDef htim3;      //��ʱ��3���
/**************************************************************************
�������ܣ���ʱ��3ͨ��3���벶���ʼ��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
ʹ�üĴ����Ϳ⺯�����ַ�ʽ���г�ʼ��
**************************************************************************/
void TIM3_Cap_Init(u16 arr,u16 psc)
{
//	RCC->APB1ENR|=1<<1;     //TIM3ʱ��ʹ��     
//	RCC->APB2ENR|=1<<3;    	//ʹ��PORTBʱ��   	 
//	GPIOB->CRL&=0XFFFFFF00; 
//	GPIOB->CRL|=0X00000028;//  	PB.0 ���� PB.1���

//	TIM3->ARR=arr;  		//�趨�������Զ���װֵ   
//	TIM3->PSC=psc;  		//Ԥ��Ƶ�� 
//	TIM3->CCMR2|=1<<0;	//ѡ������� 
//	TIM3->CCMR2|=0<<4; 	// ���������˲��� ���˲�
//	TIM3->CCMR2|=0<<2; 	//���������Ƶ,����Ƶ

//	TIM3->CCER|=0<<9; 	//�����ز���
//	TIM3->CCER|=1<<8; 	//�������������ֵ������Ĵ�����

//	TIM3->DIER|=1<<3;   //�������ж�				
//	TIM3->DIER|=1<<0;   //��������ж�	
//	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
//	MY_NVIC_Init(1,3,TIM3_IRQn,1);
	TIM_IC_InitTypeDef TIM3_CH3_IC_Initure;	//��ʱ����ʼ�����
    
    htim3.Instance=TIM3;                          //ѡ��ʱ������ʱ��3
    htim3.Init.Prescaler=psc;                  //Ԥ��Ƶ����72��Ƶ,1M�ļ���Ƶ��,1um����
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;    //�����������ϼ���
    htim3.Init.Period=arr;                      //�Զ�װ��ֵ������ʱ10ms
    htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ������Ƶ
    HAL_TIM_IC_Init(&htim3);						//���г�ʼ��
    
    //��ʼ��TIM1���벶�����
    TIM3_CH3_IC_Initure.ICPolarity=TIM_ICPOLARITY_RISING;    //�����ԣ������ز���
    TIM3_CH3_IC_Initure.ICSelection=TIM_ICSELECTION_DIRECTTI;//����ӳ�䣺ֱ��ӳ�䵽TI3��
    TIM3_CH3_IC_Initure.ICPrescaler=TIM_ICPSC_DIV1;          //���벶���Ƶ������Ƶ
    TIM3_CH3_IC_Initure.ICFilter=0x03;                       //Ӳ���˲�������8����ʱ��ʱ�������˲�
    HAL_TIM_IC_ConfigChannel(&htim3,&TIM3_CH3_IC_Initure,TIM_CHANNEL_3);//ͨ��ѡ������TIM4ͨ��3
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);   //��ʼ����TIM3��ͨ��3
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);   //ʹ�ܸ����ж�
}

//Input Capture ���벶��MSP����
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM3_CLK_ENABLE();            //ʹ��TIM3ʱ��
    __HAL_RCC_GPIOB_CLK_ENABLE();			//����GPIOBʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_0;            //PB0
    GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;  	//��������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_1;            //PB1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM3_IRQn,1,3); 	//�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
    HAL_NVIC_EnableIRQ(TIM3_IRQn);       	//����ITM4�ж�
}
/**************************************************************************
�������ܣ����������ջز�����
��ڲ�������
����  ֵ����
**************************************************************************/
u16 TIM3CH3_CAPTURE_STA,TIM3CH3_CAPTURE_VAL;
void Read_Distane(void)
{   
	 PBout(1)=1;
	 delay_us(15);  
	 PBout(1)=0;	
			if(TIM3CH3_CAPTURE_STA&0X80)//�ɹ�������һ�θߵ�ƽ
		{
			Distance=TIM3CH3_CAPTURE_STA&0X3F;
			Distance*=65536;					        //���ʱ���ܺ�
			Distance+=TIM3CH3_CAPTURE_VAL;		//�õ��ܵĸߵ�ƽʱ��
			Distance=Distance*170/1000;
		//	printf("%d \r\n",Distance);
			TIM3CH3_CAPTURE_STA=0;			//������һ�β���
		}				
}
/**************************************************************************
�������ܣ��������ز������ȡ�ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM3_IRQHandler(void)
{ 		    		  			    
	u16 tsr;
	tsr=TIM3->SR;
	if((TIM3CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(tsr&0X01)//���
		{	    
			if(TIM3CH3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM3CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM3CH3_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM3CH3_CAPTURE_VAL=0XFFFF;
				}
				else TIM3CH3_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x08)//����3���������¼�
		{	
			if(TIM3CH3_CAPTURE_STA&0X40)		//����һ���½��� 		
			{		
				TIM3CH3_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM3CH3_CAPTURE_VAL=TIM3->CCR3;	//��ȡ��ǰ�Ĳ���ֵ.
				TIM3->CCER&=~(1<<9);			//CC1P=0 ����Ϊ�����ز���
			}
			else  								//��δ��ʼ,��һ�β���������
			{
				TIM3CH3_CAPTURE_STA=0;			//���
				TIM3CH3_CAPTURE_VAL=0;
				TIM3CH3_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM3->CNT=0;					//���������
				TIM3->CCER|=1<<9; 				//CC1P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
	}
	TIM3->SR=0;//����жϱ�־λ 	     
}
