#include "adc.h"
#define TSL_SI	PAout(7)	 //SI	7
#define TSL_CLK	 PAout(2)	 //CLK 2

ADC_HandleTypeDef hadc;		//ADC���

/**************************************************************************
�������ܣ���ʱ
��ڲ�������
����	ֵ����
**************************************************************************/
void Dly_us(void)
{
	 int ii;	
	 for(ii=0;ii<10;ii++);		
}
/**************************************************************************
�������ܣ�CCD���ݲɼ�
��ڲ�������
����	ֵ����
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
	Dly_us();	//�����ع�ʱ��
	ADV[tslp]=(Get_Adc(3))>>4;
	++tslp;
	TSL_CLK=1;
	 Dly_us();
	}
}
/**************************************************************************
�������ܣ�AD����
��ڲ�����ADC1 ��ͨ��
����	ֵ��ADת�����
**************************************************************************/
u16 Get_Adc(u8 ch)	 
{
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	
	ADC1_ChanConf.Channel=ch;									 //ͨ��
	ADC1_ChanConf.Rank=1;										 //��1�����У�����1
	ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;		//����ʱ��				 
	HAL_ADC_ConfigChannel(&hadc,&ADC1_ChanConf);		//ͨ������
	HAL_ADC_Start(&hadc);								 //����ADC
	HAL_ADC_PollForConversion(&hadc,1);				//��ѯת��
	return (u16)HAL_ADC_GetValue(&hadc);				//�������һ��ADC1�������ת�����

//	ADC1->SQR3&=0XFFFFFFE0;//��������1 ͨ��ch
//	ADC1->SQR3|=ch;							
//	ADC1->CR2|=1<<22;		 //��������ת��ͨ�� 
//	while(!(ADC1->SR&1<<1));//�ȴ�ת������	 		 
//	return ADC1->DR;		//����adcֵ
}

/**************************************************************************
�������ܣ�����CCD��ʼ��
��ڲ�������
����	ֵ����
��ע����������������ַ�ʽ�����Ĵ��������ʹ�HAL�����
**************************************************************************/
void	ccd_Init(void)
{	
//�ȳ�ʼ��IO��
// 	RCC->APB2ENR|=1<<2;	//ʹ��PORTA��ʱ�� 
//	GPIOA->CRL&=0XFFFF0FFF;//PA3 anolog���� 	 
	GPIO_InitTypeDef GPIO_Initure;
 	__HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
	
	
	GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
	GPIO_Initure.Mode=GPIO_MODE_ANALOG;	 //ģ��
	GPIO_Initure.Pull=GPIO_NOPULL;			//����������
	
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	
//	GPIOA->CRL&=0X0FFFF0FF;//PA2	 7 
//	GPIOA->CRL|=0X20000200;//Pa2	 7 ������� 2MHZ
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_7;			//PA2 7
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;	//�������
	GPIO_Initure.Pull=GPIO_PULLDOWN;			//����������
	
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
//	//ͨ��10/11����			 
//	RCC->APB2ENR|=1<<9;	//ADC1ʱ��ʹ��		
//	RCC->APB2RSTR|=1<<9;	 //ADC1��λ
//	RCC->APB2RSTR&=~(1<<9);//��λ����		
//	RCC->CFGR&=~(3<<14);	 //��Ƶ��������
//	//SYSCLK/DIV6=12M ADCʱ������Ϊ12M,ADC���ʱ�Ӳ��ܳ���14M!
//	//���򽫵���ADC׼ȷ���½�!
//	RCC->CFGR|=2<<14;
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;
	__HAL_RCC_ADC1_CLK_ENABLE();			//ʹ��ADC1ʱ��
	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;			//ADC����ʱ��
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//��Ƶ����6ʱ��Ϊ72M/6=12MHz
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);					//����ADCʱ��

	hadc.Instance=ADC1;
	hadc.Init.DataAlign=ADC_DATAALIGN_RIGHT;			 //�Ҷ���
	hadc.Init.ScanConvMode=DISABLE;						//��ɨ��ģʽ
	hadc.Init.ContinuousConvMode=DISABLE;				//�ر�����ת��
	hadc.Init.NbrOfConversion=1;						 //1��ת���ڹ��������� Ҳ����ֻת����������1 
	hadc.Init.DiscontinuousConvMode=DISABLE;			 //��ֹ����������ģʽ
	hadc.Init.NbrOfDiscConversion=0;					 //����������ͨ����Ϊ0
	hadc.Init.ExternalTrigConv=ADC_SOFTWARE_START;		 //�������
	HAL_ADC_Init(&hadc);									//��ʼ��
	
	//ADC1->CR1&=0XF0FFFF;	 //����ģʽ����
	//ADC1->CR1|=0<<16;		//��������ģʽ
	//ADC1->CR1&=~(1<<8);	//��ɨ��ģʽ		
	//ADC1->CR2&=~(1<<1);	//����ת��ģʽ
	//ADC1->CR2&=~(7<<17);
	//ADC1->CR2|=7<<17;		 //�������ת��	
	ADC1->CR2|=1<<20;		//ʹ�����ⲿ����(SWSTART)�ҷ���HAL��ʼ��֮���ⲿ����û��ʹ�ܣ������ֶ�ʹ��
	//ADC1->CR2&=~(1<<11);	 //�Ҷ���	 
	//ADC1->SQR1&=~(0XF<<20);
	//ADC1->SQR1&=0<<20;	 //1��ת���ڹ��������� Ҳ����ֻת����������1
	//����ͨ��7�Ĳ���ʱ��
	
//	ADC1->SMPR2&=0XFFFF0FFF;//ͨ��3����ʱ�����
//	ADC1->SMPR2|=7<<9;		//ͨ��3	239.5����,��߲���ʱ�������߾�ȷ��
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	
	ADC1_ChanConf.Channel=3;									 //ͨ��
	ADC1_ChanConf.Rank=1;										 //��1�����У�����1
	ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;		//����ʱ��				 
	HAL_ADC_ConfigChannel(&hadc,&ADC1_ChanConf);				//ͨ������	

// 	ADC1->CR2|=1<<0;		//ADC�ϵ�
//	ADC1->CR2|=1<<3;		//ʹ�ܸ�λУ׼�������λ��Ӳ������
//	while(ADC1->CR2&1<<3);	//�ȴ�У׼���� 		 
//	ADC1->CR2|=1<<2;		//ʹ��ADУ׼�������λ��Ӳ������ 
//	while(ADC1->CR2&1<<2);	//�ȴ�У׼����
	HAL_ADCEx_Calibration_Start(&hadc);					 //У׼ADC
	HAL_Delay(1);
}
//�����MSP��ʼ������Ҫ�ˣ����е����ų�ʼ������ǰ���CCD��ʼ��������

//void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
//{
//	GPIO_InitTypeDef GPIO_Initure;
//	__HAL_RCC_ADC1_CLK_ENABLE();			//ʹ��ADC1ʱ��
//	__HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
//	
//	GPIO_Initure.Pin=GPIO_PIN_3;			//PA1
//	GPIO_Initure.Mode=GPIO_MODE_ANALOG;	 //ģ��
//	GPIO_Initure.Pull=GPIO_NOPULL;			//����������
//	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
//}
