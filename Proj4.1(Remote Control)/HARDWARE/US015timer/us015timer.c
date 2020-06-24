#include "us015timer.h"

//  PC8---Echo     TIM3 CH3 


//TIM3  �����ʼ��
TIM_HandleTypeDef TIM3_Handler;         //��ʱ��3���

//��ʱ��3ͨ��1���벶������
//arr���Զ���װֵ(TIM3��16λ��!!)
//psc��ʱ��Ԥ��Ƶ��
void TIM3_CH3_Cap_Init(u32 arr,u16 psc)
{  
    TIM_IC_InitTypeDef TIM3_CH3Config;  
    
    TIM3_Handler.Instance=TIM3;                          //ͨ�ö�ʱ��3
    TIM3_Handler.Init.Prescaler=psc;                     //��Ƶϵ��
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM3_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
   	TIM3_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;//ʹ���Զ�����
    HAL_TIM_IC_Init(&TIM3_Handler);						//��ʼ�����벶��ʱ������
    
    TIM3_CH3Config.ICPolarity=TIM_ICPOLARITY_RISING;    //�����ز���
    TIM3_CH3Config.ICSelection=TIM_ICSELECTION_DIRECTTI;//ӳ�䵽TI1��
    TIM3_CH3Config.ICPrescaler=TIM_ICPSC_DIV1;          //���������Ƶ������Ƶ
    TIM3_CH3Config.ICFilter=0;                          //���������˲��������˲�
    HAL_TIM_IC_ConfigChannel(&TIM3_Handler,&TIM3_CH3Config,TIM_CHANNEL_3);//����TIM3ͨ��3
	
    HAL_TIM_IC_Start_IT(&TIM3_Handler,TIM_CHANNEL_3);   //����TIM3�Ĳ���ͨ��3�����ҿ��������ж�
    __HAL_TIM_ENABLE_IT(&TIM3_Handler,TIM_IT_UPDATE);   //ʹ�ܸ����ж�
	
	HAL_NVIC_SetPriority(TIM3_IRQn,0,0);    //�����ж����ȼ�����ռ���ȼ�2�������ȼ�0
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          //����ITM3�ж�ͨ��  
}

//��ʱ��2�ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_IC_Init()����
//htim:��ʱ��2���
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM3_CLK_ENABLE();            //ʹ��TIM3ʱ��
    __HAL_AFIO_REMAP_TIM3_ENABLE();	
	__HAL_RCC_GPIOB_CLK_ENABLE();			//����GPIOCʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_0;            //PB0
    GPIO_Initure.Mode=GPIO_MODE_AF_INPUT; 	//������������
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;     //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM3_IRQn,0,0);    //�����ж����ȼ�����ռ���ȼ�3�������ȼ�0
    HAL_NVIC_EnableIRQ(TIM3_IRQn);          //����ITM3�ж�ͨ��  
}

//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���
u8  TIM3CH3_CAPTURE_STA=0;							//���벶��״̬		    				
u16	TIM3CH3_CAPTURE_VAL;							  //���벶��ֵ(TIM3��16λ)

//��ʱ��2�жϷ�����
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
											}else TIM3CH3_CAPTURE_STA++;
										}	 
								}
						   	if(tsr&0x08)//����3���������¼�
				    	{	
											if(TIM3CH3_CAPTURE_STA&0X40)		//����һ���½��� 		
											{	  			
											TIM3CH3_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
											TIM3CH3_CAPTURE_VAL=TIM3->CCR3;	//��ȡ��ǰ�Ĳ���ֵ.
											TIM3->CCER&=~(1<<9);			//CC1P=0 ����Ϊ�����ز���
									  	}else  								//��δ��ʼ,��һ�β���������
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
 
//��ʱ�������жϣ�����������жϴ���ص������� �ú�����HAL_TIM_IRQHandler�лᱻ����,ʵ�ֶ����������װ�صļ���
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//�����жϣ����������ʱִ��
{
	
}

//��ʱ�����벶���жϴ���ص��������ú�����HAL_TIM_IRQHandler�лᱻ����
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//�����жϷ���ʱִ��
{
	
}

/**************************************************************************
�������ܣ����������ջز�����
��ڲ�������
����  ֵ����
**************************************************************************/
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
			TIM3CH3_CAPTURE_STA=0;			//������һ�β���
		}				
}
