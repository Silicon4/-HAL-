#include "timer.h"
#include "delay.h"


TIM_HandleTypeDef htim3;      //��ʱ��3���

void Remote_Init(void)
{  
    TIM_IC_InitTypeDef TIM3_CH3_IC_Initure;	//��ʱ����ʼ�����
    
    htim3.Instance=TIM3;                          //ѡ��ʱ������ʱ��3
    htim3.Init.Prescaler=(72-1);                  //Ԥ��Ƶ����72��Ƶ,1M�ļ���Ƶ��,1um����
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;    //�����������ϼ���
    htim3.Init.Period=10000;                      //�Զ�װ��ֵ������ʱ10ms
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

    HAL_NVIC_SetPriority(TIM3_IRQn,1,3); 	//�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
    HAL_NVIC_EnableIRQ(TIM3_IRQn);       	//����ITM4�ж�
}

//ң��������״̬
//[7]:�յ����������־
//[6]:�õ���һ��������������Ϣ
//[5]:����	
//[4]:����������Ƿ��Ѿ�������								   
//[3:0]:�����ʱ��
u8 	RmtSta=0;	  	  
u16 Dval;		//�������ڼ�,���������ӵ�ֵ
u32 RmtRec=0;	//������յ�������	   		    
u8  RmtCnt=0;	//�������µĴ���

//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);//HAL�ⶨʱ��������
} 

//��ʱ�����£�������жϻص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
 		if(RmtSta&0x80)//����[7]λ������յ�������
		{
			RmtSta&=~0X10;						//�����[5]λ,��������ز����־
			if((RmtSta&0X0F)==0X00)RmtSta|=1<<6;//����Ѿ����һ�ΰ����ļ�ֵ��Ϣ�ɼ�
			if((RmtSta&0X0F)<14)RmtSta++;
			else
			{
				RmtSta&=~(0x80);//���������ʶ
				RmtSta&=~(0x0F);//������������
			}						 	   	
		}	
	}
}

//��ʱ�����벶���жϻص�����
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//�����жϷ���ʱִ��
{
	if(htim->Instance==TIM3)
	{
		if(DATA_PIN)//��ȡ����״̬��������Ÿߵ�ƽ��˵���������ز���
		{
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3);   					//���������
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING);//���ò����ԣ�����Ϊ�½��ز���
			__HAL_TIM_SET_COUNTER(&htim3,0);		//��ն�ʱ������ֵ	  
		  	RmtSta|=0X10;							//���õ�[4]λ������������Ѿ�������
		}else //�½��ز���
		{
			Dval=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_3);//��ȡ���벶��ļ���ֵ
			TIM_RESET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3);   //���������
			TIM_SET_CAPTUREPOLARITY(&htim3,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING);//���ò����ԣ�����Ϊ�����ز���
			if(RmtSta&0X10)							//���һ�������ز��� 
			{
 				if(RmtSta&0X80)//���յ���������
				{
					
					if(Dval>300&&Dval<800)			//�յ��߼�0,560us
					{
						RmtRec<<=1;
						RmtRec|=0;   
					}
					else if(Dval>1400&&Dval<1800)	//�յ��߼�1,1680us
					{
						RmtRec<<=1;
						RmtRec|=1;
					}
					else if(Dval>2200&&Dval<2600)	//�յ��ظ���,2.5ms
					{
						RmtCnt++; 					//������������1��
						RmtSta&=~(0x0F);			//������������		
					}
 				}
				else if(Dval>4200&&Dval<4700)		//�յ���ʼ��,4.5ms
				{
					RmtSta|=0x80;					//���õ�[7]λ����ʾ�ɹ����յ���������
					RmtCnt=0;						//�����������������
				}						 
			}
			RmtSta&=~(0x10);						//��������ز����־
		}
	}
}

u8 Remote_Scan(void)
{        
	u8 sta=0;       
	u8 temp1,temp2;
	if(RmtSta&(1<<6))//�õ�һ��������������Ϣ��
	{ 
	    temp1=RmtRec>>24;			//�õ���ַ��
	    temp2=(RmtRec>>16)&0xff;	//�õ���ַ���� 
 	    if((temp1==(u8)~temp2)&&temp1==REMOTE_ID)//����ң�ص�ַ��,ң�������͵ĵ�ַ��Ҫ������궨�����ͬ��ƥ�� 
	    { 
	        temp1=RmtRec>>8;
	        temp2=RmtRec; 	
	        if(temp1==(u8)~temp2)sta=temp1;//��ֵ��ȷ	 
		}   
		if((sta==0)||((RmtSta&0X80)==0))//�������ݴ���/ң���Ѿ�û�а�����
		{
		 	RmtSta&=~(1<<6);//������յ���Ч������ʶ
			RmtCnt=0;		//�����������������
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
		case 24: Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0; break;	//ǰ��
		case 74: Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0; break;	//����
		case 16: Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0; break;	//��ת
		case 90: Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1; break;	//��ת
		case 56: Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0; break;	//ֹͣ
		case 162: mode_flag = 0; break;	//ң��ģʽ
		case 98: mode_flag = 1; break;	//CCDѭ��ģʽ
		case 226: mode_flag = 2; break;	//����������ģʽ
	}
}
