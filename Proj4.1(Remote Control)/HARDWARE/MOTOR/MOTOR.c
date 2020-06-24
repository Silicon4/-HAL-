#include "MOTOR.h"

TIM_HandleTypeDef 	TIM1_Handler = {0};	  	//��ʱ��1��� 
TIM_OC_InitTypeDef 	TIM1_CH14Handler = {0};	//��ʱ��1ͨ��14���


void MiniBalance_Motor_Init(void)		//���4���������ŵĳ�ʼ��
{
	GPIO_InitTypeDef GPIO_Initure = {0};	//������ʼ���ṹ��
	__HAL_RCC_GPIOB_CLK_ENABLE();			//����GPIOBʱ��

	GPIO_Initure.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;	//�˿�����
	GPIO_Initure.Pull = GPIO_NOPULL;
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;	  						//�������
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;					 //50M
	
	HAL_GPIO_Init(GPIOB, &GPIO_Initure);				  //�����趨������ʼ��GPIOB 
}


//TIM1 PWM��ʼ�� 
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
void MiniBalance_PWM_Init(u16 arr,u16 psc)		//�������PWM������ŵĳ�ʼ��
{
	TIM1_Handler.Instance=TIM1;							//��ʱ��1
	TIM1_Handler.Init.Prescaler=psc;					//��ʱ����Ƶ
	TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;	//���ϼ���ģʽ
	TIM1_Handler.Init.Period=arr;		 				//�Զ���װ��ֵ
	TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&TIM1_Handler);	  			 //��ʼ��PWM
	
	TIM1_CH14Handler.OCMode=TIM_OCMODE_PWM1;		 	//ģʽѡ��PWM1
	TIM1_CH14Handler.Pulse= 0;						//���ñȽ�ֵ,��ʼռ�ձ���Ϊ0
	TIM1_CH14Handler.OCPolarity=TIM_OCPOLARITY_HIGH; 	//������� �ߵ�ƽ��Ч 
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH14Handler,TIM_CHANNEL_1);//��ʼ��TIM1��PWMͨ��1��4
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH14Handler,TIM_CHANNEL_4);//��ʼ��TIM1��PWMͨ��1��4
	
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1);	//����PWMͨ��1��4
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_4);	//����PWMͨ��1��4
}

//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()�������� ���ﲢ��ʹ��
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM1)		//��
	{
		__HAL_RCC_TIM1_CLK_ENABLE();            //ʹ��TIM1ʱ��
		HAL_NVIC_SetPriority(TIM1_UP_IRQn,2,2); //���ø���(�������)�жϣ���ռ���ȼ�2�������ȼ�2
		HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);          //����ITM1�����ж�
	}
}

//��ʱ���ײ�������ʱ��ʹ�ܣ���������
//�˺����ᱻHAL_TIM_PWM_Init()����
//htim:��ʱ�����
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	
    if(htim->Instance==TIM1)
	{
		__HAL_RCC_TIM1_CLK_ENABLE();			//ʹ��TIM1ʱ��
		__HAL_AFIO_REMAP_TIM1_PARTIAL();		//TIM1ͨ�����Ų�����ӳ��ʹ��
		__HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
		
		GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_11;           	//PA8 PA11
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�����������,�������PWM
		GPIO_Initure.Pull=GPIO_PULLDOWN;          //����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		HAL_GPIO_Init(GPIOA,&GPIO_Initure); 	
	}
}

//��ʱ��3�жϷ�����
void TIM1_IRQHandler(void)
{
    //HAL_TIM_IRQHandler(&TIM1_Handler);		//����HAL�ⶨʱ���жϴ�����
}
