/*--------------------------------*/
//���İ����ʹ�ú���ң��ң�أ�CCDѲ�ߣ��������������ں������ռ�ö�ʱ�������޷�ʹ�á�
//�����Ǻ���ң�����ļ�ֵ���û���ַ����0x00
//OK��    0011 1000��56��
//ǰ����  0001 1000��24��
//���ˣ�  0100 1010��74��
//��ת��  0001 0000��16��
//��ת��  0101 1010��90��
//����1��1010 0010��162��ң��ģʽ
//����2��0110 0010��98��CCDѲ��
//����3��1110 0010��226������
/*--------------------------------*/
#include "delay.h"
#include "sys.h"
#include "usart.h"



/*����ȫ�ֱ���*************************************/
u8 Way_Angle=1;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=1; //����ң����صı���
u8 Flag_Stop=1,Flag_Show=0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
u32 Distance = 2000;                               //���������
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send;//��ʱ�͵��εȱ���
float Acceleration_Z;                       //Z����ٶȼ�  
float Balance_Kp=223,Balance_Kd=0.82,Velocity_Kp=83,Velocity_Ki=0.41;//PID����
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������
u16 ADV[128]={0};
u8 CCD_Zhongzhi,CCD_Yuzhi;                 //����CCD���
u8 mode_flag = 0;			//ģʽ�л���־λ��0������ң�أ�1��ѭ��

int main(void)
{
	HAL_Init();
	Stm32_Clock_Init(RCC_CFGR_PLLMULL9);
	JTAG_Set(SWD_ENABLE);
	uart_init(128000);
	MX_USART3_UART_Init();
	delay_init(72);
	KEY_Init();
	LED_Init();
	
	MiniBalance_Motor_Init();
	MiniBalance_PWM_Init(7199,0);   //PWM��������ֵ7200������Ƶ��10KHzƵ�ʣ�100um����
	
	MX_TIM2_Init();
	MX_TIM4_Init();
	
	Remote_Init();
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	
	IIC_Init();
	MPU6050_initialize();
	DMP_Init();
	
	MiniBalance_EXTI_Init();
	
	ccd_Init();
	while(1)
	{
		
	}
}
