#include "CONTROL.h"

int	  Balance_Pwm,Velocity_Pwm,Turn_Pwm;
uint8_t  Flag_Target;
float Roll;

void EXTI15_10_IRQHandler(void) 
{
	EXTI->PR=1<<12;				//����жϱ�־λ   
	Flag_Target=!Flag_Target;	//5msһ�ν�����һ�δ���һ�ε��״̬�ı�
	if(Flag_Target==1)			//5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ�����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
	{
		Get_Angle();			//===������̬
		RD_TSL(); 
		return;
	}									//10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
	Encoder_Left	=	Read_Encoder(2);		//===��ȡ������1��ֵ
	Encoder_Right	=	Read_Encoder(4);		//===��ȡ������2��ֵ 
	Get_Angle();						//===������̬	
	Find_CCD_Zhongzhi();
	read_yaokong();
	//Led_Flash(100);						//===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
	Balance_Pwm		=	Get_balance_PWM(Angle_Balance,Gyro_Balance);	//===ƽ��PID����	
	Velocity_Pwm	=	Get_velocity_PWM(Encoder_Left,Encoder_Right);	//===�ٶȻ�PID����
	switch (mode_flag)
	{
		case 0: Turn_Pwm    = Get_BT_turn_PWM(Encoder_Left,Encoder_Right,Gyro_Turn);break;      //����ң��ת��PD����
		case 1: Turn_Pwm    = Get_CCD_turn_PWM(CCD_Zhongzhi,Gyro_Turn);break;     				//CCDѭ��PD�������
//		case 2: Read_Distane();
//				Turn_Pwm    = Get_BT_turn_PWM(Encoder_Left,Encoder_Right,Gyro_Turn);break;      //����ת��PD����
	}
	Moto1			=	Balance_Pwm - Velocity_Pwm - Turn_Pwm;					//===�������ֵ������PWM
	Moto2			=	Balance_Pwm - Velocity_Pwm + Turn_Pwm;					//===�������ֵ������PWM
	Xianfu_Pwm();						//===PWM�޷�
	Set_Pwm(Moto1,Moto2);				//===��ֵ��PWM�Ĵ���
	return;  
}


/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int Get_balance_PWM(float Angle,float Gyro)
{  
	float Bias;
	int balance;
	Bias=Angle-ZHONGZHI;										   //===���ƽ��ĽǶ���ֵ �ͻ�е���
	balance=Balance_Kp*Bias+Balance_Kd*Gyro;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	return balance;
}


/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ������ң���ٶȣ�����Target_Velocity��
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
**************************************************************************/
int Get_velocity_PWM(int encoder_left,int encoder_right)
{  
	static float Velocity,Encoder_Least,Encoder,Movement;	
	static float Encoder_Integral,Target_Velocity=60;
	
	if (mode_flag == 0)		//�����ң��ģʽ
	{
		if(1==Flag_Qian)    	Movement=Target_Velocity;	      //===ǰ����־λ��1 
		else if(1==Flag_Hou)	Movement=-Target_Velocity;         //===���˱�־λ��1
		else  Movement=0;
	}
	else if (mode_flag == 1)		//�����CCDѭ��ģʽ
	{
		Movement=Target_Velocity + 40;			//����ǰ��ѭ��
	}
	else if (mode_flag == 2)		//����Ǳ���ģʽ
	{
		if(Distance<400)  Movement=-Target_Velocity;
		else
		{
			if(1==Flag_Qian)    	Movement=Target_Velocity;	      //===ǰ����־λ��1 
			else if(1==Flag_Hou)	Movement=-Target_Velocity;         //===���˱�־λ��1
			else  Movement=0;
		}
	}
	else
		Movement = 0;

	//=============�ٶ�PI������=======================//	
	Encoder_Least =(encoder_left+encoder_right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
	Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
	Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
	if(Encoder_Integral>5500)  	Encoder_Integral=5500;             		//===�����޷�
	if(Encoder_Integral<-5500)	Encoder_Integral=-5500;               //===�����޷�	
	Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===�ٶȿ���
	return Velocity;
}

int Get_BT_turn_PWM(int encoder_left,int encoder_right,float gyro)//����ת�����
{
    static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.7,Turn_Count,Kp=42,Kd=0;
	  float Turn_Amplitude=70;    
	  //=============ң��������ת����=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
		{
			if(++Turn_Count==1)
			Encoder_temp=myabs(encoder_left+encoder_right);
			Turn_Convert=50/Encoder_temp;
			if(Turn_Convert<0.4)Turn_Convert=0.4;
			if(Turn_Convert>1)Turn_Convert=1;
		}	
	  else
		{
			Turn_Convert=0.7;
			Turn_Count=0;
			Encoder_temp=0;
		}		
		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;        //===����ת��ң������
		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert;        //===����ת��ң������
		else Turn_Target=0;                                            //===����ת��ң������
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;   //===ת���ٶ��޷�
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.8;                         //===����ת��ң������ֱ�����ߵ�ʱ�����������Ǿ;���    
		else Kd=0.7;                                   
  	//=============ת��PD������=======================//
		Turn=Turn_Target*Kp+gyro*Kd;                 //===���Z�������ǽ���PD����
	  return Turn;
}

int Get_CCD_turn_PWM(u8 CCD,float gyro)  //CCDѭ��ת�����
{
	float Turn;     
  float Bias,kp=35,Kd=0.13;	  
	  Bias=CCD-64;
	 Turn=Bias*kp+gyro*Kd;
	  return Turn;
}

/**************************************************************************
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ����� 
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(void)
{
	Read_DMP();					  //===��ȡ���ٶȡ����ٶȡ����
	Angle_Balance=-Roll;			 //===����ƽ�����
	Gyro_Balance=-gyro[0];			//===����ƽ����ٶ�
	Gyro_Turn=gyro[2];			   //===����ת����ٶ�
	Acceleration_Z=accel[2];		 //===����Z����ٶȼ�
}

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM;	
	switch(TIMX)
	{
		case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;	
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		default:  Encoder_TIM=0;
	}
	return Encoder_TIM;
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	int Amplitude=6900;	//===PWM������7200 ������6900
	if(Moto1<-Amplitude) Moto1=-Amplitude;	
	if(Moto1>Amplitude)  Moto1=Amplitude;	
	if(Moto2<-Amplitude) Moto2=-Amplitude;	
	if(Moto2>Amplitude)  Moto2=Amplitude;		
}



/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	int siqu=400;//��������,���ٵ��
	if(moto1<0)			BIN2=0,			BIN1=1;		//��ת
	else 			  BIN2=1,			BIN1=0;		//��ת
	PWMB=myabs(moto1)+siqu;
	if(moto2<0)	AIN1=0,			AIN2=1;				//��ת
	else		AIN1=1,			AIN2=0;				//��ת
	PWMA=myabs(moto2)+siqu;	
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}

/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u8 i,j,Left,Right,Last_CCD_Zhongzhi;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ
     for(i=5;i<123;i++)   //���߸�ȥ��5����
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //��Сֵ
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //���������������ȡ����ֵ
	 for(i = 5;i<118; i++)   //Ѱ�����������
  {
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
		  Left=i;
		  break;	
		}
  }
	 for(j = 118;j>5; j--)//Ѱ���ұ�������
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//��������λ��
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>70)   //�������ߵ�ƫ����̫��
	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ��
}	




