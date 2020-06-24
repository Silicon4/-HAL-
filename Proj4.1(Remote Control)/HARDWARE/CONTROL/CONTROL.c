#include "CONTROL.h"

int	  Balance_Pwm,Velocity_Pwm,Turn_Pwm;
uint8_t  Flag_Target;
float Roll;

void EXTI15_10_IRQHandler(void) 
{
	EXTI->PR=1<<12;				//清除中断标志位   
	Flag_Target=!Flag_Target;	//5ms一次交错，隔一次触发一次电机状态改变
	if(Flag_Target==1)			//5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
	{
		Get_Angle();			//===更新姿态
		RD_TSL(); 
		return;
	}									//10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
	Encoder_Left	=	Read_Encoder(2);		//===读取编码器1的值
	Encoder_Right	=	Read_Encoder(4);		//===读取编码器2的值 
	Get_Angle();						//===更新姿态	
	Find_CCD_Zhongzhi();
	read_yaokong();
	//Led_Flash(100);						//===LED闪烁;常规模式 1s改变一次指示灯的状态	
	Balance_Pwm		=	Get_balance_PWM(Angle_Balance,Gyro_Balance);	//===平衡PID控制	
	Velocity_Pwm	=	Get_velocity_PWM(Encoder_Left,Encoder_Right);	//===速度环PID控制
	switch (mode_flag)
	{
		case 0: Turn_Pwm    = Get_BT_turn_PWM(Encoder_Left,Encoder_Right,Gyro_Turn);break;      //蓝牙遥控转向环PD控制
		case 1: Turn_Pwm    = Get_CCD_turn_PWM(CCD_Zhongzhi,Gyro_Turn);break;     				//CCD循迹PD方向控制
//		case 2: Read_Distane();
//				Turn_Pwm    = Get_BT_turn_PWM(Encoder_Left,Encoder_Right,Gyro_Turn);break;      //避障转向环PD控制
	}
	Moto1			=	Balance_Pwm - Velocity_Pwm - Turn_Pwm;					//===计算左轮电机最终PWM
	Moto2			=	Balance_Pwm - Velocity_Pwm + Turn_Pwm;					//===计算右轮电机最终PWM
	Xianfu_Pwm();						//===PWM限幅
	Set_Pwm(Moto1,Moto2);				//===赋值给PWM寄存器
	return;  
}


/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int Get_balance_PWM(float Angle,float Gyro)
{  
	float Bias;
	int balance;
	Bias=Angle-ZHONGZHI;										   //===求出平衡的角度中值 和机械相关
	balance=Balance_Kp*Bias+Balance_Kd*Gyro;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}


/**************************************************************************
函数功能：速度PI控制 修改前进后退遥控速度，请修Target_Velocity，
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int Get_velocity_PWM(int encoder_left,int encoder_right)
{  
	static float Velocity,Encoder_Least,Encoder,Movement;	
	static float Encoder_Integral,Target_Velocity=60;
	
	if (mode_flag == 0)		//如果是遥控模式
	{
		if(1==Flag_Qian)    	Movement=Target_Velocity;	      //===前进标志位置1 
		else if(1==Flag_Hou)	Movement=-Target_Velocity;         //===后退标志位置1
		else  Movement=0;
	}
	else if (mode_flag == 1)		//如果是CCD循迹模式
	{
		Movement=Target_Velocity + 40;			//高速前进循迹
	}
	else if (mode_flag == 2)		//如果是避障模式
	{
		if(Distance<400)  Movement=-Target_Velocity;
		else
		{
			if(1==Flag_Qian)    	Movement=Target_Velocity;	      //===前进标志位置1 
			else if(1==Flag_Hou)	Movement=-Target_Velocity;         //===后退标志位置1
			else  Movement=0;
		}
	}
	else
		Movement = 0;

	//=============速度PI控制器=======================//	
	Encoder_Least =(encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
	Encoder *= 0.7;		                                                //===一阶低通滤波器       
	Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
	Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
	if(Encoder_Integral>5500)  	Encoder_Integral=5500;             		//===积分限幅
	if(Encoder_Integral<-5500)	Encoder_Integral=-5500;               //===积分限幅	
	Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制
	return Velocity;
}

int Get_BT_turn_PWM(int encoder_left,int encoder_right,float gyro)//蓝牙转向控制
{
    static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.7,Turn_Count,Kp=42,Kd=0;
	  float Turn_Amplitude=70;    
	  //=============遥控左右旋转部分=======================//
  	if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
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
		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;        //===接收转向遥控数据
		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert;        //===接收转向遥控数据
		else Turn_Target=0;                                            //===接收转向遥控数据
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;   //===转向速度限幅
		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.8;                         //===接收转向遥控数据直立行走的时候增加陀螺仪就纠正    
		else Kd=0.7;                                   
  	//=============转向PD控制器=======================//
		Turn=Turn_Target*Kp+gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
	  return Turn;
}

int Get_CCD_turn_PWM(u8 CCD,float gyro)  //CCD循迹转向控制
{
	float Turn;     
  float Bias,kp=35,Kd=0.13;	  
	  Bias=CCD-64;
	 Turn=Bias*kp+gyro*Kd;
	  return Turn;
}

/**************************************************************************
函数功能：获取角度 三种算法经过我们的调校，都非常理想 
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(void)
{
	Read_DMP();					  //===读取加速度、角速度、倾角
	Angle_Balance=-Roll;			 //===更新平衡倾角
	Gyro_Balance=-gyro[0];			//===更新平衡角速度
	Gyro_Turn=gyro[2];			   //===更新转向角速度
	Acceleration_Z=accel[2];		 //===更新Z轴加速度计
}

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
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
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	int Amplitude=6900;	//===PWM满幅是7200 限制在6900
	if(Moto1<-Amplitude) Moto1=-Amplitude;	
	if(Moto1>Amplitude)  Moto1=Amplitude;	
	if(Moto2<-Amplitude) Moto2=-Amplitude;	
	if(Moto2>Amplitude)  Moto2=Amplitude;		
}



/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
	int siqu=400;//死区补偿,减速电机
	if(moto1<0)			BIN2=0,			BIN1=1;		//正转
	else 			  BIN2=1,			BIN1=0;		//反转
	PWMB=myabs(moto1)+siqu;
	if(moto2<0)	AIN1=0,			AIN2=1;				//正转
	else		AIN1=1,			AIN2=0;				//反转
	PWMA=myabs(moto2)+siqu;	
}

/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u8 i,j,Left,Right,Last_CCD_Zhongzhi;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
	 for(i = 5;i<118; i++)   //寻找左边跳变沿
  {
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
		  Left=i;
		  break;	
		}
  }
	 for(j = 118;j>5; j--)//寻找右边跳变沿
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>70)   //计算中线的偏差，如果太大
	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}	




