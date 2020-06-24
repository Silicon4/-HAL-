/*--------------------------------*/
//第四版程序，使用红外遥控遥控，CCD巡线，超声波功能由于红外解码占用定时器，故无法使用。
//下面是红外遥控器的键值，用户地址码是0x00
//OK：    0011 1000（56）
//前进：  0001 1000（24）
//后退：  0100 1010（74）
//左转：  0001 0000（16）
//右转：  0101 1010（90）
//数字1：1010 0010（162）遥控模式
//数字2：0110 0010（98）CCD巡线
//数字3：1110 0010（226）避障
/*--------------------------------*/
#include "delay.h"
#include "sys.h"
#include "usart.h"



/*控制全局变量*************************************/
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=1; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
u32 Distance = 2000;                               //超声波测距
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send;//延时和调参等变量
float Acceleration_Z;                       //Z轴加速度计  
float Balance_Kp=223,Balance_Kd=0.82,Velocity_Kp=83,Velocity_Ki=0.41;//PID参数
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash相关数组
u16 ADV[128]={0};
u8 CCD_Zhongzhi,CCD_Yuzhi;                 //线性CCD相关
u8 mode_flag = 0;			//模式切换标志位，0：蓝牙遥控，1：循迹

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
	MiniBalance_PWM_Init(7199,0);   //PWM输出，溢出值7200，不分频，10KHz频率，100um周期
	
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
