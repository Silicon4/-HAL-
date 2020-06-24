#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define ZHONGZHI 0						//平衡角度中值，与MPU6050的安装位置有关


extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern float Roll;

void Get_Angle(void);					//获取角度
int Read_Encoder(u8 TIMX);				//读取编码计数值

void Xianfu_Pwm(void);					//PWM限幅

int Get_balance_PWM(float Angle,float Gyro);				//计算平衡PWM值
int Get_velocity_PWM(int encoder_left,int encoder_right);	//计算速度PWM值

int Get_BT_turn_PWM(int encoder_left,int encoder_right,float gyro);//蓝牙转向控制
int Get_CCD_turn_PWM(u8 CCD,float gyro);  //CCD循迹转向控制
void Set_Pwm(int moto1,int moto2);
int myabs(int a);						//绝对值函数
void  Find_CCD_Zhongzhi(void);

#endif
