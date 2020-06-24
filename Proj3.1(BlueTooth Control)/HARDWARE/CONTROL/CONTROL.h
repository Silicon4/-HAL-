#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define ZHONGZHI 0						//ƽ��Ƕ���ֵ����MPU6050�İ�װλ���й�


extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern float Roll;

void Get_Angle(void);					//��ȡ�Ƕ�
int Read_Encoder(u8 TIMX);				//��ȡ�������ֵ

void Xianfu_Pwm(void);					//PWM�޷�

int Get_balance_PWM(float Angle,float Gyro);				//����ƽ��PWMֵ
int Get_velocity_PWM(int encoder_left,int encoder_right);	//�����ٶ�PWMֵ

int Get_BT_turn_PWM(int encoder_left,int encoder_right,float gyro);//����ת�����
int Get_CCD_turn_PWM(u8 CCD,float gyro);  //CCDѭ��ת�����
void Set_Pwm(int moto1,int moto2);
int myabs(int a);						//����ֵ����
void  Find_CCD_Zhongzhi(void);

#endif
