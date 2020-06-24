#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"			//��������ת���ĺ꺯�������ڱ�д����

#define PWMA   TIM1->CCR1  //���ڿ���A�����PWMռ�ձȵ���
#define AIN2   PBout(15)
#define AIN1   PBout(14)

#define PWMB   TIM1->CCR4  //���ڿ���B�����PWMռ�ձȵ���
#define BIN1   PBout(13)
#define BIN2   PBout(12)

extern TIM_HandleTypeDef 	TIM1_Handler;	  	//��ʱ��1���
extern GPIO_InitTypeDef GPIO_Initure;	//������ʼ���ṹ��

void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);

#endif
