#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"			//包含引脚转换的宏函数，便于编写代码

#define PWMA   TIM1->CCR1  //用于控制A电机的PWM占空比调节
#define AIN2   PBout(15)
#define AIN1   PBout(14)

#define PWMB   TIM1->CCR4  //用于控制B电机的PWM占空比调节
#define BIN1   PBout(13)
#define BIN2   PBout(12)

extern TIM_HandleTypeDef 	TIM1_Handler;	  	//定时器1句柄
extern GPIO_InitTypeDef GPIO_Initure;	//声明初始化结构体

void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);

#endif
