#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"


#define DATA_PIN   PBin(0)		//红外数据输入脚

#define REMOTE_ID 0      		   

extern u8 RmtCnt;	        //按键按下的次数

void Remote_Init(void);     //红外传感器接收头引脚初始化
u8 Remote_Scan(void);
void read_yaokong(void);
#endif
