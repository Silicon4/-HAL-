#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"


#define DATA_PIN   PBin(0)		//�������������

#define REMOTE_ID 0      		   

extern u8 RmtCnt;	        //�������µĴ���

void Remote_Init(void);     //���⴫��������ͷ���ų�ʼ��
u8 Remote_Scan(void);
void read_yaokong(void);
#endif
