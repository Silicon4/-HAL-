#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"	 

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM2_Init(void);
void MX_TIM4_Init(void);



/* USER CODE BEGIN Prototypes */
int Read_Encoder(u8 TIMX);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

