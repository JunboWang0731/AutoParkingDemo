#ifndef __ENCODER_H_
#define __ENCODER_H_
#include "sys.h"

#define ENCODER_TIM_PERIOD (u16)(65535)   //���ɴ���65535 ��ΪF103�Ķ�ʱ����16λ�ġ�
void Encoder_Init_TIM3(void);
int Read_Encoder(u8 TIMX);
void Encoder_Init_TIM_Exit0(void);
void Encoder_Init_TIM_Exit1(void);


#endif
