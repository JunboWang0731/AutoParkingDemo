#include "Control.h"
#include "control_app.h"
#include "encoder.h"
#include "key.h"
u8 time=0;
#define	LED PBout(0)  
u8 led_time=0;
extern u8 delay_50,delay_flag; 
extern char menu;
extern unsigned char flag_scan_ps2;/*
void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);
		led_time++;
		time++;
		if(delay_flag==1)
		 {
			 if(++delay_50==20)	 {delay_50=0,delay_flag=0;}                      //���������ṩ50ms�ľ�׼��ʱ
		 }
		if(led_time==50)
		{
			flag_scan_ps2=1;
			led_time=0;
			LED=~LED;
		}

		if(menu==1)
		{
			//App_control_car();
		}
		
		if(time==4)
		{
			time=0;
			Encoder_Left=-Read_Encoder(3);                        //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
			Encoder_Right=-Read_Encoder(2);
		}
		//TIM_ClearFlag(TIM1, TIM_FLAG_Update); 
	}
}
*/
