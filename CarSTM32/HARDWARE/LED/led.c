#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "delay.h"


//��ʼ��PA0��PA1Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
 
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��PORTBʱ��
	

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;//PA4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //����Ϊ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOB0,1	
	GPIO_SetBits(GPIOA,GPIO_Pin_4);				 //PB0 �����


}
 
void Led_Test(void)
{
		LED1(ON);
	  Delay_ms(500);
	  LED1(OFF);
	  Delay_ms(500);
		LED1(ON);
	  Delay_ms(500);
		LED1(OFF);
	  Delay_ms(500);
	 
}

void vTask_led(void *pvParameters){
	LED_Init();
	while(1){
		LED1(ON);
		vTaskDelay(500);
		LED1(OFF);
		vTaskDelay(500);		
	}
	
}
