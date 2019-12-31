#ifndef ALLTASKS_H
#define ALLTASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "stm32f10x.h"
#include "core_cm3.h"
#ifdef __cplusplus
extern "C" {
#endif

#define ADD_TASK(fun_name, stack_size, priority)                      \
void fun_name (void *pvParameters);                            \
xTaskCreate(fun_name, #fun_name, (stack_size), (void *)0, (priority), \
            (void *)0);

static inline void init(){
	
	void logger_init();
 	logger_init();	
	void LED_Init();
	LED_Init();	
	void KEY_Init();
	KEY_Init();
	void Beep_Init();
	Beep_Init();
	
	void Adc_Init();
	//Adc_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

static inline void create_tasks() {
    //观察程序有没有在运行
    ADD_TASK(vTask_led, 32, 0);
    //ADD_TASK(vTask_print,64,1);
    ADD_TASK(vTask_motor_control,128,3);
    //ADD_TASK(vTask_motor_test,128,1);
	//ADD_TASK(vTask_encoder_test,64,1);
	ADD_TASK(vTask_oled_refresh,64,0);
    
}
#ifdef __cplusplus
}
#endif
#endif

