#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <iso646.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "sys.h"
#include "usart.h"	
#include "common.h"

#define OLED_MAX_CHAR_POSX 122
#define OLED_MAX_CHAR_POSY 58 

#define LOGGER_START 0xa0
#define WAVE_START 0xa8

#define LOGGER_PRINT_DEVICE ((FILE*)3)

void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);

FILE* screen=(FILE*)4;
FILE* bluetooth=(FILE*)3;

static uint16_t OLED_x = 0;
static uint16_t OLED_y = 0;

static SemaphoreHandle_t xLoggerSemaphore; //串口调用锁

/*
 * 串口初始化函数
 *
 */

void logger_init(){
	void Uart_Init(u16 uart_num);
	//Uart_Init(1);
	//Uart_Init(2);	
	Uart_Init(3);
	
	void USART_Config(USART_TypeDef* TUSARTx,u32 bound);
	//USART_Config(USART1,115200); 
	//USART_Config(USART2,115200); 
	USART_Config(USART3,115200); 
	NVIC_SetPriority(USART3_IRQn,1);
	xLoggerSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive( xLoggerSemaphore );
}

/*
 * 重定向fprintf
 *
 */

int fputc(int ch, FILE *f)
{      
	if(f == stdout){
		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
		USART1->DR = (u8) ch;      
		return ch;
	} else if(f == bluetooth){
		while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
		USART3->DR = (u8) ch;      
		return ch;
	} else if(f == screen){
		if(ch == '\n'){
			OLED_x = 0;
			OLED_y += 16;	
			return ch;			
		}
		if(OLED_x>OLED_MAX_CHAR_POSX){
			OLED_x = 0;
			OLED_y += 16;
		} 
		if(OLED_y>OLED_MAX_CHAR_POSY){
			OLED_y = OLED_x = 0;
			//OLED_Clear();
		}
        OLED_ShowChar(OLED_x,OLED_y,ch,12,1);	 
        OLED_x += 8;
		
		return ch;
	} else {
		return ch;
	}
}

void USART1_IRQHandler(void) {
	uint8_t Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) { 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		Res =USART_ReceiveData(USART1);				
	} 
} 

void USART2_IRQHandler(void) {
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) { 
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	}
}

/*
 * 蓝牙中断
 *
 */

void USART3_IRQHandler(void) {
	void after_receive(uint8_t start_byte,uint8_t* rx_buffer);
    static uint8_t buffer[64];
    static uint8_t start_byte = 0;
    static uint8_t* ptr = 0;	
	static bool is_start = false;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){ 
		USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//关闭串口接受中断
		uint8_t a =USART_ReceiveData(USART3);	
		if ((a >= 0xb1) and (a <= 0xcf) and (a & 0x01)) {  //起始位
			is_start = true;
			ptr = buffer;
			start_byte = a;
		} else if (a == '\0') {  //终止位
			*ptr = '\0';
			ptr = buffer;
			if(is_start){
				after_receive(start_byte,buffer);
			}
			is_start = false;
		} else {
			*ptr = a;
			++ptr;
		}  // if(a=='s')
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
    }
} 
//解析0xc3
static void read_str2float(uint8_t* rx_buffer,float* x,float* y,int* speed){
    uint8_t* p = rx_buffer;
    //x
    float t = 0;
    bool positive = true;
    if(*p=='-'){
        positive = false;
        ++p;
    }else if(*p=='+'){
        ++p;
    }
    while(*p!='.'){
        t = t*10 + *p-'0';
        ++p;
        //t*=10;
    }
    ++p;
    float i = 0.1f;
    while(*p!=' '){
        t += (*p-'0')*i;
        ++p;
        i*=0.1f;
    }
    if(positive){
        *x=t;
    } else {
        *x=-t;
    }
    //y
    while(*p==' '){
        ++p;
    }
    positive = true;
    if(*p=='-'){
        positive = false;
        ++p;
    } else if(*p=='+'){
        ++p;
    }
    t=0;
    while(*p!='.'){
        t = t*10 + *p-'0';
        ++p;
    }
    ++p;
    i = 0.1f;
    while(*p!=' '){
        t += (*p-'0')*i;
        ++p;
        i*=0.1f;
    }
    if(positive){
        *y=t;
    } else {
        *y=-t;
    }
    //speed
    while(*p==' '){
        ++p;
    }
    positive = true;
    if(*p=='-'){
        positive = false;
        ++p;
    }else if(*p=='+'){
        ++p;
    }
    
    int value = 0;
    while(*p!='\0'){
        value = value*10 + (*p-'0');
		p++;
    }
    if(positive){
        *speed=value;
    } else {
        *speed=-value;
    }
}
/*
 * @brief 蓝牙串口读到\0后处理函数
 * 
 * @param start_byte 开始的字节，代表哪一类传输
 * @param rx_buffer 接收的字符串缓冲区
 */
static void after_receive(uint8_t start_byte,uint8_t* rx_buffer){
	void set_speed_angle(int speed,int angle);
	void set_speed_mm_angle(int speed_mm,int angle);
    switch (start_byte) {
	case 0xc1: {
		int direction, speed;
		sscanf((const char *)rx_buffer, "%d %d", &direction, &speed);
		if(speed>10240){//限幅
			speed=10240;
		}
		speed /= 50;
		switch (direction) {
			case 0:
				set_speed_angle(0,0);
				break;
			case 1:
				//car_state[0]=turn_round;
				break;
			case 2:
				set_speed_angle(-speed,0);
				break;
			case 4: 
				//move_left(speed);
				set_speed_angle(speed,-45);
				break;
			case 6: 
				//move_right(speed);
				set_speed_angle(speed,+45);
				break;
			case 7: 
				//turn_right(speed);
				set_speed_angle(speed,+45);
				break;
			case 8: 
				//go_forward(speed);
				set_speed_angle(+speed,0);
				break;
			case 9: 
				//turn_left(speed);
				set_speed_angle(speed,-45);
			default:
				break;
		}
	} break;
	case 0xc3: {
		int  speed;
		float x,y;
		sscanf((const char *)rx_buffer, "%f %f %d", &x, &y, &speed);
		//read_str2float(rx_buffer,&x, &y, &speed);
		int logger_print(const char* str,...);
		//fprintf(screen,"%4f %4f\n",x,y);
		if(speed>10240){//限幅
			speed=10240;
		}
		if(x > 1.0f){
			x = 1.0f;
		} else if (x < -1.0f){
			x = -1.0f;
		}
		if(y > 1.0f){
			y = 1.0f;
		} else if (y < -1.0f){
			y = -1.0f;
		}
		speed /= 50;
        set_speed_angle((int)(speed*y),(int)(45*x));
	} break;
	case 0xc5: {
		int angle , speed;
		sscanf((const char *)rx_buffer, "%d %d", &angle, &speed);
		if(speed>1000) {//限幅
			speed=1000;
		}
		if(angle < -45){
			angle = -45;
		} else if(angle > 45) {
			angle = 45;
		}
		set_speed_mm_angle(speed,angle);
	} break;
	default:
		break;
    }
}

/*
 * 打印信息
 *
 */
int logger_print(const char* str,...){
	int ret = -1;
	va_list ap;
	va_start(ap, str);
	if(xLoggerSemaphore && xSemaphoreTake(xLoggerSemaphore,( TickType_t )1000) == pdTRUE ) {
        fputc(LOGGER_START,LOGGER_PRINT_DEVICE);
        ret = vfprintf(LOGGER_PRINT_DEVICE,str, ap);
        fputc(0,LOGGER_PRINT_DEVICE);
        xSemaphoreGive( xLoggerSemaphore );
    }
	va_end(ap);
	return ret;
}

/*
 * 打印波形
 *
 */
int wave_print(const char* str,...){
	int ret = -1;
	va_list ap;
	va_start(ap, str);
	if(xLoggerSemaphore && xSemaphoreTake(xLoggerSemaphore,( TickType_t )1000) == pdTRUE ) {
        fputc(WAVE_START,LOGGER_PRINT_DEVICE);
        ret = vfprintf(LOGGER_PRINT_DEVICE,str, ap);
        fputc(0,LOGGER_PRINT_DEVICE);
        xSemaphoreGive( xLoggerSemaphore );
    }
	va_end(ap);
	return ret;
}

/*
 * printf测试任务
 *
 */
void vTask_print(void *pvParameters){
	TickType_t  xLastWakeTime = xTaskGetTickCount();
	while(1){
		printf("Hello,world!\n");
		vTaskDelayUntil(&xLastWakeTime,500);
	}
}

