/*****
pwmΪ1000ʱ��500ms���������500
������һȦԼ1438������
����ֱ��64mm
*****/


#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"

#define T 0.156f
#define L 0.1445f
#define K 311.4f

#define PI_DIVIDE_180 (0.0174532925f)

#define RESOLUTION  (1438)
#define MM_PER_R (201)
#define CONTROL_PERIOD (32)

int Read_Encoder(int);

//static int Velocity,Angle;
static int Servo,Target_A,Target_B;



/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/

#define Amplitude 6900   //===PWM������7200 ������6900
#define SERVO_INIT 750	
#define Xianfu_Pwm(Motor_A,Motor_B,Servo) \
	if(Motor_A<-Amplitude) {\
		Motor_A=-Amplitude;	\
	}else if(Motor_A>Amplitude) {\
		Motor_A=Amplitude;\
	}	\
	if(Motor_B<-Amplitude) {\
		Motor_B=-Amplitude;\
	}else if(Motor_B>Amplitude) {\
		Motor_B=Amplitude;\
	}\
	if(Servo<(SERVO_INIT-250)) {\
		Servo=SERVO_INIT-250;\
	} \
	if(Servo>(SERVO_INIT+250)){\
		Servo=SERVO_INIT+250;\
	} \


/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/ 
#define Velocity_KP 8.0f
#define Velocity_KI 8.5f	       //�ٶȿ���PID����
#define Velocity_KP_A 	(Velocity_KP)
#define Velocity_KI_A 	(Velocity_KI)
#define Velocity_KP_B 	(Velocity_KP - 3.0f)
#define Velocity_KI_B 	(Velocity_KI - 2.5f)

static int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	// Bias=Encoder-Target;                //����ƫ��
	Bias = Target - Encoder;                //����ƫ��
	Pwm += Velocity_KP_A * (Bias - Last_bias) + Velocity_KI_A * Bias;   //����ʽPI������
	Last_bias = Bias;	                   //������һ��ƫ�� 
	return Pwm;                         //�������
}
static int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	// Bias=Encoder-Target;                //����ƫ��
	Bias = Target - Encoder;                //����ƫ��
	Pwm += Velocity_KP_B * (Bias - Last_bias) + Velocity_KI_B * Bias;   //����ʽPI������
	Last_bias = Bias;	                   //������һ��ƫ�� 
	return Pwm;                         //�������
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
#define SERVO   TIM1->CCR4  //�������
static inline void Set_Pwm(int motor_a,int motor_b,int servo) {
	if(motor_a<0) {
		PWMA1=7200;
		PWMA2=7200+motor_a;
	} else { 
		PWMA2=7200;
		PWMA1=7200-motor_a;
	}
	if(motor_b<0) {
		PWMB2=7200;
		PWMB1=7200+motor_b;
	} else {
		PWMB1=7200;
		PWMB2=7200-motor_b;
	}
     SERVO=servo;	
}

/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ������ٶȺ�ת��
����  ֵ����
**************************************************************************/
static inline void Kinematic_Analysis(float velocity,float angle)
{
		Target_A=velocity*(1+T*tan(angle)/2/L); 
		Target_B=velocity*(1-T*tan(angle)/2/L);      //���ֲ���
		Servo=SERVO_INIT+angle*K;                    //���ת��   
}

/**************************************************************************
�������ܣ����PWM��ʼ��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
static void Servo_PWM_Init(u16 arr,u16 psc)	
{	 
	RCC->APB2ENR|=1<<11;       //ʹ��TIM1ʱ��    
	RCC->APB2ENR|=1<<2;        //PORTAʱ��ʹ�� 
	GPIOA->CRH&=0XFFFF0FFF;    //PORTA11�������
	GPIOA->CRH|=0X0000B000;    //PORTA11�������
	TIM1->ARR=arr;             //�趨�������Զ���װֵ 
	TIM1->PSC=psc;             //Ԥ��Ƶ������Ƶ
	TIM1->CCMR2|=6<<12;        //CH4 PWM1ģʽ	
	TIM1->CCMR2|=1<<11;        //CH4Ԥװ��ʹ��	   
	TIM1->CCER|=1<<12;         //CH4���ʹ��	   
	TIM1->BDTR |= 1<<15;       //TIM1����Ҫ��仰�������PWM
	TIM1->CR1 = 0x80;           //ARPEʹ�� 
	TIM1->CR1|=0x01;          //ʹ�ܶ�ʱ��1 		
	TIM1->CCR4=750;	
}

static inline void Motor_Gpio_init(void) {
	 GPIO_InitTypeDef GPIO_InitStructure;
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);// 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
   //���ø�����Ϊ�����������,���TIM3 CH3 CH4 PWM���岨��  
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7 |GPIO_Pin_8|GPIO_Pin_9; //TIM3_CH3 //TIM3_CH4
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
}




static inline void PWM2_Init(u16 arr,u16 psc) {		 		
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = arr;							//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc;						//����������Ϊ TIMx ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//���ϼ���
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;					//�ظ��Ĵ����������Զ�����
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 				//�ڳ�ʼ�� TIMx

	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //ʹ�� TIMx �� ARR �ϵ�Ԥװ�ؼĴ���
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //�������ģʽ ͨ��1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;//ʹ�ܻ��������
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset; //���������״̬
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//�����󻥲������״̬
	TIM_OCInitStructure.TIM_Pulse = 0; 							//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //������Ը�
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High; //���û������������
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 			//�۳�ʼ������ TIMx	
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 	//CH1 Ԥװ��ʹ��
	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); 			//�۳�ʼ������ TIMx	
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 	//CH1 Ԥװ��ʹ��
	
	TIM_OC3Init(TIM4, &TIM_OCInitStructure); 			//�۳�ʼ������ TIMx	
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 	//CH1 Ԥװ��ʹ��
	
	TIM_OC4Init(TIM4, &TIM_OCInitStructure); 			//�۳�ʼ������ TIMx	
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 	//CH1 Ԥװ��ʹ��
	
	TIM_Cmd(TIM4, ENABLE); //��ʹ�� TIM2
	
	TIM_CtrlPWMOutputs(TIM4,ENABLE); //��MOE �����ʹ��
	
	TIM_SetAutoreload(TIM4, arr);
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);
	
	return;
} 

/*
 * �����������ʼ��
 *
 */
void motor_init(){
	Servo_PWM_Init(9999,144);   		//=====��ʼ��PWM50HZ���� ���  TIM1
	PWM2_Init(7199,0);  //TIM4
	Motor_Gpio_init();  
	
	void Encoder_Init_TIM3();
	Encoder_Init_TIM3();
	/*void Encoder_Init_TIM_Exit0();
	Encoder_Init_TIM_Exit0();
	void Encoder_Init_TIM_Exit1();
	Encoder_Init_TIM_Exit1();*/
	void Encoder_Init_TIM2(void);
	Encoder_Init_TIM2();
}

/*
 * �����ٶȺͽǶȣ���λ�㣩
 *
 */
void set_speed_angle(int speed,int angle){
	Kinematic_Analysis(speed,angle * PI_DIVIDE_180);
}

/*
 * �����ٶȣ���λmm���ͽǶȣ���λ�㣩
 *
 */
void set_speed_mm_angle(int speed_mm,int angle){
	Kinematic_Analysis(
		speed_mm * RESOLUTION * CONTROL_PERIOD / MM_PER_R /1024,
		angle * PI_DIVIDE_180
	);
}
/*
 * �ֱ������������ٶȺͽǶȣ���λ�㣩
 *
 */
void set_speed2(int speed_left,int speed_right,int angle){
	Target_A = speed_left;
	Target_B = speed_right;
	Servo = SERVO_INIT+angle * PI_DIVIDE_180 * K;                    //���ת��  
}

int logger_print(const char* str,...);
int wave_print(const char* str,...);

/*
 * ���PIDһ�ο��ƺ���
 *
 */
void control_one_time(){
	int Encoder_Left  = Read_Encoder(3);                        //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
	int Encoder_Right = -Read_Encoder(2);
	 
	int Motor_A=Incremental_PI_A(Encoder_Left,Target_A);                   //===�ٶȱջ����Ƽ�����A����PWM
	int Motor_B=Incremental_PI_B(Encoder_Right,Target_B);                  //===�ٶȱջ����Ƽ�����B����PWM 
	Xianfu_Pwm(Motor_A,Motor_B,Servo);                                                      //===PWM�޷�
	Set_Pwm(Motor_B,Motor_A,Servo); 
	static int i = 0;
	if(++i==2){
		wave_print("EncoderA:%d\nEncoderB:%d\n",Encoder_Left,Encoder_Right);	
		i=0;
	}
}

/*
 * ���PID��������
 *
 */
void vTask_motor_control(void *pvParameters){
	motor_init();
	TickType_t  xLastWakeTime = xTaskGetTickCount();
	//Set_Pwm(1000,1000,750); 
	set_speed_angle(0,0);
	while(1){
		//int Encoder_Left  = Read_Encoder(2);                        //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
		//int Encoder_Right = Read_Encoder(3);
		control_one_time();
		
		vTaskDelayUntil(&xLastWakeTime,CONTROL_PERIOD);
	}
}

/*
 * ��PID
 *
 */
void vTask_motor_test(void *pvParameters){
	TickType_t  xLastWakeTime = xTaskGetTickCount();
	while(1){
		set_speed2(-100,+100,0);
		vTaskDelayUntil(&xLastWakeTime,2000);
		set_speed2(+100,-100,0);
		vTaskDelayUntil(&xLastWakeTime,2000);
	}
}
/*
 * ���Ա�����ת��
 *
 */
void vTask_encoder_test(void *pvParameters){
	motor_init();
	TickType_t  xLastWakeTime = xTaskGetTickCount();
	while(1){
		short value = (short)TIM3 -> CNT;
		logger_print("encoder:%d",(int)value);
		vTaskDelayUntil(&xLastWakeTime,500);
	}
}
