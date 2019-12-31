/*****
pwm为1000时，500ms编码器测得500
编码器一圈约1438个计数
轮子直径64mm
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
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/

#define Amplitude 6900   //===PWM满幅是7200 限制在6900
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
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/ 
#define Velocity_KP 8.0f
#define Velocity_KI 8.5f	       //速度控制PID参数
#define Velocity_KP_A 	(Velocity_KP)
#define Velocity_KI_A 	(Velocity_KI)
#define Velocity_KP_B 	(Velocity_KP - 3.0f)
#define Velocity_KI_B 	(Velocity_KI - 2.5f)

static int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	// Bias=Encoder-Target;                //计算偏差
	Bias = Target - Encoder;                //计算偏差
	Pwm += Velocity_KP_A * (Bias - Last_bias) + Velocity_KI_A * Bias;   //增量式PI控制器
	Last_bias = Bias;	                   //保存上一次偏差 
	return Pwm;                         //增量输出
}
static int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	// Bias=Encoder-Target;                //计算偏差
	Bias = Target - Encoder;                //计算偏差
	Pwm += Velocity_KP_B * (Bias - Last_bias) + Velocity_KI_B * Bias;   //增量式PI控制器
	Last_bias = Bias;	                   //保存上一次偏差 
	return Pwm;                         //增量输出
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
#define SERVO   TIM1->CCR4  //舵机引脚
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
函数功能：小车运动数学模型
入口参数：速度和转角
返回  值：无
**************************************************************************/
static inline void Kinematic_Analysis(float velocity,float angle)
{
		Target_A=velocity*(1+T*tan(angle)/2/L); 
		Target_B=velocity*(1-T*tan(angle)/2/L);      //后轮差速
		Servo=SERVO_INIT+angle*K;                    //舵机转向   
}

/**************************************************************************
函数功能：舵机PWM初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
static void Servo_PWM_Init(u16 arr,u16 psc)	
{	 
	RCC->APB2ENR|=1<<11;       //使能TIM1时钟    
	RCC->APB2ENR|=1<<2;        //PORTA时钟使能 
	GPIOA->CRH&=0XFFFF0FFF;    //PORTA11复用输出
	GPIOA->CRH|=0X0000B000;    //PORTA11复用输出
	TIM1->ARR=arr;             //设定计数器自动重装值 
	TIM1->PSC=psc;             //预分频器不分频
	TIM1->CCMR2|=6<<12;        //CH4 PWM1模式	
	TIM1->CCMR2|=1<<11;        //CH4预装载使能	   
	TIM1->CCER|=1<<12;         //CH4输出使能	   
	TIM1->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
	TIM1->CR1 = 0x80;           //ARPE使能 
	TIM1->CR1|=0x01;          //使能定时器1 		
	TIM1->CCR4=750;	
}

static inline void Motor_Gpio_init(void) {
	 GPIO_InitTypeDef GPIO_InitStructure;
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);// 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
   //设置该引脚为复用输出功能,输出TIM3 CH3 CH4 PWM脉冲波形  
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7 |GPIO_Pin_8|GPIO_Pin_9; //TIM3_CH3 //TIM3_CH4
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
}




static inline void PWM2_Init(u16 arr,u16 psc) {		 		
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = arr;							//设置在下一个更新事件装入活动的自动重装载寄存器周期的值 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc;						//设置用来作为 TIMx 时钟频率除数的预分频值 不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 					//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//向上计数
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;					//重复寄存器，用于自动更新
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 				//②初始化 TIMx

	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //使能 TIMx 在 ARR 上的预装载寄存器
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //脉宽调制模式 通道1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;//使能互补端输出
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset; //死区后输出状态
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//死区后互补端输出状态
	TIM_OCInitStructure.TIM_Pulse = 0; 							//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性高
	TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_High; //设置互补端输出极性
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 			//③初始化外设 TIMx	
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 	//CH1 预装载使能
	
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); 			//③初始化外设 TIMx	
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 	//CH1 预装载使能
	
	TIM_OC3Init(TIM4, &TIM_OCInitStructure); 			//③初始化外设 TIMx	
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 	//CH1 预装载使能
	
	TIM_OC4Init(TIM4, &TIM_OCInitStructure); 			//③初始化外设 TIMx	
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 	//CH1 预装载使能
	
	TIM_Cmd(TIM4, ENABLE); //④使能 TIM2
	
	TIM_CtrlPWMOutputs(TIM4,ENABLE); //⑤MOE 主输出使能
	
	TIM_SetAutoreload(TIM4, arr);
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);
	
	return;
} 

/*
 * 电机编码器初始化
 *
 */
void motor_init(){
	Servo_PWM_Init(9999,144);   		//=====初始化PWM50HZ驱动 舵机  TIM1
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
 * 设置速度和角度（单位°）
 *
 */
void set_speed_angle(int speed,int angle){
	Kinematic_Analysis(speed,angle * PI_DIVIDE_180);
}

/*
 * 设置速度（单位mm）和角度（单位°）
 *
 */
void set_speed_mm_angle(int speed_mm,int angle){
	Kinematic_Analysis(
		speed_mm * RESOLUTION * CONTROL_PERIOD / MM_PER_R /1024,
		angle * PI_DIVIDE_180
	);
}
/*
 * 分别设置左右轮速度和角度（单位°）
 *
 */
void set_speed2(int speed_left,int speed_right,int angle){
	Target_A = speed_left;
	Target_B = speed_right;
	Servo = SERVO_INIT+angle * PI_DIVIDE_180 * K;                    //舵机转向  
}

int logger_print(const char* str,...);
int wave_print(const char* str,...);

/*
 * 电机PID一次控制函数
 *
 */
void control_one_time(){
	int Encoder_Left  = Read_Encoder(3);                        //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
	int Encoder_Right = -Read_Encoder(2);
	 
	int Motor_A=Incremental_PI_A(Encoder_Left,Target_A);                   //===速度闭环控制计算电机A最终PWM
	int Motor_B=Incremental_PI_B(Encoder_Right,Target_B);                  //===速度闭环控制计算电机B最终PWM 
	Xianfu_Pwm(Motor_A,Motor_B,Servo);                                                      //===PWM限幅
	Set_Pwm(Motor_B,Motor_A,Servo); 
	static int i = 0;
	if(++i==2){
		wave_print("EncoderA:%d\nEncoderB:%d\n",Encoder_Left,Encoder_Right);	
		i=0;
	}
}

/*
 * 电机PID控制任务
 *
 */
void vTask_motor_control(void *pvParameters){
	motor_init();
	TickType_t  xLastWakeTime = xTaskGetTickCount();
	//Set_Pwm(1000,1000,750); 
	set_speed_angle(0,0);
	while(1){
		//int Encoder_Left  = Read_Encoder(2);                        //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
		//int Encoder_Right = Read_Encoder(3);
		control_one_time();
		
		vTaskDelayUntil(&xLastWakeTime,CONTROL_PERIOD);
	}
}

/*
 * 调PID
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
 * 测试编码器转速
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
