// Produced by Jason_cauc
// 基于正点原子STM32F103ZET6精英版单片机开发板，使用库函数编写。使用时除正点原子提供的标准头文件，还需要包括LCD1602.h
// 外设包括LCD1602液晶显示屏，自制0-9按键的键盘，以及L298N驱动模块
// 系统基本功能：控制L298N模块连接的电机，使其带动指针旋转到0-360度之间的给定值。可通过键盘输入期望角度，并通过LCD显示
// 包含LCD1602和自制键盘的驱动程序，其实现方法为通过库函数对GPIO进行操作

#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"	 	 
#include "usmart.h"
#include "math.h"
#include "stdio.h"
#include "LCD1602.h"
#include "stm32f10x_exti.h"



 

int key = 0;                                      // 按键输入转录变量  
int key_value = 0;                                // 期望值存储变量，值等于度数
int value_t = 0;                                  // 期望值转录变量
int I9,I8,I7,I6,I5,I4,I3,I2,I1,I0 = 0;            // 工具人
int to_clear_screen = 0;                          // “要执行清屏”标志位。用于判断按键输入时是否重置屏幕
int positon = 0;                                  // 位置信息
int count_i = 0;
int count_j = 0;

u8 keyscan = 0;


u16 ana_val = 0;

float vol_val = 0;

u16 PWM_val = 0;                       // PWM范围是0-899，越大则占空比越小。PWM_val=1时输出电压为3.47V
                                       // PWM_val = 899时输出电压为0V
float Kp = 9;
float Ki = 0.025;
float Kd = 1.2;
float err, err_integral, err_0 = 0;    // 误差信号，积分误差信号，前误差信号
float pid_out = 0;                     // 直接由PID公式得到的输出值
float position = 0;                    // 实际位置。取值范围为0-360，以和key_value对应



static int ana_a = 850;                      //*重点：这两个是ana_val的值域的首尾两个值，介于0-4096之间。
static int ana_b = 1260;                      // 经过测试后这两个值需要在此修改。


/* LCD1602 Section   */

#define dig_0 0x30;
#define dig_1 0x31;
#define dig_2 0x32;
#define dig_3 0x33;
#define dig_4 0x34;
#define dig_5 0x35;
#define dig_6 0x36;
#define dig_7 0x37;
#define dig_8 0x38;
#define dig_9 0x39;
int dig_hun = 0;
int dig_ten = 0;
u8 dig_hun_u8 = 0x00;
u8 dig_ten_u8 = 0x00;





/******************* PWM hardware zone **********************/

void PWM_Out_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3 , ENABLE);    // 使能定时器3时钟
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO , ENABLE);    // 复用时钟使能
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;    // TIM_CH2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);                    // 配置引脚PA7
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				      // 配置PA6
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
   GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	// 此处不用重映射
	TIM_TimeBaseStructure.TIM_Period = arr;               //设置在自动重装载周期值
  TIM_TimeBaseStructure.TIM_Prescaler =psc;             //设置预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;              //设置时钟分割:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;      //TIM 向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                  //③初始化 TIMx
	
	//初始化 TIM3 Channel2 PWM 模式
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                //选择 PWM 模式 2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    //比较输出使能
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;        //输出极性高
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);                         //④初始化外设 TIM3 OC2
	
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);                //使能预装载寄存器
  TIM_Cmd(TIM3, ENABLE);                                           //⑤使能 TIM3
	
}

//注意：PWM经设定后自动进行。PWM_val值用于控制占空比
//PWM_val为1时输出电压约3.47V，为899时输出电压为0V
//使用方法：修改PWM_val,再使用TIM_SetCompare2(TIM3,PWM_val)即可
//PWM信号由PA7单引脚输出，电压范围为0-3.47V
//




/******************* display zone ***************************/
void LCD1602_Display0()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 0000
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_ResetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	GPIO_ResetBits(GPIOF, GPIO_Pin_0);
	
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(1);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	delay_ms(1);
	
}
void LCD1602_Display1()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	

	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 0001
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_ResetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	GPIO_SetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display2()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 0010
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_ResetBits(GPIOF, GPIO_Pin_2);
	GPIO_SetBits(GPIOF, GPIO_Pin_1);
	GPIO_ResetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display3()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 0011
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_ResetBits(GPIOF, GPIO_Pin_2);
	GPIO_SetBits(GPIOF, GPIO_Pin_1);
	GPIO_SetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display4()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 0100
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_SetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	GPIO_ResetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display5()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 0101
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_SetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	GPIO_SetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display6()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 0110
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_SetBits(GPIOF, GPIO_Pin_2);
	GPIO_SetBits(GPIOF, GPIO_Pin_1);
	GPIO_ResetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display7()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 0111
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);
	GPIO_SetBits(GPIOF, GPIO_Pin_2);
	GPIO_SetBits(GPIOF, GPIO_Pin_1);
	GPIO_SetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display8()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 1000
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_SetBits(GPIOF, GPIO_Pin_3);
	GPIO_ResetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	GPIO_ResetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display9()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0011 1001
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_SetBits(GPIOF, GPIO_Pin_4);
	GPIO_SetBits(GPIOF, GPIO_Pin_3);
	GPIO_ResetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_1);
	GPIO_SetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
void LCD1602_Display_slash()
{
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
	GPIO_SetBits(GPIOF, GPIO_Pin_15);     // RS = 1
	GPIO_ResetBits(GPIOF, GPIO_Pin_13);   // RW = 0
	
	GPIO_ResetBits(GPIOF, GPIO_Pin_7);    // 0010 1111
	GPIO_ResetBits(GPIOF, GPIO_Pin_6);
	GPIO_SetBits(GPIOF, GPIO_Pin_5);
	GPIO_ResetBits(GPIOF, GPIO_Pin_4);
	GPIO_SetBits(GPIOF, GPIO_Pin_3);
	GPIO_SetBits(GPIOF, GPIO_Pin_2);
	GPIO_SetBits(GPIOF, GPIO_Pin_1);
	GPIO_SetBits(GPIOF, GPIO_Pin_0);
		
	GPIO_SetBits(GPIOG, GPIO_Pin_1);      // E = 1
	delay_ms(2);
	GPIO_ResetBits(GPIOG, GPIO_Pin_1);    // E = 0
}
/********************************************************/




void Key_Input_Config ()
{
	
	GPIO_InitTypeDef   GPIO_InitStructure;

	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOF, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOG, ENABLE );		//打开GPIOE,F,G

	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4;				  
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                             // 设置PE4,PE6-15为上拉输入
   GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				  
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                             // 设置PG2为上拉输入 
   GPIO_Init(GPIOG, &GPIO_InitStructure);

}

void Init_EXTI()
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOG , GPIO_PinSource2 );             // 设置中断线2和PG2的映射关系
  GPIO_EXTILineConfig (GPIO_PortSourceGPIOE , GPIO_PinSource3 );             // 设置PE3(KEY1)的中断线映射
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOE , GPIO_PinSource4 );             // 设置PE4(KEY0)的中断线映射
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOE , GPIO_PinSource6 );             // 设置PE6(输入0)的中断线映射
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line2 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE ;
	EXTI_Init(&EXTI_InitStructure );
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line3 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE ;
	EXTI_Init(&EXTI_InitStructure );
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line4 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE ;
	EXTI_Init(&EXTI_InitStructure );
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line6 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE ;
	EXTI_Init(&EXTI_InitStructure );
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;               // 2号中断线的中断优先级配置：抢占优先级1，响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;               // 3号中断线的中断优先级配置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;               // 4号中断线的中断优先级配置。
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;               // 6号中断线(5-9)的中断优先级配置。
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}








void Get_Key_Input()
{
	value_t = 0;
	
			 I9 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15);
			 I8 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14);
			 I7 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13);
			 I6 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12);
			 I5 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);
			 I4 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10);
			 I3 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9);
			 I2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8);
			 I1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7);
	
			    if (I9 == Bit_RESET)                                              
					 { value_t = 9;}
					else if (I8 == Bit_RESET)
				 	 { value_t = 8;}
					else if (I7 == Bit_RESET)
					 { value_t = 7;}
					else if (I6 == Bit_RESET)
					 { value_t = 6;}
					else if (I5 == Bit_RESET)
					 { value_t = 5;}
					else if (I4 == Bit_RESET)
					 { value_t = 4;}
					else if (I3 == Bit_RESET)
					 { value_t = 3;}
					else if (I2 == Bit_RESET)
					 { value_t = 2;}
					else if (I1 == Bit_RESET)
					 { value_t = 1;}
					else 
					 { value_t = 0;}
	
					 
	 delay_ms(10);
}

void Display_Key_Input()
{
  switch(value_t)
	{
		case 0: LCD1602_Display0(); break;
		case 1: LCD1602_Display1(); break;
		case 2: LCD1602_Display2(); break;
		case 3: LCD1602_Display3(); break;
		case 4: LCD1602_Display4(); break;
		case 5: LCD1602_Display5(); break;
		case 6: LCD1602_Display6(); break;
		case 7: LCD1602_Display7(); break;
		case 8: LCD1602_Display8(); break;
		case 9: LCD1602_Display9(); break;
	}
	delay_ms(100);
}	
void Display_position()
{
	int pos = 0;
  pos = position / 100;
	  switch( pos ) 
	{
		case 0: LCD1602_Display0(); break;
		case 1: LCD1602_Display1(); break;
		case 2: LCD1602_Display2(); break;
		case 3: LCD1602_Display3(); break;
		case 4: LCD1602_Display4(); break;
		case 5: LCD1602_Display5(); break;
		case 6: LCD1602_Display6(); break;
		case 7: LCD1602_Display7(); break;
		case 8: LCD1602_Display8(); break;
		case 9: LCD1602_Display9(); break;
	}
	pos = (int)position % 100;
  	
	  switch(pos / 10)
	{
		case 0: LCD1602_Display0(); break;
		case 1: LCD1602_Display1(); break;
		case 2: LCD1602_Display2(); break;
		case 3: LCD1602_Display3(); break;
		case 4: LCD1602_Display4(); break;
		case 5: LCD1602_Display5(); break;
		case 6: LCD1602_Display6(); break;
		case 7: LCD1602_Display7(); break;
		case 8: LCD1602_Display8(); break;
		case 9: LCD1602_Display9(); break;
	}
	  switch(pos %10)
	{
		case 0: LCD1602_Display0(); break;
		case 1: LCD1602_Display1(); break;
		case 2: LCD1602_Display2(); break;
		case 3: LCD1602_Display3(); break;
		case 4: LCD1602_Display4(); break;
		case 5: LCD1602_Display5(); break;
		case 6: LCD1602_Display6(); break;
		case 7: LCD1602_Display7(); break;
		case 8: LCD1602_Display8(); break;
		case 9: LCD1602_Display9(); break;
	}
}	

void Display_expected()
{
	int exp = 0;
  exp = key_value / 100;
	  switch( exp ) 
	{
		case 0: LCD1602_Display0(); break;
		case 1: LCD1602_Display1(); break;
		case 2: LCD1602_Display2(); break;
		case 3: LCD1602_Display3(); break;
		case 4: LCD1602_Display4(); break;
		case 5: LCD1602_Display5(); break;
		case 6: LCD1602_Display6(); break;
		case 7: LCD1602_Display7(); break;
		case 8: LCD1602_Display8(); break;
		case 9: LCD1602_Display9(); break;
	}
	exp = key_value % 100;
  	
	  switch(exp / 10)
	{
		case 0: LCD1602_Display0(); break;
		case 1: LCD1602_Display1(); break;
		case 2: LCD1602_Display2(); break;
		case 3: LCD1602_Display3(); break;
		case 4: LCD1602_Display4(); break;
		case 5: LCD1602_Display5(); break;
		case 6: LCD1602_Display6(); break;
		case 7: LCD1602_Display7(); break;
		case 8: LCD1602_Display8(); break;
		case 9: LCD1602_Display9(); break;
	}
	  switch(exp %10)
	{
		case 0: LCD1602_Display0(); break;
		case 1: LCD1602_Display1(); break;
		case 2: LCD1602_Display2(); break;
		case 3: LCD1602_Display3(); break;
		case 4: LCD1602_Display4(); break;
		case 5: LCD1602_Display5(); break;
		case 6: LCD1602_Display6(); break;
		case 7: LCD1602_Display7(); break;
		case 8: LCD1602_Display8(); break;
		case 9: LCD1602_Display9(); break;
	}
}	

/***********反馈信号检测&ADC区**************/

void ADC_Function_Init()
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 , ENABLE);  //使能ADC1通道时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);            //设置分频因子为6，使得时钟频率为72/6=12Hz
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                             
  GPIO_Init(GPIOA, &GPIO_InitStructure);       //初始化PA1
	
	ADC_DeInit (ADC1);                           //时钟复位
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;     //ADC独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;          //单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;    //单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;   //转换由软件启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;                //顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1,&ADC_InitStructure);                     //初始化外设ADC1
	ADC_Cmd(ADC1, ENABLE);	                               //使能指定的ADC1
	
	ADC_ResetCalibration (ADC1);                           //开启复位校准
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);                            //复位校准结束后开始AD校准
	while(ADC_GetCalibrationStatus(ADC1));                       //等待校准结束

}

u16 Get_ADC_Input(u8 ch)    //ch=1
{
	ADC_RegularChannelConfig (ADC1,ch,1,ADC_SampleTime_55Cycles5);   //通道1，规则采样顺序值为1，采样时间55.5周期
	ADC_SoftwareStartConvCmd (ADC1, ENABLE);                         //使能软件转换功能
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) );                   //等待转换结束
	return ADC_GetConversionValue(ADC1);                             //返回结果
}
	//注：例程中有多次转换取平均值的操作的函数。本程序省去
	
	
u16 Get_ADC_Avr(u8 ch,u8 times)    //ch=1  
{
  u32 temp_val=0;
  u8 t;
  for(t=0;t<times;t++)
  { 
		temp_val+=Get_ADC_Input(ch);
    delay_ms(2);
  }
  return temp_val/times;
}
	


void ADC_Action()         //ADC动作单元。执行一次该封装的函数，获取一次和位置线性相关的ana_val和vol_val信息
{
	ana_val = Get_ADC_Avr (ADC_Channel_1,3);             //ADC模块给的模拟量的值。此处取相隔2ms两个值的平均
	    //  * 取值范围： ana_val (0 , 4096)      vol_val (0 ,3.3)
	// *ver3.24  指针0-360度与 vol_val(0-3.3) / ana_val(0-4096) 并非线性对应。由于有上拉电阻，其数值变动在一定受限制的范围内。需要手动测量
	
	// *ver3.24 to-do-list：制作完毕实物后，须编写测试例程，通过液晶显示屏显示-360度至+360度时vol_val的变动范围
 
}



/*******************  PID Core Zone ***********************/

//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
//pid_out 有正负。
//err通过计算ana_val和key_value之差得到。注意二者并非等价，因此需要进行折算：前者值域为0-4096的整数，但实际上由于采用的是多圈电位器，
//其值实际上是其一个子集（待测量）。设此子集为[a,b]；则key_value的0-360与其a-b线性相关呈函数关系。
//将ana_val和position的值向key_value靠近，key_value就不变化了。
//position = 【(ana_val - a) * 360】 /(b-a).经过这样的变换，position和key_value就都在0-360的值域上了

void Process_Position()    //处理位置信息相关的内容。
{
	ADC_Action();
	//if (ana_val < ana_a)
	//	ana_val = ana_a;
	
	position = ( ( ana_val - ana_a ) * 360 ) / ( ana_b - ana_a );    //这是个浮点数，单位为度，值域为0到360
	err = key_value - position;                                      //注意err可正可负
}

void PID_Cal_PWM()
{
	err_integral += err;      
	err_0 = err;	
	pid_out = Kp * err + Ki * err_integral + Kd * (err - err_0 );    //*重要：PWM_val的值域是0-899，但pid_out的值域未知，需要转换。
	
	if ( abs(pid_out) >= 899)
		pid_out = 898;                                                 //此处pid_out越大输出PWM波有效值越高，但PWM_val相反。

	
	if (err > 0)
	{
	   GPIO_ResetBits(GPIOA, GPIO_Pin_6);                            //对照电平，接入驱动模块的一个端子。PA6低电平则正转，否则反转
		PWM_val = 899 - abs(pid_out);
			if (PWM_val < 500)                                               // ver4-27 限幅，防止电机转动过快。
        PWM_val = 500;	
	}
	else 
	{
		 GPIO_SetBits(GPIOA, GPIO_Pin_6);
		PWM_val = abs(pid_out);
		if (PWM_val > 450)
			 PWM_val = 450;
	}

	
}







/***********中断程序区**************/


void EXTI3_IRQHandler(void)          //KEY_1 清零中断函数
{
	if (EXTI_GetFlagStatus(EXTI_Line3) != RESET)       //该实现的功能有：value_digit清零，key_value清零，
	{
		delay_ms(50);
		key_value = 0;
		value_t = 0;
		LCD1602_Write_Cmd(0x01);	        //  清屏
	  LCD1602_Write_Cmd(0x02);          //  光标归位 
		LCD1602_Display0();
		delay_ms(50);
		to_clear_screen = 1;
		
	}
	EXTI_ClearITPendingBit(EXTI_Line3);
}


void EXTI4_IRQHandler(void)          //KEY_0 动作中断函数
{
	if (EXTI_GetFlagStatus(EXTI_Line4) != RESET)       
	{
		LED1 = 1;
		to_clear_screen = 1;
		count_j = 0;
      
			PWM_val = 899;                             // 先停止。PWM是自动的，需手动停止
			TIM_SetCompare2(TIM3,PWM_val);
		
			Process_Position();                        // 基本动作单元：获取位置、计算参数、调整PWM参数&令PWM自动变化
			PID_Cal_PWM();
			TIM_SetCompare2(TIM3,PWM_val);
			delay_ms(20);
		while( abs(err)>3  )                                           
		{
	//		if (ana_val > 3000 || ana_val < 600)     // 保险，防止旋转角度过大
	//	   {
	//		   PWM_val = 899;
	//		   TIM_SetCompare2(TIM3, PWM_val);
	//		   GPIO_ResetBits(GPIOA, GPIO_Pin_6) ;
	//	   }
	//		else
			{

			Process_Position();
			PID_Cal_PWM();
			TIM_SetCompare2(TIM3,PWM_val);
			delay_ms(5);
			count_i ++;	
				if(count_i > 20)
				{
					LCD1602_Write_Cmd(0x01);          //  光标归位
			  	LCD1602_Write_Cmd(0x02);          //  光标归位
				  Display_position();
				  LCD1602_Display_slash();
				  Display_expected();
					count_i = 0;
				}
			}
		}
		START:			Process_Position();
			          PID_Cal_PWM();
                if (abs(err) > 3 || count_j < 35)
									goto LOOP;
								else
									goto FINISH;
								
		LOOP:      			Process_Position();
			                PID_Cal_PWM();
			                TIM_SetCompare2(TIM3,PWM_val);
		                	delay_ms(2);
							  	    count_j ++;
			                goto START;
		
		FINISH:   GPIO_ResetBits(GPIOA, GPIO_Pin_6);
								
		PWM_val = 899;
		TIM_SetCompare2(TIM3,PWM_val);					
		LCD1602_Write_Cmd(0x01);                //  光标归位
			  	LCD1602_Write_Cmd(0x02);          //  光标归位
				  Display_position();
				  LCD1602_Display_slash();
				  Display_expected();
		delay_ms(500);
		LED1 = 0;
	EXTI_ClearITPendingBit(EXTI_Line4);
	}

}

void EXTI2_IRQHandler(void)             //PG2 输入中断函数
{
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)      
	{
		LED1 = 1;
		delay_ms(5);
		if (to_clear_screen == 1)
	  {
		  LCD1602_Write_Cmd(0x01);	        //  清屏
	    LCD1602_Write_Cmd(0x02);          //  光标归位 
			to_clear_screen = 0;
			key_value = 0;
	  }
		
		Get_Key_Input();                    //得到value_t转录变量
		
		Display_Key_Input();                //在显示屏上显示这次的输入量
		
		key_value = key_value * 10 + value_t;
		
		if (key_value >= 360)
		{
			key_value = 360;                  //  限幅。预设值最大为360度
			LCD1602_Write_Cmd(0x01);	        //  清屏
	    LCD1602_Write_Cmd(0x02);          //  光标归位
			LCD1602_Display3();
			LCD1602_Display6();
			LCD1602_Display0();
		}
		
		delay_ms(200);
		LED1 = 0;
			EXTI_ClearITPendingBit(EXTI_Line2);  //清除中断标志位
	}

}

void EXTI9_5_IRQHandler(void)           //输入0中断
{

	if (EXTI_GetFlagStatus(EXTI_Line6) != RESET)       
	{
			delay_ms(50);
		key_value = key_value * 10;

		if (to_clear_screen == 1)
	  {
		  LCD1602_Write_Cmd(0x01);	        //  清屏
	    LCD1602_Write_Cmd(0x02);          //  光标归位 
			to_clear_screen = 0;
			key_value = 0;
	  }
		delay_ms(2);
		  LCD1602_Display0();
				delay_ms(2);
		
		if (key_value >= 360)
		{
			key_value = 360;
			LCD1602_Write_Cmd(0x01);	        //  清屏
	    LCD1602_Write_Cmd(0x02);          //  光标归位
			LCD1602_Display3();
			LCD1602_Display6();
			LCD1602_Display0();
		}
		delay_ms(200);
	}
	EXTI_ClearITPendingBit(EXTI_Line6);
}
/**********************************/









/* 主程序  */

int main()
{
	delay_init();	   	 	  //延时初始化 
	LED_Init();		  		  //初始化与LED连接的硬件接口
	KEY_Init();         	//初始化与按键连接的硬件接口
	LED0 = 1;           
	LCD1602_Init();       //使用LCD1602A液晶显示模块，初始化函数
	Key_Input_Config();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);//设置中断优先级分组为3:3位8个抢占优先级（0-7），1位2个响应优先级（0-1）。其中数值越低，级别越高

	
	  uart_init(115200);                             //串口初始化为115200
	  usmart_dev.init(72);                           //初始化USMART
	  delay_ms(500);
	
	  RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO , ENABLE);   //开启AFIO时钟
	  ADC_Function_Init();    // 初始化ADC
	  PWM_Out_Init (899,0);
	

	
	PWM_val = 899;
	TIM_SetCompare2(TIM3, PWM_val);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	Init_EXTI();
	
	LCD1602_Display0();
	
	LED0 = 0;               // 填装完毕后亮灯，可以操作
			
	while(1)                                   // 主程序                    *Note:不要break掉这个
	{                                          // 主程序 while开始
    keyscan = KEY_Scan (0);
    if (keyscan == WKUP_PRES )
	  {
		  ADC_Action();
		  position = ( ( ana_val - ana_a ) * 360 ) / ( ana_b - ana_a );
		  Display_position();
		  delay_ms(1000);
		  LCD1602_Write_Cmd(0x01);	        //  清屏
	     LCD1602_Write_Cmd(0x02);          //  光标归位
		  LCD1602_Display0();
		}
    LED1 = 0;
		if (ana_val > 3000 || ana_val < 800)
		{
			PWM_val = 899;
			TIM_SetCompare2(TIM3, PWM_val);
			GPIO_ResetBits(GPIOA, GPIO_Pin_6) ;
		}
		
	}                                          // 主程序 while结束
	
	
}











