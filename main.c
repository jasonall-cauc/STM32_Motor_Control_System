// Produced by Jason_cauc
// ��������ԭ��STM32F103ZET6��Ӣ�浥Ƭ�������壬ʹ�ÿ⺯����д��ʹ��ʱ������ԭ���ṩ�ı�׼ͷ�ļ�������Ҫ����LCD1602.h
// �������LCD1602Һ����ʾ��������0-9�����ļ��̣��Լ�L298N����ģ��
// ϵͳ�������ܣ�����L298Nģ�����ӵĵ����ʹ�����ָ����ת��0-360��֮��ĸ���ֵ����ͨ���������������Ƕȣ���ͨ��LCD��ʾ
// ����LCD1602�����Ƽ��̵�����������ʵ�ַ���Ϊͨ���⺯����GPIO���в���

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



 

int key = 0;                                      // ��������ת¼����  
int key_value = 0;                                // ����ֵ�洢������ֵ���ڶ���
int value_t = 0;                                  // ����ֵת¼����
int I9,I8,I7,I6,I5,I4,I3,I2,I1,I0 = 0;            // ������
int to_clear_screen = 0;                          // ��Ҫִ����������־λ�������жϰ�������ʱ�Ƿ�������Ļ
int positon = 0;                                  // λ����Ϣ
int count_i = 0;
int count_j = 0;

u8 keyscan = 0;


u16 ana_val = 0;

float vol_val = 0;

u16 PWM_val = 0;                       // PWM��Χ��0-899��Խ����ռ�ձ�ԽС��PWM_val=1ʱ�����ѹΪ3.47V
                                       // PWM_val = 899ʱ�����ѹΪ0V
float Kp = 9;
float Ki = 0.025;
float Kd = 1.2;
float err, err_integral, err_0 = 0;    // ����źţ���������źţ�ǰ����ź�
float pid_out = 0;                     // ֱ����PID��ʽ�õ������ֵ
float position = 0;                    // ʵ��λ�á�ȡֵ��ΧΪ0-360���Ժ�key_value��Ӧ



static int ana_a = 850;                      //*�ص㣺��������ana_val��ֵ�����β����ֵ������0-4096֮�䡣
static int ana_b = 1260;                      // �������Ժ�������ֵ��Ҫ�ڴ��޸ġ�


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
	
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3 , ENABLE);    // ʹ�ܶ�ʱ��3ʱ��
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO , ENABLE);    // ����ʱ��ʹ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;    // TIM_CH2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);                    // ��������PA7
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;				      // ����PA6
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
   GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	// �˴�������ӳ��
	TIM_TimeBaseStructure.TIM_Period = arr;               //�������Զ���װ������ֵ
  TIM_TimeBaseStructure.TIM_Prescaler =psc;             //����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;              //����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;      //TIM ���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                  //�۳�ʼ�� TIMx
	
	//��ʼ�� TIM3 Channel2 PWM ģʽ
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                //ѡ�� PWM ģʽ 2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;        //������Ը�
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);                         //�ܳ�ʼ������ TIM3 OC2
	
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);                //ʹ��Ԥװ�ؼĴ���
  TIM_Cmd(TIM3, ENABLE);                                           //��ʹ�� TIM3
	
}

//ע�⣺PWM���趨���Զ����С�PWM_valֵ���ڿ���ռ�ձ�
//PWM_valΪ1ʱ�����ѹԼ3.47V��Ϊ899ʱ�����ѹΪ0V
//ʹ�÷������޸�PWM_val,��ʹ��TIM_SetCompare2(TIM3,PWM_val)����
//PWM�ź���PA7�������������ѹ��ΧΪ0-3.47V
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
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOG, ENABLE );		//��GPIOE,F,G

	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_4;				  
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                             // ����PE4,PE6-15Ϊ��������
   GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				  
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                             // ����PG2Ϊ�������� 
   GPIO_Init(GPIOG, &GPIO_InitStructure);

}

void Init_EXTI()
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOG , GPIO_PinSource2 );             // �����ж���2��PG2��ӳ���ϵ
  GPIO_EXTILineConfig (GPIO_PortSourceGPIOE , GPIO_PinSource3 );             // ����PE3(KEY1)���ж���ӳ��
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOE , GPIO_PinSource4 );             // ����PE4(KEY0)���ж���ӳ��
	GPIO_EXTILineConfig (GPIO_PortSourceGPIOE , GPIO_PinSource6 );             // ����PE6(����0)���ж���ӳ��
	
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;               // 2���ж��ߵ��ж����ȼ����ã���ռ���ȼ�1����Ӧ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;               // 3���ж��ߵ��ж����ȼ�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;               // 4���ж��ߵ��ж����ȼ����á�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;               // 6���ж���(5-9)���ж����ȼ����á�
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

/***********�����źż��&ADC��**************/

void ADC_Function_Init()
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 , ENABLE);  //ʹ��ADC1ͨ��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);            //���÷�Ƶ����Ϊ6��ʹ��ʱ��Ƶ��Ϊ72/6=12Hz
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                             
  GPIO_Init(GPIOA, &GPIO_InitStructure);       //��ʼ��PA1
	
	ADC_DeInit (ADC1);                           //ʱ�Ӹ�λ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;     //ADC����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;          //��ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;    //����ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;   //ת�����������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;                //˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1,&ADC_InitStructure);                     //��ʼ������ADC1
	ADC_Cmd(ADC1, ENABLE);	                               //ʹ��ָ����ADC1
	
	ADC_ResetCalibration (ADC1);                           //������λУ׼
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);                            //��λУ׼������ʼADУ׼
	while(ADC_GetCalibrationStatus(ADC1));                       //�ȴ�У׼����

}

u16 Get_ADC_Input(u8 ch)    //ch=1
{
	ADC_RegularChannelConfig (ADC1,ch,1,ADC_SampleTime_55Cycles5);   //ͨ��1���������˳��ֵΪ1������ʱ��55.5����
	ADC_SoftwareStartConvCmd (ADC1, ENABLE);                         //ʹ�����ת������
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) );                   //�ȴ�ת������
	return ADC_GetConversionValue(ADC1);                             //���ؽ��
}
	//ע���������ж��ת��ȡƽ��ֵ�Ĳ����ĺ�����������ʡȥ
	
	
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
	


void ADC_Action()         //ADC������Ԫ��ִ��һ�θ÷�װ�ĺ�������ȡһ�κ�λ��������ص�ana_val��vol_val��Ϣ
{
	ana_val = Get_ADC_Avr (ADC_Channel_1,3);             //ADCģ�����ģ������ֵ���˴�ȡ���2ms����ֵ��ƽ��
	    //  * ȡֵ��Χ�� ana_val (0 , 4096)      vol_val (0 ,3.3)
	// *ver3.24  ָ��0-360���� vol_val(0-3.3) / ana_val(0-4096) �������Զ�Ӧ���������������裬����ֵ�䶯��һ�������Ƶķ�Χ�ڡ���Ҫ�ֶ�����
	
	// *ver3.24 to-do-list���������ʵ������д�������̣�ͨ��Һ����ʾ����ʾ-360����+360��ʱvol_val�ı䶯��Χ
 
}



/*******************  PID Core Zone ***********************/

//pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
//pid_out ��������
//errͨ������ana_val��key_value֮��õ���ע����߲��ǵȼۣ������Ҫ�������㣺ǰ��ֵ��Ϊ0-4096����������ʵ�������ڲ��õ��Ƕ�Ȧ��λ����
//��ֵʵ��������һ���Ӽ�����������������Ӽ�Ϊ[a,b]����key_value��0-360����a-b������سʺ�����ϵ��
//��ana_val��position��ֵ��key_value������key_value�Ͳ��仯�ˡ�
//position = ��(ana_val - a) * 360�� /(b-a).���������ı任��position��key_value�Ͷ���0-360��ֵ������

void Process_Position()    //����λ����Ϣ��ص����ݡ�
{
	ADC_Action();
	//if (ana_val < ana_a)
	//	ana_val = ana_a;
	
	position = ( ( ana_val - ana_a ) * 360 ) / ( ana_b - ana_a );    //���Ǹ�����������λΪ�ȣ�ֵ��Ϊ0��360
	err = key_value - position;                                      //ע��err�����ɸ�
}

void PID_Cal_PWM()
{
	err_integral += err;      
	err_0 = err;	
	pid_out = Kp * err + Ki * err_integral + Kd * (err - err_0 );    //*��Ҫ��PWM_val��ֵ����0-899����pid_out��ֵ��δ֪����Ҫת����
	
	if ( abs(pid_out) >= 899)
		pid_out = 898;                                                 //�˴�pid_outԽ�����PWM����ЧֵԽ�ߣ���PWM_val�෴��

	
	if (err > 0)
	{
	   GPIO_ResetBits(GPIOA, GPIO_Pin_6);                            //���յ�ƽ����������ģ���һ�����ӡ�PA6�͵�ƽ����ת������ת
		PWM_val = 899 - abs(pid_out);
			if (PWM_val < 500)                                               // ver4-27 �޷�����ֹ���ת�����졣
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







/***********�жϳ�����**************/


void EXTI3_IRQHandler(void)          //KEY_1 �����жϺ���
{
	if (EXTI_GetFlagStatus(EXTI_Line3) != RESET)       //��ʵ�ֵĹ����У�value_digit���㣬key_value���㣬
	{
		delay_ms(50);
		key_value = 0;
		value_t = 0;
		LCD1602_Write_Cmd(0x01);	        //  ����
	  LCD1602_Write_Cmd(0x02);          //  ����λ 
		LCD1602_Display0();
		delay_ms(50);
		to_clear_screen = 1;
		
	}
	EXTI_ClearITPendingBit(EXTI_Line3);
}


void EXTI4_IRQHandler(void)          //KEY_0 �����жϺ���
{
	if (EXTI_GetFlagStatus(EXTI_Line4) != RESET)       
	{
		LED1 = 1;
		to_clear_screen = 1;
		count_j = 0;
      
			PWM_val = 899;                             // ��ֹͣ��PWM���Զ��ģ����ֶ�ֹͣ
			TIM_SetCompare2(TIM3,PWM_val);
		
			Process_Position();                        // ����������Ԫ����ȡλ�á��������������PWM����&��PWM�Զ��仯
			PID_Cal_PWM();
			TIM_SetCompare2(TIM3,PWM_val);
			delay_ms(20);
		while( abs(err)>3  )                                           
		{
	//		if (ana_val > 3000 || ana_val < 600)     // ���գ���ֹ��ת�Ƕȹ���
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
					LCD1602_Write_Cmd(0x01);          //  ����λ
			  	LCD1602_Write_Cmd(0x02);          //  ����λ
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
		LCD1602_Write_Cmd(0x01);                //  ����λ
			  	LCD1602_Write_Cmd(0x02);          //  ����λ
				  Display_position();
				  LCD1602_Display_slash();
				  Display_expected();
		delay_ms(500);
		LED1 = 0;
	EXTI_ClearITPendingBit(EXTI_Line4);
	}

}

void EXTI2_IRQHandler(void)             //PG2 �����жϺ���
{
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)      
	{
		LED1 = 1;
		delay_ms(5);
		if (to_clear_screen == 1)
	  {
		  LCD1602_Write_Cmd(0x01);	        //  ����
	    LCD1602_Write_Cmd(0x02);          //  ����λ 
			to_clear_screen = 0;
			key_value = 0;
	  }
		
		Get_Key_Input();                    //�õ�value_tת¼����
		
		Display_Key_Input();                //����ʾ������ʾ��ε�������
		
		key_value = key_value * 10 + value_t;
		
		if (key_value >= 360)
		{
			key_value = 360;                  //  �޷���Ԥ��ֵ���Ϊ360��
			LCD1602_Write_Cmd(0x01);	        //  ����
	    LCD1602_Write_Cmd(0x02);          //  ����λ
			LCD1602_Display3();
			LCD1602_Display6();
			LCD1602_Display0();
		}
		
		delay_ms(200);
		LED1 = 0;
			EXTI_ClearITPendingBit(EXTI_Line2);  //����жϱ�־λ
	}

}

void EXTI9_5_IRQHandler(void)           //����0�ж�
{

	if (EXTI_GetFlagStatus(EXTI_Line6) != RESET)       
	{
			delay_ms(50);
		key_value = key_value * 10;

		if (to_clear_screen == 1)
	  {
		  LCD1602_Write_Cmd(0x01);	        //  ����
	    LCD1602_Write_Cmd(0x02);          //  ����λ 
			to_clear_screen = 0;
			key_value = 0;
	  }
		delay_ms(2);
		  LCD1602_Display0();
				delay_ms(2);
		
		if (key_value >= 360)
		{
			key_value = 360;
			LCD1602_Write_Cmd(0x01);	        //  ����
	    LCD1602_Write_Cmd(0x02);          //  ����λ
			LCD1602_Display3();
			LCD1602_Display6();
			LCD1602_Display0();
		}
		delay_ms(200);
	}
	EXTI_ClearITPendingBit(EXTI_Line6);
}
/**********************************/









/* ������  */

int main()
{
	delay_init();	   	 	  //��ʱ��ʼ�� 
	LED_Init();		  		  //��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();         	//��ʼ���밴�����ӵ�Ӳ���ӿ�
	LED0 = 1;           
	LCD1602_Init();       //ʹ��LCD1602AҺ����ʾģ�飬��ʼ������
	Key_Input_Config();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);//�����ж����ȼ�����Ϊ3:3λ8����ռ���ȼ���0-7����1λ2����Ӧ���ȼ���0-1����������ֵԽ�ͣ�����Խ��

	
	  uart_init(115200);                             //���ڳ�ʼ��Ϊ115200
	  usmart_dev.init(72);                           //��ʼ��USMART
	  delay_ms(500);
	
	  RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO , ENABLE);   //����AFIOʱ��
	  ADC_Function_Init();    // ��ʼ��ADC
	  PWM_Out_Init (899,0);
	

	
	PWM_val = 899;
	TIM_SetCompare2(TIM3, PWM_val);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	Init_EXTI();
	
	LCD1602_Display0();
	
	LED0 = 0;               // ��װ��Ϻ����ƣ����Բ���
			
	while(1)                                   // ������                    *Note:��Ҫbreak�����
	{                                          // ������ while��ʼ
    keyscan = KEY_Scan (0);
    if (keyscan == WKUP_PRES )
	  {
		  ADC_Action();
		  position = ( ( ana_val - ana_a ) * 360 ) / ( ana_b - ana_a );
		  Display_position();
		  delay_ms(1000);
		  LCD1602_Write_Cmd(0x01);	        //  ����
	     LCD1602_Write_Cmd(0x02);          //  ����λ
		  LCD1602_Display0();
		}
    LED1 = 0;
		if (ana_val > 3000 || ana_val < 800)
		{
			PWM_val = 899;
			TIM_SetCompare2(TIM3, PWM_val);
			GPIO_ResetBits(GPIOA, GPIO_Pin_6) ;
		}
		
	}                                          // ������ while����
	
	
}











