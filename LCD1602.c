#include"LCD1602.h"
#include"sys.h"
#include "delay.h"
#include "stdio.h"


void GPIO_Configuration(void)
{
   GPIO_InitTypeDef   GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOF, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOG, ENABLE );		//��GPIOF,G
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				      // ��������PG1������EN
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
   GPIO_Init(GPIOG, &GPIO_InitStructure);	
	
	
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOF, &GPIO_InitStructure);                 // ��������PF0-7, PF13, PF15, ����D0-D7, RW, RS
	
	
	
}

void LCD1602_Wait_Ready(void)
{
	u8 sta;
	
	DATAOUT(0xff);
	LCD_RS_Clr();
	LCD_RW_Set();
	do
	{
		LCD_EN_Set();
		delay_ms(5);	
  		sta = GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_7);
		LCD_EN_Clr();
	}while(sta & 0x80);
}



void LCD1602_Write_Cmd(u8 cmd)
{
	LCD1602_Wait_Ready();
	LCD_RS_Clr();
	LCD_RW_Clr();
	DATAOUT(cmd);
	LCD_EN_Set();
	delay_ms(1);
	LCD_EN_Clr();
}



void LCD1602_Write_Dat(u8 dat)
{
	LCD_RS_Set();
	delay_ms(1);
	LCD_RW_Clr();
	delay_ms(1);
	DATAOUT(dat);
	delay_ms(1);
	LCD_EN_Set();
	delay_ms(1);
	LCD_EN_Clr();
}



void LCD1602_ClearScreen(void)
{
	LCD1602_Write_Cmd(0x01);
}



void LCD1602_Set_Cursor(u8 x, u8 y)
{
	u8 addr;
	
	if (y == 0)
		addr = 0x00 + x;
	else
		addr = 0x40 + x;
	LCD1602_Write_Cmd(addr | 0x80);
}



void LCD1602_Show_Str(u8 x, u8 y, u8 *str)
{
	LCD1602_Set_Cursor(x, y);
	while(*str != '\0')
	{
		LCD1602_Write_Dat(*str++);
	}
}



void LCD1602_Init(void)
{
            GPIO_Configuration();
	LCD1602_Write_Cmd(0x30);	//  8λ���ݽӿڣ�һ����ʾ��5*8�����ַ�
	LCD1602_Write_Cmd(0x0c);  //  ��ʾ���������ʾ�������˸
	LCD1602_Write_Cmd(0x06);	//  д��������ƣ�����ʾ�ƶ� 
	LCD1602_Write_Cmd(0x80);  //  ���õ�ַ00H
	LCD1602_Write_Cmd(0x01);	//  ����
	LCD1602_Write_Cmd(0x02);  //  ����λ
    
}





/*
��������������������������������
��Ȩ����������ΪCSDN������jlp101585����ԭ�����£���ѭ CC 4.0 BY-SA ��ȨЭ�飬ת���븽��ԭ�ĳ������Ӽ���������
ԭ�����ӣ�https://blog.csdn.net/jlp11260/article/details/80573720
*/
