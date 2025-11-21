////////////////////////////////////////////////////////////
//  Project Name: Ballbot Remote stm32f103C6t6程序  PCB制版20251110
//	File Name: 
//  芯片：STM32F103C6T6A	
//  version: V0.0
//	Author: ZJ
//	At:	Xi'an China
//  Date From:20251029
//	**********************************
//	功能说明
//				1.8寸tft-LCD 驱动：ST7735S 显示  SPI1通讯
//
//  			BallBot 遥控器
//
//				////////////////////////////////////////////////////////////
//  Project Name: Ballbot Remote stm32f103C6t6程序  PCB制版20251110
//	File Name: 
//  芯片：STM32F103C6T6A	
//  version: V0.0
//	Author: ZJ
//	At:	Xi'an China
//  Date From:20251029
//	**********************************
//	功能说明
//				1.8寸tft-LCD 驱动：ST7735S 显示  SPI1通讯
//
//  			BallBot 遥控器
//		
// 	date		comment
//  2025-10-29    	初忋化
//	2025-11-13		PCB制版
//	2025-11-14   	转vscode
//
////////////////////////////////////////////////////////////
//==== 引脚连接 =========//
//	1.8寸LCD(ST7735)SPI1					stm32f103c6t6
//		GND										GND
//		Vcc										3.3V
//		SCL	（SPI时钟线）		LCD_SCL		PB13(SPI2)
//		SDA	（SPI数据线）		LCD_SDA		PB15(作为主机向从机LCD发逿)
//		RES （复位）			LCS_RST		PB14
//		DC  （数据与命令切换）	 LCD_RS		 PB12
//		CS   (片选)				LCD_CS		PB11
//		BK	 (背光)				LED_BK		PB10
//
//
//=========================================================
//说明＿
//====开发板===============================================
//		D1(LED1)	<------->		PC13
//=========================================================
// SPI方式选择
// 俿改lcd.h文件＿#define USE_HARDWARE_SPI    0  //1:Enable Hardware SPI;0:USE Soft SPI
//
// 横屏、竖屏选择
// 俿改lcd.h文件＿#define USE_HORIZONTAL  	1	//定义是否使用横屏 		0,不使用.1,使用.
////////////////////////////////////////////////////////////
//====UART1================================================= 
// UART1   57600＿1start＿8bit＿1stop，no parity  
//====UART2================================================= 
// UART2   115200＿1start＿8bit＿1stop，no parity 
//=================================================
#include <stm32f10x.h>
#include "usart.h"		
#include "port.h"
#include "timer.h"
#include "stmflash.h"
#include "delay.h"

#include "exti.h" 
#include "SysTick.h"
#include "stdint.h"			//定义bool变量需要
#include "stdbool.h"		//定义bool变量需要

#include "math.h"

#include "spi.h"

#include "adc.h"

#include "lcd.h"
#include "Picture.h"
#include "gui.h"
//-------------------

//---------------------
extern u8 USART2_RX_BUF[128];     	//接收缓冲,最大128字节.
extern u8 USART2_TX_BUF[128];     	//发送缓冲,最大128字节.
extern u8  USART1_RX_BUF[128];     	//接收缓冲,最大128字节.
extern u8  USART1_TX_BUF[128];     	//发送缓冲,最大128字节.
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;

extern u8 EXIT0_flag,t2_flag;
extern u8 key_press,wk_press;

extern u16 u16ADC_CH0,u16ADC_CH1,u16ADC_CH4,u16ADC_CH5;
extern u16 u16ADC_CH6,u16ADC_CH7,u16ADC_CH8,u16ADC_CH9;
//=====================================================
short Gyro[3],Acc[3];

u16 res,u8Led0_Counter;

float fpitch,froll,fyaw;

u8 i;
u8 canbuf[8],key;
u8 canRXbuf[8];
//==========================================================
int main(void)
{
//------ 系统初始化 ------
	Stm32_Clock_Init(9);     //系统时钟设置
	delay_init(72);
	uart_init1(72,57600);	 //串口1初始化 115200
	uart_init2(36,57600);	 //串口2初忋化 57600 APB1/2预分频
	delay_ms(200);	
	PORT_Init();			 //IO口初始化
	STM32Adc_Init();		 //STM32 ADC 初始化
	delay_ms(200);
	LCD_Init();
	LCD_Clear(BLACK); 		 //清屏
	delay_ms(200);
	Timer2_Init(999,71); 	 //1ms
	delay_ms(100);

	
	//SysTick_init(72,10); 	 //72MHz系统初忋化TickTime时间间隔100ms
	//EXTIX_Init();          //初始化外部输入 
	JTAG_Set(SWD_ENABLE);	 //设置成SWD调试模式，关闭JTAG模式

//--------------------------
	//LED1= 1;		//LED on
	
	delay_ms(100);
	
	//LCD_DrawRectangle(0,0,lcddev.width,lcddev.height);	//??? 
	//LCD_Clear(BLACK);
	LCD_Fill(0,0,lcddev.width,lcddev.height,BLACK);
	Show_Str(5,4,GREEN,BLACK,"ͨµÀ1:",16,1);
	Show_Str(5,24,GREEN,BLACK,"ͨµÀ2:",16,1);
	Show_Str(5,44,GREEN,BLACK,"ͨµÀ3:",16,1);
	Show_Str(5,64,GREEN,BLACK,"ͨµÀ4:",16,1);
	Show_Str(5,84,GREEN,BLACK,"ͨµÀ5:",16,1);
	Show_Str(5,104,GREEN,BLACK,"ͨµÀ6:",16,1);
	delay_ms(100);
	//LCD_BLK = 0;

	//SPI2_ReadWriteByte(0x55);
//--------主循环程序--------
	while(1)
	{	
		//
		//POINT_COLOR=BLACK; 
		//LCD_DrawFillRectangle(60,5,100,25);
		//POINT_COLOR=GREEN;
		if(SW1==0)
		{
		
			LCD_Fill(0,0,lcddev.width,lcddev.height,BLACK);
		}
		if(t2_flag == 1)
		{
			t2_flag = 0;
			LCD_Fill(60,0,100,25,BLACK);
			
			LCD_ShowNum(60,5,u16ADC_CH7,4,16);
			
			LCD_Fill(60,25,100,45,BLACK);
			LCD_ShowNum(60,25,u16ADC_CH8,4,16);
			
			LCD_Fill(60,45,100,65,BLACK);
			LCD_ShowNum(60,45,u16ADC_CH4,4,16);

			LCD_Fill(60,65,100,85,BLACK);
			LCD_ShowNum(60,65,u16ADC_CH5,4,16);

			LCD_Fill(60,85,100,105,BLACK);
			LCD_ShowNum(60,85,u16ADC_CH1,4,16);

			LCD_Fill(60,105,100,125,BLACK);
			LCD_ShowNum(60,105,u16ADC_CH9,4,16);
		}
		
	}   // while(1) end
}		// int main(void) end

