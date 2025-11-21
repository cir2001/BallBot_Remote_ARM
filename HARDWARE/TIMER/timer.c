#include "timer.h"
#include "port.h"
#include "usart.h"
#include "delay.h"
#include "adc.h"
#include "lcd.h"

#include "gui.h"
//==========================================


//*******************************************
extern u8 USART1_TX_BUF[128],USART2_TX_BUF[128];
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;

u8	ADC_ChNum=0,LCD_ChNum;
u8	u8ch0TPt,u8ch1TPt,u8ch4TPt,u8ch5TPt;
u8  u8ch6TPt,u8ch7TPt,u8ch8TPt,u8ch9TPt;


u16	ADDataAdj;
u16 u16ADC_CH0,u16ADC_CH1,u16ADC_CH4,u16ADC_CH5;
u16 u16ADC_CH6,u16ADC_CH7,u16ADC_CH8,u16ADC_CH9;

u16 u16Ch0_data[8],u16Ch1_data[8],u16Ch4_data[8],u16Ch5_data[8];
u16 u16Ch6_data[8],u16Ch7_data[8],u16Ch8_data[8],u16Ch9_data[8];
u16 u16ADCTimer;
int ii=0;

u8 t1,t2,t2_flag;
u32 t,p;

//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时2中断服务程序	 未使用
//===============================  	 
void TIM2_IRQHandler(void)
{
	if(TIM2->SR&0X0001)//溢出中断
	{
		t2++;
		u16ADCTimer++;
		if(t2>=100)
		{
			t2 = 0;
			LMain = !LMain;
			LLCD = !LLCD;
			LESP01 = !LESP01;
			LADC = !LADC;
			LHC08 = !LHC08;
			t2_flag = 1;
		}
		
		//---- STM32 ADC 1ms/次----
		if(u16ADCTimer>=2) 	// 1ms
		{
			u16ADCTimer = 0;
			switch(ADC_ChNum)	{
			case 0:
	//  Temp. ADC ch0
				ADDataAdj = ADC1->DR;	//返回ADC_CurrentX adc值
				u8ch0TPt++;
				u8ch0TPt &= 0x07;
				u16Ch0_data[u8ch0TPt] = ADDataAdj;
				ADDataAdj = (u16Ch0_data[0]+u16Ch0_data[1]+u16Ch0_data[2]+u16Ch0_data[3]
									 + u16Ch0_data[4]+u16Ch0_data[5]+u16Ch0_data[6]+u16Ch0_data[7])>>3;
				u16ADC_CH0 = ADDataAdj;
				Start_STM32Adc(CH1_ADC);	//启动ch5 ADC_CurrentY ADC
				ADC_ChNum = 1;	// 指向下一个通道号
			break;			
			case 1:
	//  Temp. ADC ch1	
				ADDataAdj = ADC1->DR;	//返回ADC_CurrentY adc值
				u8ch1TPt++;
				u8ch1TPt &= 0x07;
				u16Ch1_data[u8ch1TPt] = ADDataAdj;
				ADDataAdj = (u16Ch1_data[0]+u16Ch1_data[1]+u16Ch1_data[2]+u16Ch1_data[3]
									 + u16Ch1_data[4]+u16Ch1_data[5]+u16Ch1_data[6]+u16Ch1_data[7])>>3;
				u16ADC_CH1 = ADDataAdj;
				Start_STM32Adc(CH4_ADC);	//启动ch10 ADC_AdjX ADC
				ADC_ChNum = 4;	// 指向下一个通道号
			break;					
			case 4:
	//  Temp. ADC ch4
				ADDataAdj = ADC1->DR;	//返回ADC_AdjX adc值
				u8ch4TPt++;
				u8ch4TPt &= 0x07;
				u16Ch4_data[u8ch4TPt] = ADDataAdj;
				ADDataAdj = (u16Ch4_data[0]+u16Ch4_data[1]+u16Ch4_data[2]+u16Ch4_data[3]
									 + u16Ch4_data[4]+u16Ch4_data[5]+u16Ch4_data[6]+u16Ch4_data[7])>>3;
				u16ADC_CH4 = ADDataAdj;
				Start_STM32Adc(CH5_ADC);	//启动ch1 ADC_AdjY ADC
				ADC_ChNum = 5;	//指向下一个通道号
			break;	
			case 5:
	//  Temp. ADC ch5	
				ADDataAdj = ADC1->DR;	//返回ADC_AdjY adc值
				u8ch5TPt++;
				u8ch5TPt &= 0x07;
				u16Ch5_data[u8ch5TPt] = ADDataAdj;
				ADDataAdj = (u16Ch5_data[0]+u16Ch5_data[1]+u16Ch5_data[2]+u16Ch5_data[3]
									 + u16Ch5_data[4]+u16Ch5_data[5]+u16Ch5_data[6]+u16Ch5_data[7])>>3;
				u16ADC_CH5 = ADDataAdj;
				Start_STM32Adc(CH6_ADC);	  	//启动ch12 ADC_AdjY ADC
				ADC_ChNum = 6;	// 指向下一个通道号
			break;	
			case 6:
	//  Temp. ADC ch6	
				ADDataAdj = ADC1->DR;	//返回ADC_AdjY adc值
				u8ch6TPt++;
				u8ch6TPt &= 0x07;
				u16Ch6_data[u8ch6TPt] = ADDataAdj;
				ADDataAdj = (u16Ch6_data[0]+u16Ch6_data[1]+u16Ch6_data[2]+u16Ch6_data[3]
									 + u16Ch6_data[4]+u16Ch6_data[5]+u16Ch6_data[6]+u16Ch6_data[7])>>3;
				u16ADC_CH6 = ADDataAdj;
				Start_STM32Adc(CH7_ADC);	  	//启动ch12 ADC_AdjY ADC
				ADC_ChNum = 7;	// 指向下一个通道号
			break;
			case 7:
	//  Temp. ADC ch7	
				ADDataAdj = ADC1->DR;	//返回ADC_AdjY adc值
				u8ch7TPt++;
				u8ch7TPt &= 0x07;
				u16Ch7_data[u8ch7TPt] = ADDataAdj;
				ADDataAdj = (u16Ch7_data[0]+u16Ch7_data[1]+u16Ch7_data[2]+u16Ch7_data[3]
									 + u16Ch7_data[4]+u16Ch7_data[5]+u16Ch7_data[6]+u16Ch7_data[7])>>3;
				u16ADC_CH7 = ADDataAdj;
				Start_STM32Adc(CH8_ADC);	  	//启动ch12 ADC_AdjY ADC
				ADC_ChNum = 8;	// 指向下一个通道号
			break;
			case 8:
	//  Temp. ADC ch8	
				ADDataAdj = ADC1->DR;	//返回ADC_AdjY adc值
				u8ch8TPt++;
				u8ch8TPt &= 0x07;
				u16Ch8_data[u8ch8TPt] = ADDataAdj;
				ADDataAdj = (u16Ch8_data[0]+u16Ch8_data[1]+u16Ch8_data[2]+u16Ch8_data[3]
									 + u16Ch8_data[4]+u16Ch8_data[5]+u16Ch8_data[6]+u16Ch8_data[7])>>3;
				u16ADC_CH8 = ADDataAdj;
				Start_STM32Adc(CH9_ADC);	  	//启动ch12 ADC_AdjY ADC
				ADC_ChNum = 9;	// 指向下一个通道号
			break;
			case 9:
	//  Temp. ADC ch9	
				ADDataAdj = ADC1->DR;	//返回ADC_AdjY adc值
				u8ch9TPt++;
				u8ch9TPt &= 0x07;
				u16Ch9_data[u8ch9TPt] = ADDataAdj;
				ADDataAdj = (u16Ch9_data[0]+u16Ch9_data[1]+u16Ch9_data[2]+u16Ch9_data[3]
									 + u16Ch9_data[4]+u16Ch9_data[5]+u16Ch9_data[6]+u16Ch9_data[7])>>3;
				u16ADC_CH9 = ADDataAdj;
				Start_STM32Adc(CH0_ADC);	  	//启动ch12 ADC_AdjY ADC
				ADC_ChNum = 0;	// 指向下一个通道号
			break;
			default:
				ADC_ChNum = 0;	// 指向第一通道号
			break;	}	
			//Data2ASCII(u16ADC_LX,u16ADC_LY,u16ADC_RX,u16ADC_RY);
			//Data2ASCII_U2(u16ADC_LX,u16ADC_LY,u16ADC_RX,u16ADC_RY);
		}
		
	}
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}
//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时3中断服务程序	 未使用
//===============================  	 
void TIM3_IRQHandler(void)
{
	if(TIM3->SR&0X0001)//溢出中断
	{
	
	}
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时4中断服务程序	 未使用
//===============================  	 
void TIM4_IRQHandler(void)
{
	if(TIM4->SR&0X0001)//溢出中断
	{
	
	}
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}


//=======Timer2 Init ============
//通用定时器初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//===============================
void Timer2_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<0;//TIM2时钟使能    
 	TIM2->ARR=arr;     //设定计数器自动重装值   
	TIM2->PSC=psc;     //预分频器7200,得到10Khz的计数时钟
	//这两个东东要同时设置才可以使用中断
	TIM2->DIER|=1<<0;   //允许更新中断				
	TIM2->DIER|=1<<6;   //允许触发中断	   
 	MY_NVIC_Init(1,3,TIM2_IRQn,2);//抢占1，子优先级1，组2									 
	TIM2->CR1|=0x01;    //使能定时器2
	//TIM2->CR1&=0xfffe;    //禁止定时器2
}
//===============================
//通用定时器PWM初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
//===============================
void PWM_Init_Timer1(u16 arr,u16 psc)
{
	RCC->APB2ENR |= 1 << 11;		//使能TIM1时钟
	//RCC->APB2ENR |= 1 << 2;		//使能TIM1时钟	
	//RCC->APB2ENR |= 1 << 0;		//使能TIM1时钟		
	//配置对应引脚PA0的复用输出功能
	GPIOA->CRH&=0XFFFFF000; 
	GPIOA->CRH|=0X00000BBB; //PA8  	推挽复用输出  PWM1		D-Out		Pin41
													//PA9  	推挽复用输出  PWM2		D-Out		Pin42
													//PA10  推挽复用输出  PWM3		D-Out		Pin43
													//PA11  推挽复用输出 	PWM4		D-Out		Pin44
	GPIOA->ODR |= 1 << 8;				//PA8上拉  ch1
	GPIOA->ODR |= 1 << 9;				//PA9上拉  ch1
	GPIOA->ODR |= 1 << 10;			//PA10上拉  ch1
	//GPIOA->ODR |= 1 << 11;		//PA11上拉 ch4
	//设定计数器自动重装值及是否分频
	TIM1->ARR = arr;			//设定计数器自动重装值-决定PWM的频率
	TIM1->PSC = psc;			//预分频器0为不分频
	//TIM1->BDTR  |= 0xCD;
	TIM1->BDTR  |= 1 << 15;
	//ch1输出使能设置
	//TIM1->CCMR1 |= 7 << 4;//CH1  PWM2模式
	TIM1->CCMR1 |= 6 << 4;//CH1  PWM1模式
	TIM1->CCMR1 |= 1 << 3;//CH1预装载使能
	//ch1输出使能设置
	TIM1->CCER  |= 1 << 0;//输入/捕获1输出使能
	//-----------------
	//TIM1->CCMR1 |= 7 << 12;//CH2  PWM2模式
	TIM1->CCMR1 |= 6 << 12;//CH2  PWM1模式
	TIM1->CCMR1 |= 1 << 11;//CH2预装载使能
	//ch2输出使能设置
	TIM1->CCER  |= 1 << 4;//输入/捕获2输出使能
	//-----------------
	//TIM1->CCMR2 |= 7 << 4;//CH3  PWM2模式
	TIM1->CCMR2 |= 6 << 4;//CH3  PWM1模式
	TIM1->CCMR2 |= 1 << 3;//CH3预装载使能
	//ch3输出使能设置
	TIM1->CCER  |= 1 << 8;//输入/捕获3输出使能
	//ch4输出使能设置
//	TIM1->CCMR2 |= 7 << 12;//CH4  PWM2模式
//	TIM1->CCMR2 |= 1 << 11;//CH4预装载使能
//	TIM1->CCER  |= 1 << 12;//输入/捕获4输出使能
	
	//自动重装载预装载允许位ARPE及定时器使能
	TIM1->CR1 = 0X0080;//ARPE使能
	TIM1->CR1 |= 0X01;//使能定时器1
}




