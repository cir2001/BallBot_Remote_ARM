#ifndef __PORT_H
#define __PORT_H	 
#include "sys.h"
//=============================	   
//====PortA====
#define CH1			PAin(0)			//PA0   	AD_CH1				A-In(ADC_CH0)			Pin14
#define CH2			PAin(1)			//PA1   	AD_CH2				A-In(ADC_CH1)	  	Pin15
									//PA2   	N.U						D-in							Pin16
									//PA3   	N.U						D-in							Pin17
#define CH4			PAin(4)			//PA4   	AD_CH3				A-In(ADC_CH4)			Pin20
#define CH5			PAin(5)			//PA5   	AD_CH4				A-In(ADC_CH5)			Pin21
#define CH6			PAin(6)			//PA6   	AD_CH5				A-In(ADC_CH6)			Pin22
#define CH7			PAin(7)			//PA7   	AD_CH6				A-In(ADC_CH7)			Pin23
									//PA8  	 	LED0					D-Out							Pin41
									//PA9   	UART1_TxD  		D-Out							Pin42
									//PA10  	UART1_RxD  		D-In							Pin43
									//PA11  	N.U						D-In							Pin44
									//PA12  	N.U						D-In							Pin45
									//PA13  	JTMS														Pin46
									//PA14  	JTCK														Pin49
									//PA15  	N.U						D-In							Pin50
//====PortB====
#define CH8			PBin(0)			//PB0			AD_CH8				A-In(ADC-CH8)			Pin26
#define CH9			PBin(1)			//PB1			AD_CH9				A-In(ADC-CH9)			Pin27
									//PB2   	Boot1														Pin28
									//PB3			N.U						D-In							Pin55
#define LESP01 		PBout(4)		//PB4   	N.U						D-Out 							Pin56
									//PB5   	N.U						D-Out							Pin57
#define LADC   		PBout(6)		//PB6   	PWM_OUT_CH3		D-Out							Pin58
#define LHC08   	PBout(7)		//PB7   	PWM_OUT_CH4		D-Out							Pin59	
#define LMain   	PBout(8)		//PB8   	PWM_OUT_CH5		D-Out							Pin62
#define LLCD   		PBout(9)		//PB9   	PWM_OUT_CH6		D-Out							Pin62
#define LCD_BLK   	PBout(10)		//PB10   	PWM_OUT_CH6		D-Out							Pin62
#define LCD_CS  	PBout(11)		//PB11  	N.U						D-Out							Pin30
#define LCD_RS   	PBout(12)		//PB12  	N.U						D-Out							Pin33
#define LCD_SCL  	PBout(13)		//PB13  	N.U						D-Out							Pin34
#define LCD_RST  	PBout(14)		//PB14  	N.U						D-In							Pin35
#define LCD_SDA   	PBout(15)		//PB15  	N.U						D-Out							Pin36

//====PortC====
									//PC0   	N.U						D-In(ADC-CH10)		Pin8
									//PC1   	N.U						D-In(ADC-CH11)		Pin9
									//PC2   	N.U						D-In(ADC-CH12)		Pin10
									//PC3	  	N.U						D-In(ADC-CH13)		Pin11
									//PC4	  	N.U						D-Out							Pin24
									//PC5	  	N.U						D-Out							Pin25
									//PC6	  	N.U						D-Out							Pin37
									//PC7	  	N.U						D-Out							Pin38
									//PC8	  	N.U						D-Out							Pin39
									//PC9	  	N.U						D-In							Pin40
									//PC10 	 	N.U						D-In							Pin51
									//PC11		N.U						D-In							Pin52
									//PC12   	N.U						D-In							Pin53
									//PC13   	N.U						D-In							Pin2
#define SW1  		PCin(14)		//PC14   	N.U						D-In							Pin3
									//PC15   	N.U						D-In							Pin4

//====PortD====
															//PD0    8MHz															Pin5
															//PD1    8MHz															Pin6
															//PD2    LED1						D-Out							Pin54


//液晶控制口置1操作宏定义
#define	LCD_CS_SET  	LCD_CS = 1   
#define	LCD_RS_SET  	LCD_RS = 1   
#define	LCD_SDA_SET  	LCD_SDA = 1   
#define	LCD_SCL_SET  	LCD_SCL = 1   
#define	LCD_RST_SET  	LCD_RST = 1    
#define	LCD_LED_SET  	LCD_BLK = 1

//液晶控制口置0操作宏定义
#define	LCD_CS_CLR  	LCD_CS = 0     
#define	LCD_RS_CLR  	LCD_RS = 0     
#define	LCD_SDA_CLR  	LCD_SDA = 0    
#define	LCD_SCL_CLR  	LCD_SCL = 0  
#define	LCD_RST_CLR  	LCD_RST = 0    
#define	LCD_LED_CLR  	LCD_BLK = 0 

void PORT_Init(void);//IO初始化	 					    
#endif
