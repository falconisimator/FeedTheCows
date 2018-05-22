


/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */


#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stdio.h"
#include "misc.h"
#include <string.h> /* memset */

static u32 sinwave[628]={130,133,135,138,140,143,145,148,150,153,155,158,160,163,165,168,170,172,175,177,179,182,184,186,189,191,193,195,197,199,202,204,206,208,210,212,213,215,217,219,221,222,224,226,227,229,230,232,233,235,236,237,239,240,241,242,243,244,245,246,247,248,249,250,250,251,252,252,253,253,254,254,254,254,255,255,255,255,255,255,255,255,254,254,254,254,253,253,252,252,251,250,250,249,248,247,246,245,244,243,242,241,240,239,238,236,235,234,232,231,229,228,226,224,223,221,219,217,215,214,212,210,208,206,204,202,200,198,195,193,191,189,187,184,182,180,177,175,173,170,168,165,163,160,158,156,153,151,148,145,143,140,138,135,133,130,128,125,123,120,118,115,112,110,107,105,102,100,97,95,92,90,88,85,83,80,78,76,73,71,69,67,64,62,60,58,56,54,52,49,47,46,44,42,40,38,36,34,33,31,29,28,26,25,23,22,20,19,18,16,15,14,13,12,11,10,9,8,7,6,5,5,4,3,3,2,2,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,5,5,6,7,8,9,9,10,11,13,14,15,16,17,19,20,21,23,24,26,27,29,31,32,34,36,38,39,41,43,45,47,49,51,53,55,57,59,62,64,66,68,71,73,75,77,80,82,85,87,89,92,94,97,99,102,104,107,109,112,114,117,119,122,125,127,130,132,135,137,140,142,145,147,150,152,155,157,160,162,165,167,170,172,174,177,179,181,184,186,188,190,193,195,197,199,201,203,205,207,209,211,213,215,217,219,220,222,224,226,227,229,230,232,233,235,236,237,239,240,241,242,243,244,245,246,247,248,249,250,250,251,252,252,253,253,253,254,254,254,255,255,255,255,255,255,255,255,255,254,254,254,253,253,252,252,251,251,250,249,248,247,247,246,245,244,243,241,240,239,238,236,235,234,232,231,229,228,226,225,223,221,219,218,216,214,212,210,208,206,204,202,200,198,196,194,191,189,187,185,182,180,178,175,173,171,168,166,163,161,158,156,153,151,148,146,143,141,138,136,133,131,128,126,123,120,118,115,113,110,108,105,103,100,98,95,93,90,88,86,83,81,78,76,74,71,69,67,65,62,60,58,56,54,52,50,48,46,44,42,40,38,36,35,33,31,30,28,26,25,23,22,21,19,18,17,15,14,13,12,11,10,9,8,7,6,6,5,4,4,3,2,2,2,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,4,5,6,7,7,8,9,10,11,12,14,15,16,17,18,20,21,23,24,26,27,29,30,32,34,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,66,68,70,72,75,77,79,82,84,87,89,91,94,96,99,101,104,106,109,111,114,117,119,122,124,127};

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

ErrorStatus HSEStartUpStatus;


volatile char buffer[50] = {'\0'};
volatile uint16_t ADCBuffer[] = {0xAAAA, 0xAAAA, 0xAAAA};
volatile int m=0;
volatile int omega=0;
volatile int omega_last=0;
volatile float speed;
volatile int position=0;
volatile int position_last=0;
volatile int direction=-1;
volatile int A=0;
volatile int B=0;
volatile int C=0;
volatile int hall_states=0;
volatile int hall_states_last=0;
volatile int motor_enable=0;
volatile int i=0;
volatile int p=0;
volatile int flag=1;

volatile int I_alpha;
volatile int I_beta;

volatile int alpha_mod;
volatile int beta_mod;

volatile int I_d;
volatile int I_q;

volatile int Iq_ref=1000;
volatile int Id_ref=0;

volatile int V_alpha;
volatile int V_beta;

volatile float V_d;
volatile float V_q;

volatile float Kps=3;
volatile float Kis=.5;

volatile float Kpid=-0.1;
volatile float Kiid=-0.03;

volatile float Kpiq=.5;
volatile float Kiiq=0.004;



/*
 * Control sequences
 *
 */

void controller_If(float target)
{

}


int sum=0;
int e=0;

int d=500;
int q=500;


float sp_int=0;
float iq_int=0;
float id_int=0;
int e_speed=0;
int e_id=0;
int e_iq=0;

volatile float target=0;



void controller_dq(float target)
{
	ABC_dq();
	speed=60*(500000/(6*(float)omega))*direction/(23);
	if (speed>5000)
	{
		speed=0;
		omega=40000;
		TIM4->ARR = omega / 52;
	}
	if (speed<-5000)
	{
		speed=0;
		omega=40000;
		TIM4->ARR = omega / 52;

	}

	float dt=omega/52*(1/500000);

	if (flag==1 | 1)
	{
		flag=0;
		float e_speed_last=e_speed;
		e_speed=target-(int)speed;
		if (e_speed>10)
		{
			e_speed=10;
		}
		else if (e_speed<-10)
		{
			e_speed=-10;
		}

		sp_int=sp_int+((e_speed+e_speed_last)/2)*52/100;

		if (sp_int>10000)
		{
			sp_int=10000;
		}
		else if (sp_int<-10000)
		{
			sp_int=-100000;
		}

		Iq_ref=Kps*e_speed+Kis*sp_int;
		if (Iq_ref>3500)
		{
			Iq_ref=3500;
		}
		if (Iq_ref<-3500)
		{
			Iq_ref=-3500;
		}
		//		Iq_ref=500;
	}

	int e_id_last=e_id;
	e_id=Id_ref-I_d;

	id_int=id_int+((e_id+e_id_last)/2)/100;
	if (id_int>100000)
	{
		id_int=100000;
	}
	else if (id_int<-100000)
	{
		id_int=-100000;
	}

	d=Kpid*e_id+Kiid*id_int;

	int e_iq_last=e_iq;
	e_iq=Iq_ref-I_q;

	iq_int=iq_int+((e_iq+e_iq_last)/2)/100;
	if (iq_int>100000)
	{
		iq_int=100000;
	}
	else if (iq_int<-100000)
	{
		iq_int=-100000;
	}

	q=Kpiq*e_iq+Kiiq*iq_int;


	if (d>3500)
	{
		d=3500;
	}
	else if (d<-3500)
	{
		d=-3500;
	}
	if (q>3500)
	{
		q=3500;
	}
	else if (q<-3500)
	{
		q=-3500;
	}


//	d=500;
//	q=2000;



	dq_ABC(d,q);
}

void test(void)
{

}

void Pos_at_zero(void)
{

}




/*
 * Utility Functions
 *
 */

void hall_position(void)
{

}

float meas_current(int channel)
{
	return ADCBuffer[channel];
}

void ABC_dq(void)
{
	I_alpha=meas_current(0)-alpha_mod;
	I_beta=100*(I_alpha+2*(meas_current(1)-beta_mod))/173;

	I_d=I_alpha*(COS_lookup(position/2)-128)/128+I_beta*(SIN_lookup(position/2)-128)/128;
	I_q=I_beta*(COS_lookup(position/2)-128)/128-I_alpha*(SIN_lookup(position/2)-128)/128;


}

void dq_ABC(V_d,V_q)
{
	V_alpha = (((V_d*(COS_lookup(position/2)-128)) - (V_q*(SIN_lookup(position/2)-128)))/4096);
	V_beta = (((V_q*(COS_lookup(position/2)-128)) + (V_d*(SIN_lookup(position/2)-128)))/4096);

	A=V_alpha+128;
	if (A>255)
	{
		A=255;
	}
	else if (A<0)
	{
		A=0;
	}
	//	B=V_beta;
	B=(-V_alpha+((171*V_beta)/100))/2+128;
	if (B>255)
	{
		B=255;
	}
	else if (B<0)
	{
		B=0;
	}
	C=((-V_alpha-((171*V_beta)/100))/2)+128;
	if (C>255)
	{
		C=255;
	}
	else if (C<0)
	{
		C=0;
	}


	//		sprintf(buffer, "\n Alpha: %d Beta: %d A : %d B : %d C : %d \n",V_alpha,V_beta,A,B,C);
	//		USARTSend(buffer, sizeof(buffer));
	//		memset(buffer, ' ', sizeof(buffer));

}

int SIN_lookup(int angle)
{
	while (angle>314)
	{
		angle=angle-314;
	}
	while (angle<0)
	{
		angle=angle+314;
	}

	int value=sinwave[(int)(angle)];
	return value;
}

int COS_lookup(int angle)
{
	angle=angle+78;
	while (angle>314)
	{
		angle=angle-314;
	}
	while (angle<0)
	{
		angle=angle+314;
	}

	int value=sinwave[(int)(angle)];
	return value;
}

void USARTSend(char *pucBuffer, unsigned long ulCount)
{
	//
	// Loop while there are more characters to send.
	//
	while(ulCount--)
	{
		USART_SendData(USART1, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
		/* Loop until the end of transmission */
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		{
		}
	}
}

/*
 * Initialization Functions
 *
 */

void Init_GPIO(void)
{
	/* Configure the GPIOs */
	//GPIO_Configuration();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_0;
	//AFIO_MAPR-> USART1_REMAP

	// input of ADCs for Current sensing
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 ;		// that's ADC1 (PA1 on STM32)
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 ;		// that's ADC1 (PA1 on STM32)
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 ;		// that's ADC1 (PA1 on STM32)
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Initialize Hall effect sensor pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIOB Configuration: Channel 1N, 2N and 3N as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void Init_ADC(void)
{

	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCBuffer;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1 , ENABLE ) ;

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	// we work in continuous sampling mode
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 3;

	ADC_Init ( ADC1, &ADC_InitStructure);	//set config of ADC1

	ADC_RegularChannelConfig(ADC1,ADC_Channel_0, 1,ADC_SampleTime_28Cycles5); // define regular conversion config
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1, 2,ADC_SampleTime_28Cycles5); // define regular conversion config
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2, 3,ADC_SampleTime_28Cycles5); // define regular conversion config


	// enable ADC
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1
	ADC_DMACmd(ADC1 , ENABLE ) ;

	//	ADC calibration (optional, but recommended at power on)
	ADC_ResetCalibration(ADC1);	// Reset previous calibration
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);	// Start new calibration (ADC must be off at that time)
	while(ADC_GetCalibrationStatus(ADC1));

	// start conversion
	ADC_Cmd (ADC1,ENABLE);	//enable ADC1
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	// start conversion (will be endless as we are in continuous mode)
}

void Init_RCC(void)
{
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* ADCCLK = PCLK2/4 */
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08)
		{}
	}

	/* TIM1, GPIOA and GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA |
			RCC_APB2Periph_GPIOB|
			RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);


	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	/* Enable ADC1 and GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE );


	/* Enable USART1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
}

void Init_NVIC(void)
{
	/* NVIC Configuration */
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else /* VECT_TAB_FLASH */
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

	//	/* Enable the USARTx Interrupt */
	//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//	NVIC_Init(&NVIC_InitStructure);

	// Init NVIC
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Timer1_Init(void)
{
	/* TIM1 Configuration ---------------------------------------------------
		Generates 6 complemantary PWM signals with 4 sinusoidal data duty cycles:
		TIM1CLK = 72 MHz, Prescaler = 0, TIM1 counter clock = 72 MHz
		TIM1 frequency = TIM1CLK/(TIM1_Period + 1) =
		Time Base configuration */

	/* TIM1 Peripheral Configuration ----------------------------------------*/
	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 8;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = 256-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* Channel 3 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 10;//127;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	/* Channel 2 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 10;//127;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	/* Channel 1 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 10;//127;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 5;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	/* TIM1 counter enable */
	//clear tim1 interrupt flag
	TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
	//TIM1 interrupt source
	TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);

	/* Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void TIM1_CC_IRQHandler(void)
{

	TIM_OCInitStructure.TIM_Pulse = A;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = B;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = C;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	/* check if array's index reaches the max: 255 */

	TIM_ClearFlag(TIM1,TIM_FLAG_CC1);
}


void Timer3_Init(void)
{
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIMER_InitStructure.TIM_Prescaler = 144;
	TIMER_InitStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM3, 0);
	//TIM_Cmd(TIM3, ENABLE);

	// NVIC Configuration
	// Enable the TIM3_IRQn Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM3, ENABLE);
}

TIM_TimeBaseInitTypeDef TIMER_InitStructure;

void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		omega_last = omega;
		omega = TIM_GetCounter(TIM3);
		TIM_SetCounter(TIM3, 0);

		//
		//		TIMER_InitStructure.TIM_Prescaler = 144;
		//		TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
		//		TIMER_InitStructure.TIM_Prescaler = 144;
		//		TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);

		// Overflow - the motor is stopped

		//omega=60000;
		//		TIM4->ARR = omega / 52;

		//				sprintf(buffer, "The Motor is Stopped\n");
		//				USARTSend(buffer, sizeof(buffer));
		//				memset(buffer, ' ', sizeof(buffer));

	}
}

void Timer4_Init(void)
{
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIMER_InitStructure.TIM_Prescaler = 144;
	TIMER_InitStructure.TIM_Period = 0;
	TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	//TIM_Cmd(TIM4, ENABLE);

	// NVIC Configuration
	// Enable the TIM4_IRQn Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void) {


	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);


		controller_dq(target);



		position=position+2*direction;


		//		if (position>628)
		//		{
		//			position=position-628;
		//		}
		//		if (position>position_last+106)
		//		{
		//			TIM_Cmd(TIM4, DISABLE);
		//		}


		//    	sprintf(buffer, "\r\n%d\r\n", p);
		//    	USARTSend(buffer, sizeof(buffer));
		//    	memset(buffer, ' ', sizeof(buffer));
	}
}

void hallEffect_Init(void)
{

	EXTI_InitTypeDef EXTI_InitStruct;

	// Tell system that you will use PB lines for EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

	// EXTI
	EXTI_InitStruct.EXTI_Line = EXTI_Line0 | EXTI_Line4 | EXTI_Line5;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;

	EXTI_Init(&EXTI_InitStruct);
}

void EXTI0_IRQHandler(void)
{
	if ((EXTI_GetITStatus(EXTI_Line0) ) != RESET)
	{
		//Clear interrupt flag
		EXTI_ClearITPendingBit(EXTI_Line0);


		//    	position=position_last+105;
		//    	position_last=position;
		//    	if (position_last>628)
		//    			{
		//    				position_last=position_last-628;
		//    			}



		hall_states_last=hall_states;
		hall_states = (uint8_t)((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)) | (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)<<1) | (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)<<2));

		if (hall_states !=hall_states_last)
		{
			if (hall_states==3)
			{
				position=0;
				if (hall_states_last==2)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==1)
			{
				position=105;
				if (hall_states_last==3)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==5)
			{
				position=210;
				if (hall_states_last==1)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==4)
			{
				position=314;
				if (hall_states_last==5)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==6)
			{
				position=420;
				if (hall_states_last==4)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==2)
			{
				position=525;
				if (hall_states_last==6)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else
			{
				sprintf(buffer, "\r\n Hall Effect Returns Impossible State: %d \r\n", hall_states);
				USARTSend(buffer, sizeof(buffer));
				memset(buffer, ' ', sizeof(buffer));
			}





			if (omega_last-omega>500)
			{
				omega=omega_last-500;
			}

			TIM_SetCounter(TIM3, 0);
			TIM_Cmd(TIM3, ENABLE);


			TIM_SetCounter(TIM4, 0);
			// Set timer period
			//TIM4->ARR = PMSM_Speed / PMSM_SINTABLESIZE;
			TIM4->ARR = omega / 52; //52 - number of items in the sine table between commutations (192/6 = 32)
			TIM_Cmd(TIM4, ENABLE);


			//		    	sprintf(buffer, "\r\n 4 : %d \r\n", hall_states );
			//		    	    	USARTSend(buffer, sizeof(buffer));
			//		    	    	memset(buffer, ' ', sizeof(buffer));

		}

	}
}

void EXTI4_IRQHandler(void)
{
	if ((EXTI_GetITStatus(EXTI_Line4) ) != RESET)
	{
		// Clear interrupt flag
		EXTI_ClearITPendingBit(EXTI_Line4);



		//    	position=position_last+105;
		//    	position_last=position;
		//    	if (position_last>628)
		//		{
		//			position_last=position_last-628;
		//		}
		hall_states_last=hall_states;
		hall_states = (uint8_t)((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)) | (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)<<1) | (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)<<2));
		if (hall_states !=hall_states_last)
		{
			if (hall_states==3)
			{
				position=0;
				if (hall_states_last==2)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==1)
			{
				position=105;
				if (hall_states_last==3)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==5)
			{
				position=210;
				if (hall_states_last==1)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==4)
			{
				position=314;
				if (hall_states_last==5)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==6)
			{
				position=420;
				if (hall_states_last==4)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==2)
			{
				position=525;
				if (hall_states_last==6)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else
			{
				sprintf(buffer, "\r\n Hall Effect Returns Impossible State: %d \r\n", hall_states);
				USARTSend(buffer, sizeof(buffer));
				memset(buffer, ' ', sizeof(buffer));
			}


			omega_last = omega;
			omega = TIM_GetCounter(TIM3);
			flag=1;

			if (omega_last-omega>500)
			{
				omega=omega_last-500;
			}

			TIM_SetCounter(TIM3, 0);
			TIM_Cmd(TIM3, ENABLE);


			TIM_SetCounter(TIM4, 0);
			// Set timer period
			//TIM4->ARR = PMSM_Speed / PMSM_SINTABLESIZE;
			TIM4->ARR = omega / 52; //52 - number of items in the sine table between commutations (192/6 = 32)
			TIM_Cmd(TIM4, ENABLE);


			//		    	sprintf(buffer, "\r\n 4 : %d \r\n",  hall_states);
			//		    	USARTSend(buffer, sizeof(buffer));
			//		    	memset(buffer, ' ', sizeof(buffer));


		}
	}
}

void EXTI9_5_IRQHandler(void)
{

	if (( EXTI_GetITStatus(EXTI_Line5)) != RESET)
	{
		// Clear interrupt flag
		EXTI_ClearITPendingBit(EXTI_Line5);


		//    	position=position_last+105;
		//    	    	position_last=position;
		//    	    	if (position_last>628)
		//    			{
		//    				position_last=position_last-628;
		//    			}

		hall_states_last=hall_states;
		hall_states = (uint8_t)((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)) | (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)<<1) | (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)<<2));
		if (hall_states !=hall_states_last)
		{
			if (hall_states==3)
			{
				position=0;
				if (hall_states_last==2)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==1)
			{
				position=105;
				if (hall_states_last==3)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==5)
			{
				position=210;
				if (hall_states_last==1)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==4)
			{
				position=314;
				if (hall_states_last==5)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==6)
			{
				position=420;
				if (hall_states_last==4)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else if(hall_states==2)
			{
				position=525;
				if (hall_states_last==6)
				{
					direction=1;
				}
				else
				{
					direction=-1;
				}
			}
			else
			{
				sprintf(buffer, "\r\n Hall Effect Returns Impossible State: %d \r\n", hall_states);
				USARTSend(buffer, sizeof(buffer));
				memset(buffer, ' ', sizeof(buffer));
			}

			omega_last = omega;
			omega = TIM_GetCounter(TIM3);

			flag=1;

			if (omega_last-omega>500)
			{
				omega=omega_last-500;
			}

			TIM_SetCounter(TIM3, 0);
			TIM_Cmd(TIM3, ENABLE);


			TIM_SetCounter(TIM4, 0);
			// Set timer period
			//TIM4->ARR = PMSM_Speed / PMSM_SINTABLESIZE;
			TIM4->ARR = omega / 52; //52 - number of items in the sine table between commutations (192/6 = 32)
			TIM_Cmd(TIM4, ENABLE);

			//		sprintf(buffer, "\r\n Omega = %d \r\n", omega);
			//								USARTSend(buffer, sizeof(buffer));
			//								memset(buffer, ' ', sizeof(buffer));

		}
	}
}

void Init_USART(void)
{

	/* Configure the USART1 */
	//USART_Configuration();
	USART_InitTypeDef USART_InitStructure;

	/* USART1 configuration ------------------------------------------------------*/
	/* USART1 configured as follow:
		          - BaudRate = 115200 baud
		          - Word Length = 8 Bits
		          - One Stop Bit
		          - No parity
		          - Hardware flow control disabled (RTS and CTS signals)
		          - Receive and transmit enabled
		          - USART Clock disabled
		          - USART CPOL: Clock is active low
		          - USART CPHA: Data is captured on the middle
		          - USART LastBit: The clock pulse of the last data bit is not output to
		                           the SCLK pin
	 */
	AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);


	//GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);

	/* Enable the USART1 Receive interrupt: this interrupt is generated when the
			USART1 receive data register is not empty */
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	sprintf(buffer, " USART Initialized\n");
	USARTSend(buffer, sizeof(buffer));



}

void SetSysClockTo72(void)
{
	ErrorStatus HSEStartUpStatus;
	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig( RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency( FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig( RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config( RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config( RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd( ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08)
		{
		}


	}
	else
	{ /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */

		/* Go to infinite loop */
		while (1)
		{
		}
	}
}


/*
 * Main Function
 */

int main(void)
{


	Init_RCC();

	Init_NVIC();

	Init_GPIO();
	Init_ADC();
	//USART1 initialized for debugging
	Init_USART();
	hallEffect_Init();
	Timer3_Init();
	Timer4_Init();

	Timer1_Init();

	alpha_mod=meas_current(0);
	beta_mod=meas_current(1);

	hall_states_last=0;
	hall_states = (uint8_t)((GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)) | (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)<<1) | (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)<<2));

	if (hall_states==3)
	{
		position=0;
	}
	else if(hall_states==1)
	{
		position=105;
	}
	else if(hall_states==5)
	{
		position=210;
	}
	else if(hall_states==4)
	{
		position=314;
	}
	else if(hall_states==6)
	{
		position=420;
	}
	else if(hall_states==2)
	{
		position=525;
	}
	else
	{
		sprintf(buffer, "\r\n Hall Effect Returns Impossible State: %d \r\n", hall_states);
		USARTSend(buffer, sizeof(buffer));
		memset(buffer, ' ', sizeof(buffer));
	}
	flag=1;
	omega=50000;
	TIM_SetCounter(TIM4, 0);
	TIM4->ARR = omega / 52;
	TIM_Cmd(TIM4, ENABLE);

	controller_dq(0);

	target=1500;

	int i_dlast=0;
	int i_qlast=0;
	int i_dlast1=0;
	int i_qlast1=0;

	while (1)
	{
		int aI_d=(I_d+i_dlast+i_dlast1)/3;
		int aI_q=(I_q+i_qlast+i_qlast1)/3;
		sprintf(buffer, "\n s: %d d: %d q: %d iD : %d iQ : %d\n",(int)(speed),d,q,aI_d,aI_q);
		USARTSend(buffer, sizeof(buffer));
		memset(buffer, ' ', sizeof(buffer));
		i_dlast=I_d;
		i_qlast=I_q;
		i_dlast1=i_dlast;
		i_qlast1=i_dlast;

		target=75;



		//		sprintf(buffer, "\n Alpha: %d Beta: %d iD : %d iQ : %d\n",I_alpha,I_beta,I_d,I_q);
		//				USARTSend(buffer, sizeof(buffer));
		//				memset(buffer, ' ', sizeof(buffer));
	}

}

