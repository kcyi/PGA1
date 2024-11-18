/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/25
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *Multiprocessor communication mode routine:
 *Master:USART1_Tx(PD5)\USART1_Rx(PD6).
 *This routine demonstrates that USART1 receives the data sent by CH341 and inverts
 *it and sends it (baud rate 115200).
 *
 *Hardware connection:PD5 -- Rx
 *                     PD6 -- Tx
 *
 */

#include "debug.h"
#include <stdint.h>
//#include "HT16K33_driver.h"
//#include "grove_alphanumeric_display.h"

#include "SSD1306.h"

/* Global define */
	/* PWM Output Mode Definition */
	#define PWM_MODE1   0
	#define PWM_MODE2   1
	/* PWM Output Mode Selection */
	#define PWM_MODE PWM_MODE1
	//#define PWM_MODE PWM_MODE2

	#define DEBOUNCE_TIME 1000 //ms
	#define TIMER_INIT 100

/* Global Variable */
vu8 val;
uint8_t interruptFlag = 0;
uint8_t re_A, re_Atemp, re_B, re_Btemp;
volatile uint32_t last_interrupt_time = 0;

int E_dir = 0;


/*********************************************************************
 * @fn      GPIO_Config
 *
 * @brief   Initializes GPIOA.0
 *
 * @return  none
 */
void GPIO_Config(void) {
	GPIO_InitTypeDef GPIO_InitStructure = { 0 };

	EXTI_InitTypeDef EXTI_InitStructure = { 0 };
	NVIC_InitTypeDef NVIC_InitStructure = { 0 };


	// LED Output Test
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // GPIO_Pin_0 ;
//
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);

	//Switch Input - interrupt
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // Defines which Pin to configure
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Defines Output Type
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//Switch Input - interrupt
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 ; // Defines which Pin to configure
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Defines Output Type
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* GPIOD ----> EXTI_Line0 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,
			GPIO_PinSource0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,
				GPIO_PinSource5);
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,
			GPIO_PinSource7);
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/*********************************************************************
 * @fn      TIM1_OutCompare_Init
 *
 * @brief   Initializes TIM1 output compare.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void PC0_T2CH3_PWMOut(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO , ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

#elif (PWM_MODE == PWM_MODE2)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // phase reverse

#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    //PC0 T2CH3
    TIM_OC3Init( TIM2, &TIM_OCInitStructure );
    TIM_CtrlPWMOutputs(TIM2, ENABLE );
    TIM_OC3PreloadConfig( TIM2, TIM_OCPreload_Enable ); //TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM2, ENABLE );
    TIM_Cmd( TIM2, ENABLE );
}

void PD6_T2CH3_BlueOut(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO , ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    // Remap PD6
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

#elif (PWM_MODE == PWM_MODE2)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // phase reverse

#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC3Init( TIM2, &TIM_OCInitStructure );
    TIM_CtrlPWMOutputs(TIM2, ENABLE );
    TIM_OC3PreloadConfig( TIM2, TIM_OCPreload_Enable ); //TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM2, ENABLE );
    TIM_Cmd( TIM2, ENABLE );
}

void PD5_T2CH4_GreenOut(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO , ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    // Remap PD6
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

#elif (PWM_MODE == PWM_MODE2)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // phase reverse

#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC4Init( TIM2, &TIM_OCInitStructure );
    TIM_CtrlPWMOutputs(TIM2, ENABLE );
    TIM_OC4PreloadConfig( TIM2, TIM_OCPreload_Enable ); //TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM2, ENABLE );
    TIM_Cmd( TIM2, ENABLE );
}

void PC3_T1CH3_PWMOut(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO , ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );


    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

#elif (PWM_MODE == PWM_MODE2)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // phase reverse

#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    //PC3 T1CH3
    TIM_OC3Init( TIM1, &TIM_OCInitStructure );
    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_OC3PreloadConfig( TIM1, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM1, ENABLE );
    TIM_Cmd( TIM1, ENABLE );

}

void PD2_T1CH1_PWMOut(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO , ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM1, ENABLE);

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

#elif (PWM_MODE == PWM_MODE2)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // phase reverse

#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init( TIM1, &TIM_OCInitStructure );
    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM1, ENABLE );
    TIM_Cmd( TIM1, ENABLE );

}

void PC6_T1CH1_RedOut(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO , ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM1, ENABLE);

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

#elif (PWM_MODE == PWM_MODE2)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // phase reverse

#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init( TIM1, &TIM_OCInitStructure );
    TIM_CtrlPWMOutputs(TIM1, ENABLE );
    TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM1, ENABLE );
    TIM_Cmd( TIM1, ENABLE );

}

void PD3_T2CH2_PWMOut(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO , ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );


    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

#elif (PWM_MODE == PWM_MODE2)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // phase reverse

#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init( TIM2, &TIM_OCInitStructure );
    TIM_CtrlPWMOutputs(TIM2, ENABLE );
    TIM_OC2PreloadConfig( TIM2, TIM_OCPreload_Disable );
    TIM_ARRPreloadConfig( TIM2, ENABLE );
    TIM_Cmd( TIM2, ENABLE );

}

/*********************************************************************
 * @fn      ADC
 *
 * @brief   ADC Config for Temperature and Pressure Sensors
 *
 * @return  none
 */

uint16_t temp_sensor_value = 0;
uint16_t pressure_sensor_value = 0;

uint8_t adcFlag = 1;

u16 TxBuf[10];

void ADCConfig()
//void ADCConfig(u8 channel)
{
	 ADC_InitTypeDef  ADC_InitStructure = {0};
	 GPIO_InitTypeDef GPIO_InitStructure = {0};

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);

//	  switch (channel) {
//	    case 0:  // PA2
//	    case 1:  // PA1
//	      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	      break;
//	    case 2:  // PC4
//	      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
//	      break;
//	    case 3:  // PD2
//	    case 4:  // PD3
//	    case 5:  // PD5
//	    case 6:  // PD6
//	    case 7:  // PD4
//	      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
//	      break;
//	    default:
//	      break;
//	  }

	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	  RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

//	  switch (channel) {
//	    case 0:  // PA2
//	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	      break;
//	    case 1:  // PA1
//	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	      break;
//	    case 2:  // PC4
//	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//	      break;
//	    case 3:  // PD2
//	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	      break;
//	    case 4:  // PD3
//	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	      break;
//	    case 5:  // PD5
//	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//	      break;
//	    case 6:  // PD6
//	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//	      break;
//	    case 7:  // PD4
//	      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//	      break;
//	    default:
//	      break;
//	  }

//	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

//	  switch (channel) {
//	    case 0:  // PA2
//	    case 1:  // PA1
//	      GPIO_Init(GPIOA, &GPIO_InitStructure);
//	      break;
//	    case 2:  // PC4
//	      GPIO_Init(GPIOC, &GPIO_InitStructure);
//	      break;
//	    case 3:  // PD2
//	    case 4:  // PD3
//	    case 5:  // PD5
//	    case 6:  // PD6
//	    case 7:  // PD4
//	      GPIO_Init(GPIOD, &GPIO_InitStructure);
//	      break;
//	    default:
//	      break;
//	  }

	   ADC_DeInit(ADC1);
	   ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	   ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	   ADC_InitStructure.ADC_NbrOfChannel = 1;
	   ADC_Init(ADC1, &ADC_InitStructure);

	   ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
	   ADC_DMACmd(ADC1, ENABLE);
	   ADC_Cmd(ADC1, ENABLE);

	   ADC_ResetCalibration(ADC1);
	   while(ADC_GetResetCalibrationStatus(ADC1));
	   ADC_StartCalibration(ADC1);
	   while(ADC_GetCalibrationStatus(ADC1));

	   //s16 Calibrattion_Val = Get_CalibrationValue(ADC1); ????
}

u16 Get_ADC_Val(u8 channel)
{
  u16 val;
  u8 rank;

  if(channel == 2)
	  rank = 1;
  else
	  rank = 1;


  ADC_RegularChannelConfig(ADC1, channel, rank, ADC_SampleTime_241Cycles);
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
  val = ADC_GetConversionValue(ADC1);

  //ADC_SoftwareStartConvCmd(ADC1, DISABLE);

  return val;
}

u16 Get_ADC_Average(u8 ch, u8 times)
{
  u32 temp_val = 0;
  u8 t;
  u16 val;

  for (t = 0; t < times; t++)
  {
    temp_val += Get_ADC_Val(ch);
    Delay_Ms(5);
  }

  val = temp_val / times;

  return val;
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

int main(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	SystemCoreClockUpdate();
	Delay_Init();

	GPIO_Config();

	//segment led
    I2C_PORTInit();
	OLED_Init();

	SSD1306_CLearDisplay();
	//SSD1306_Clear();
	//OLED_SetCursor(0,0);
	//OLED_Print("HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO");
	//OLED_SetCursor(10,20);
	//OLED_Print("0123456789");
	//SSD1306_DrawChar("HELLO", 5, 2);
	//SSD1306_DrawLine(0, 0, 127, 63, 1);
	//SSD1306_DrawLine(127, 0, 0, 63, 1);
	SSD1306_DrawString(0, 0, "Calico PG1", 2, 1); // �⺻ ũ��
//	SSD1306_DrawString(0, 16, "Lv 1 ", 2, 1); // 2�� ũ��
//	SSD1306_DrawString(0, 33, "T 30deg", 2, 1); // 2�� ũ��
//	SSD1306_DrawString(0, 50, "T 30deg", 2, 1); // 2�� ũ��
	SSD1306_Display();


	//adc
	ADCConfig();

//	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_241Cycles);
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//	Delay_Ms(50);
//	ADC_SoftwareStartConvCmd(ADC1, DISABLE);

	uint8_t ccp_lv;
	char buffer_level[16];
	char buffer_pressure[16];
	char buffer_temperature[16];


	while (1) {

		Delay_Ms(10);

		pressure_sensor_value = Get_ADC_Val(ADC_Channel_7);
		pressure_sensor_value = 0.995*(pressure_sensor_value * 325)/100;

		// snprintf(buffer_pressure, sizeof(buffer_pressure), "P: %u", pressure_sensor_value);
		// SSD1306_DrawString(0, 33, buffer_pressure, 2, 1); // 2�� ũ��
		// SSD1306_Display();

		temp_sensor_value = Get_ADC_Val(ADC_Channel_2);
		temp_sensor_value = 0.995*(temp_sensor_value * 325)/100; // Converting to real voltage w.r.t. VCC 3.3V, 0.985 multiplier factor is used with 3.3V for calibration. 330 * 0.985 = 325. This was as per my board, you might need to do it with different multiplier value.

		// snprintf(buffer_temperature, sizeof(buffer_temperature), "T: %u", temp_sensor_value);
		// SSD1306_DrawString(0, 50, buffer_temperature, 2, 1); // 2�� ũ��
		// SSD1306_Display();

//		PD5_T2CH4_GreenOut(100, 480-1, 100);
//		PC6_T1CH1_RedOut(100, 480-1, 100);
//		PD6_T2CH3_BlueOut(100, 480-1, 100);

		//PC6_T1CH1_RedOut(100, 480-1, 0);

		if(interruptFlag == 1) {
			PD5_T2CH4_GreenOut(100, 480-1, 0);
			PC6_T1CH1_RedOut(100, 480-1, 0);
			PD6_T2CH3_BlueOut(100, 480-1, 0);

			E_dir = 0;
			ccp_lv = 0;

			interruptFlag = 0;

			PC0_T2CH3_PWMOut(50-1, 24-1, 0);
			PD3_T2CH2_PWMOut(50-1, 24-1, 0); //motor 2
			PD2_T1CH1_PWMOut(50-1, 24-1, 0); //motor 3
			PC3_T1CH3_PWMOut(50-1, 24-1, 0); //motor 4

		}
		else if (interruptFlag == 2) { //encoder interrupt A

			if(E_dir == 1) // CW
			{
				ccp_lv = ccp_lv + 1;

				if(ccp_lv > 100)
					ccp_lv = 100;
				else if(ccp_lv < 0)
					ccp_lv = 0;

				//PD5_T2CH4_GreenOut(100, 480-1, 50);
				//PC6_T1CH1_RedOut(100-1, 480-1, ccp_lv);
				//PD6_T2CH3_BlueOut(100, 480-1, 0);

				snprintf(buffer_level, sizeof(buffer_level), "Lv: %4d", ccp_lv);
				SSD1306_DrawString(0, 16, buffer_level, 2, 1);
				SSD1306_Display();

				PC0_T2CH3_PWMOut(50-1, 24-1, ccp_lv/2);
				PD3_T2CH2_PWMOut(50-1, 24-1, ccp_lv/2); //motor 2
				PD2_T1CH1_PWMOut(50-1, 24-1, ccp_lv/2); //motor 3
				PC3_T1CH3_PWMOut(50-1, 24-1, ccp_lv/2); //motor 4
			}
			else
			{
				//ccp_lv = 0;
				E_dir = 0;
			}

			interruptFlag = 0;
		}
		else if (interruptFlag == 3) { //encoder interrupt B

			if(E_dir == 2) //CCW
			{
				ccp_lv = ccp_lv - 1;

				if(ccp_lv > 100)
					ccp_lv = 100;
				else if(ccp_lv < 0)
					ccp_lv = 0;

				snprintf(buffer_level, sizeof(buffer_level), "Lv: %4d", ccp_lv);
				SSD1306_DrawString(0, 16, buffer_level, 2, 1);
				SSD1306_Display();

				//PD5_T2CH4_GreenOut(100, 480-1, 00);
				//PC6_T1CH1_RedOut(100-1, 480-1, ccp_lv);
				//PD6_T2CH3_BlueOut(100, 480-1, 50);

				PC0_T2CH3_PWMOut(50-1, 24-1, ccp_lv/2);
				PD3_T2CH2_PWMOut(50-1, 24-1, ccp_lv/2); //motor 2
				PD2_T1CH1_PWMOut(50-1, 24-1, ccp_lv/2); //motor 3
				PC3_T1CH3_PWMOut(50-1, 24-1, ccp_lv/2); //motor 4
			}
			else
			{
				//ccp_lv = 0;
				E_dir = 0;
			}

			interruptFlag = 0;
		}
	}
}


void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      EXTI0_IRQHandler
 *
 * @brief   This function handles EXTI0 Handler.
 *
 * @return  none
 */
void EXTI7_0_IRQHandler(void) {

	//uint32_t current_time = SysTick->CNT;

	//if ((current_time - last_interrupt_time) > DEBOUNCE_TIME)
	{
		//last_interrupt_time = current_time;

		if (EXTI_GetITStatus(EXTI_Line0) != RESET) { //SW
			interruptFlag = 1;
			//EXTI_ClearITPendingBit(EXTI_Line0); /* Clear Flag */
		}
		if (EXTI_GetITStatus(EXTI_Line5) != RESET) { //Encoder A
			if(E_dir == 0)
				E_dir = 1;

			interruptFlag = 2;
			//EXTI_ClearITPendingBit(EXTI_Line5); /* Clear Flag */
		}
		if (EXTI_GetITStatus(EXTI_Line7) != RESET) { //Encoder B
			if(E_dir == 0)
				E_dir = 2;

			interruptFlag = 3;
			//EXTI_ClearITPendingBit(EXTI_Line5); /* Clear Flag */
		}
	}

	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
		EXTI_ClearITPendingBit(EXTI_Line0);

	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
		EXTI_ClearITPendingBit(EXTI_Line5);

	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
			EXTI_ClearITPendingBit(EXTI_Line7);
}
