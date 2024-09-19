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
//#include "SSD1306.h"

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

u32 timer_rotary = 0;
int E_dir = 0;

/*********************************************************************
 * @fn      USARTx_CFG
 *
 * @brief   Initializes the USART2 & USART3 peripheral.
 *
 * @return  none
 */
//void USARTx_CFG(void)
//{
//    GPIO_InitTypeDef  GPIO_InitStructure = {0};
//    USART_InitTypeDef USART_InitStructure = {0};
//
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);
//
//    /* USART1 TX-->D.5   RX-->D.6 */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_Init(GPIOD, &GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_Init(GPIOD, &GPIO_InitStructure);
//
//    USART_InitStructure.USART_BaudRate = 115200;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//
//    USART_Init(USART1, &USART_InitStructure);
//    USART_Cmd(USART1, ENABLE);
//}

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

//#if (SDI_PRINT == SDI_PR_OPEN)
//    SDI_Printf_Enable();
//#else
//    USART_Printf_Init(115200);
//#endif
//    printf("SystemClk:%d\r\n",SystemCoreClock);
//    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

//    USARTx_CFG();

	GPIO_Config();

	u8 i = 0, j = 0, k = 0;
	u8 red = 0, blue = 30, green = 70;
	u8 ccp_lv;
	//u8 re_B = 0, re_Btemp = 0;



	re_Atemp = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);
	re_Btemp = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);

	//Delay_Ms(25);
	//TIM1_PWMOut_Init(100, 4800-1, 0);
//	PC0_T2CH3_PWMOut(100, 4800-1, 30); //motor 1
//	PD6_T2CH3_LEDOut(100, 4800-1, 30); // blue
//	PC0_T2CH3_PWMOut(100, 4800-1, 30); //motor 1
//	PD3_T2CH2_PWMOut(100, 4800-1, 85); //motor 2
//	PD2_T1CH1_PWMOut(100, 4800-1, 50); //motor 3
//	PC6_T1CH1_LEDOut(100, 4800-1, 50); //red
//	PD2_T1CH1_PWMOut(100, 4800-1, 50); //motor 3
//	PC3_T1CH3_PWMOut(100, 4800-1, 75); //motor 4
//	PD5_T2CH4_LEDOut(100, 4800-1, 50); //green

//    I2C_PORTInit();
//    OLED_Init();
//
//    SSD1306_CLearDisplay();
//    //SSD1306_Clear();
//    //OLED_SetCursor(0,0);
//    //OLED_Print("HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO   HELLO");
//    //OLED_SetCursor(10,20);
//    //OLED_Print("0123456789");
//    //SSD1306_DrawChar("HELLO", 5, 2);
//    //SSD1306_DrawLine(0, 0, 127, 63, 1);
//    //SSD1306_DrawLine(127, 0, 0, 63, 1);
//    SSD1306_DrawString(0, 0, "Hello", 1, 1); // 기본 크기
//    SSD1306_DrawString(0, 30, "Calico", 3, 1); // 2배 크기
//    //SSD1306_DrawString(0, 30, "0123456", 3, 1); // 2배 크기
//    SSD1306_Display();
//	PC6_T1CH1_RedOut(100, 4800-1, 20);




	while (1) {
//
////      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
////      {
////          /* waiting for receiving finish */
////      }
////      val = (USART_ReceiveData(USART1));
////      USART_SendData(USART1, ~val);
////      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
////      {
////          /* waiting for sending finish */
////      }
//
//		//GPIO_ResetBits(GPIOD, GPIO_Pin_0);
//
		Delay_Ms(10);

		if(timer_rotary > 0)
			timer_rotary--;

		if (interruptFlag == 2) { //encoder interrupt A

			//if(re_B == re_Btemp)
			{
				if(E_dir == 1)
				{
					ccp_lv++;
					PD5_T2CH4_GreenOut(100, 480-1, 50);
					PC6_T1CH1_RedOut(100, 480-1, 0);
					PD6_T2CH3_BlueOut(100, 480-1, 0);
				}
			}

			re_Atemp = re_A;
			re_Btemp = re_B;

			if(timer_rotary == 0)
				timer_rotary = TIMER_INIT;

			interruptFlag = 0;

		}
		if (interruptFlag == 3) { //encoder interrupt B

			//if(re_A == re_Atemp)
			{
				if(E_dir == 2)
				{
					ccp_lv++;
					PD5_T2CH4_GreenOut(100, 480-1, 00);
					PC6_T1CH1_RedOut(100, 480-1, 0);
					PD6_T2CH3_BlueOut(100, 480-1, 50);
				}
			}

			re_Atemp = re_A;
			re_Btemp = re_B;

			if(timer_rotary == 0)
				timer_rotary = TIMER_INIT;

			interruptFlag = 0;
		}

		interruptFlag = 0;


//
////			GPIO_WriteBit(GPIOD, GPIO_Pin_0, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
////			ccp_lv = ccp_lv + 10;
////			if(ccp_lv > 100)
////				ccp_lv = 100;
////
////			TIM1_PWMOut_Init(100, 4800-1, ccp_lv);
//
//			interruptFlag = 0;
//		}
//		else if (interruptFlag == 2) { //LIGHT
////			GPIO_WriteBit(GPIOD, GPIO_Pin_6, (j == 0) ? (j = Bit_SET) : (j = Bit_RESET));
//			GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_5);
//			GPIO_SetBits(GPIOD, GPIO_Pin_6);
//
//			GPIO_ResetBits(GPIOC, GPIO_Pin_4);
//
//			switch (i) {
//			case 4:
//				GPIO_ResetBits(GPIOC, GPIO_Pin_0); //motor 1
//				i = 3;
//				break;
//			case 3:
//				GPIO_ResetBits(GPIOD, GPIO_Pin_3); //motor 2
//				i = 2;
//				break;
//			case 2:
//				GPIO_ResetBits(GPIOD, GPIO_Pin_2); //motor 3
//				i = 1;
//				break;
//			case 1:
//				GPIO_ResetBits(GPIOC, GPIO_Pin_3); //motor 4
//				i = 0;
//				break;
//			case 0:
//				break;
//			default :
//				i = 0;
//				break;
//			}
//
//			interruptFlag = 0;
//		}
//		else if (interruptFlag == 3) { //NB
//			GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_6);
//			GPIO_SetBits(GPIOD, GPIO_Pin_5);
//
//			GPIO_SetBits(GPIOC, GPIO_Pin_4);
//
////			GPIO_WriteBit(GPIOD, GPIO_Pin_6, (k == 0) ? (k = Bit_SET) : (k = Bit_RESET));
////			ccp_lv = ccp_lv - 10;
////			if(ccp_lv < 20)
////				ccp_lv = 20;
////
////			TIM1_PWMOut_Init(100, 4800-1, ccp_lv);
//
//			interruptFlag = 0;
//		}


		//TIM1_PWMOut_Init(100, 480-1, 50);


//      GPIO_ResetBits(GPIOD, GPIO_Pin_0);
//      GPIO_ResetBits(GPIOD, GPIO_Pin_5);
//    	GPIO_WriteBit(GPIOD, GPIO_Pin_5|GPIO_Pin_6, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));

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
			//if(timer_rotary == 0)
				E_dir = 1;

			interruptFlag = 2;
			re_A = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);
			re_B = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
			//EXTI_ClearITPendingBit(EXTI_Line5); /* Clear Flag */
		}
		if (EXTI_GetITStatus(EXTI_Line7) != RESET) { //Encoder B
			//if(timer_rotary == 0)
				E_dir = 2;

			interruptFlag = 3;
			re_A = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5);
			re_B = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
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
