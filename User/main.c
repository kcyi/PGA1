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

/* Global define */
	/* PWM Output Mode Definition */
	#define PWM_MODE1   0
	#define PWM_MODE2   1
	/* PWM Output Mode Selection */
	//#define PWM_MODE PWM_MODE1
	#define PWM_MODE PWM_MODE2

/* Global Variable */
vu8 val;
uint8_t interruptFlag = 0;

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


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);

	// LED Output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_5;// | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//Switch Input - interrupt
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; // Defines which Pin to configure
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Defines Output Type
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* GPIOC ----> EXTI_Line5,6,7 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,
			GPIO_PinSource5);
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,
			GPIO_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,
			GPIO_PinSource7);
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
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
void TIM1_PWMOut_Init(u16 arr, u16 psc, u16 ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    TIM_OCInitTypeDef TIM_OCInitStructure={0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOD, &GPIO_InitStructure );

    //RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM1, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

#elif (PWM_MODE == PWM_MODE2)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;

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
	GPIO_ResetBits(GPIOD, GPIO_Pin_0 | GPIO_Pin_5);// | GPIO_Pin_6);

	u8 i = 0, j = 0, k = 0;

	Delay_Ms(500);
	TIM1_PWMOut_Init(100, 480-1, 50);

	while (1) {

//      while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
//      {
//          /* waiting for receiving finish */
//      }
//      val = (USART_ReceiveData(USART1));
//      USART_SendData(USART1, ~val);
//      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
//      {
//          /* waiting for sending finish */
//      }

		Delay_Ms(100);

		if (interruptFlag == 1) {
			GPIO_WriteBit(GPIOD, GPIO_Pin_0, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
			interruptFlag = 0;
		}
		else if (interruptFlag == 2) {
			GPIO_WriteBit(GPIOD, GPIO_Pin_5, (j == 0) ? (j = Bit_SET) : (j = Bit_RESET));
			interruptFlag = 0;
		}
		else if (interruptFlag == 3) {
			//GPIO_WriteBit(GPIOD, GPIO_Pin_6, (k == 0) ? (k = Bit_SET) : (k = Bit_RESET));
			interruptFlag = 0;
		}

		//TIM1_PWMOut_Init(100, 24-1, 80);


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
	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		interruptFlag = 1;
		EXTI_ClearITPendingBit(EXTI_Line5); /* Clear Flag */
	}
	else if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		interruptFlag = 2;
		EXTI_ClearITPendingBit(EXTI_Line6); /* Clear Flag */
	}
	else if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		interruptFlag = 3;
		EXTI_ClearITPendingBit(EXTI_Line7); /* Clear Flag */
	}
}
