/**
  ******************************************************************************
  * @file    main.c
  * @author  Parker Lloyd
  * @version V1.0
  * @brief   Default main function.
  ******************************************************************************
*/

#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define TRUE 1
#define FALSE 0
#define NOT_PRESSED FALSE
#define PRESSED TRUE
#define BUFFERSIZE 2000

#ifdef USE_SEMIHOSTING
extern void initialise_monitor_handles();
#endif

//Function prototypes
static void prvSetupHardware(void);
static void UART_Configuration(void);
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void EXTI_Configuration(void);
static void NVIC_Configuration(void);
static void TIM2_Configuration(void);
static void DMA_Configuration(void);
static void ADC_Configuration(void);
static void Enable_Peripherals(void);
void buttonHandler(void);
void printdata(volatile uint16_t *data);

__IO uint16_t ADCConvertedValues[BUFFERSIZE];

char usr_msg[250]={0};
uint8_t button_status_flag = NOT_PRESSED;

int main(void)
{

#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("Semi-hosting is enabled\n");
#endif

	DWT->CTRL |= (1 << 0);

	//Reset the RCC clock configuration to the default reset state.
	//HSI ON, PLL OFF, HSE OFF, system clock = 16MHz, cpu_clock = 16MHz
	RCC_DeInit();

	SystemCoreClockUpdate();

	prvSetupHardware();

	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	vTaskStartScheduler();

	for(;;);
}

void buttonHandler(void)
{
	if(!button_status_flag)
	{
		button_status_flag = PRESSED;
		Enable_Peripherals();
	    	ADC_SoftwareStartConv(ADC1);
	}
}

static void RCC_Configuration(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

static void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//ADC Channel 11 -> PC1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitTypeDef led_init, button_init;
	led_init.GPIO_Mode = GPIO_Mode_OUT;
	led_init.GPIO_OType = GPIO_OType_PP;
	led_init.GPIO_Pin = GPIO_Pin_5;
	led_init.GPIO_Speed = GPIO_Low_Speed;
	led_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &led_init);

	button_init.GPIO_Mode = GPIO_Mode_IN;
	button_init.GPIO_OType = GPIO_OType_PP;
	button_init.GPIO_Pin = GPIO_Pin_13;
	button_init.GPIO_Speed = GPIO_Low_Speed;
	button_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &button_init);

	GPIO_InitTypeDef GPIOA_RXTX_InitStructure;

	//Enable the GPIOA, UART2, and ADC1 Peripheral clock
	//PA2 is UART2_TX, PA3 is UART2_RX

	//Alternate function configuration of MCU pins to behave as UART2 TX and RX
	memset(&GPIOA_RXTX_InitStructure,0,sizeof(GPIOA_RXTX_InitStructure));

	GPIOA_RXTX_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIOA_RXTX_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIOA_RXTX_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIOA_RXTX_InitStructure.GPIO_OType= GPIO_OType_PP;
	GPIOA_RXTX_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA, &GPIOA_RXTX_InitStructure);

	//AF mode settings for the pins
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //PA2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //PA3
}

static void EXTI_Configuration(void)
{
	//Interrupt configuration for the button (PC13)
	//System configuration for EXTI line
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

	//EXTI line configuration 13, falling edge, interrupt mode
	EXTI_InitTypeDef exti_init;
	exti_init.EXTI_Line = EXTI_Line13;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti_init);
}

static void NVIC_Configuration(void)
{
	NVIC_SetPriority(EXTI15_10_IRQn, 5);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void UART_Configuration(void)
{
	USART_InitTypeDef UART2_InitStructure;

	memset(&UART2_InitStructure,0,sizeof(UART2_InitStructure));

	UART2_InitStructure.USART_BaudRate = 115200;
	UART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART2_InitStructure.USART_Mode =  USART_Mode_Tx | USART_Mode_Rx;
	UART2_InitStructure.USART_Parity = USART_Parity_No;
	UART2_InitStructure.USART_StopBits = USART_StopBits_1;
	UART2_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&UART2_InitStructure);
}

static void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (16000000/1000) - 1; //HCLK: 16MHz, TIM2_IT: 1kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	// TIM2 TRGO selection
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO
}

static void ADC_Configuration(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	// ADC1 regular channel 11 configuration
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_15Cycles); // PC1
}

static void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA2_STR0_CH0_InitStructure;	//ADC1
	DMA2_STR0_CH0_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA2_STR0_CH0_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValues;
	DMA2_STR0_CH0_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA2_STR0_CH0_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA2_STR0_CH0_InitStructure.DMA_BufferSize = BUFFERSIZE;
	DMA2_STR0_CH0_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA2_STR0_CH0_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA2_STR0_CH0_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA2_STR0_CH0_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA2_STR0_CH0_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA2_STR0_CH0_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA2_STR0_CH0_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA2_STR0_CH0_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA2_STR0_CH0_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA2_STR0_CH0_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA2_STR0_CH0_InitStructure);
	// Enable DMA Stream Half / Transfer Complete interrupt
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);

}

static void Enable_Peripherals(void)
{
    	TIM_Cmd(TIM2, ENABLE);
    	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
    	USART_Cmd(USART2,ENABLE);
    	TIM_Cmd(TIM2, ENABLE);
    	DMA_Cmd(DMA2_Stream0, ENABLE);
    	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    	ADC_DMACmd(ADC1, ENABLE);
    	ADC_Cmd(ADC1, ENABLE);
}

static void prvSetupHardware(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	UART_Configuration();
	EXTI_Configuration();
	NVIC_Configuration();
	TIM2_Configuration();
	DMA_Configuration();
	ADC_Configuration();
}

void printdata(volatile uint16_t *data)
{
	for(uint16_t i=0; i < BUFFERSIZE/2; i++)
	{
		while ( USART_GetFlagStatus(USART2,USART_FLAG_TXE) != SET);
		USART_SendData(USART2, data[i] >> 8);		//high byte
		while ( USART_GetFlagStatus(USART2,USART_FLAG_TXE) != SET);
		USART_SendData(USART2, data[i] & 0xFF);		//low byte
	}

	while ( USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);

}

void DMA2_Stream0_IRQHandler(void) // Called at 1 KHz for 200 KHz sample rate, LED Toggles at 500 Hz
{
	traceISR_ENTER(); // for SEGGER

	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
		printdata(&ADCConvertedValues[0]);
	}
	
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
	{
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
		printdata(&ADCConvertedValues[BUFFERSIZE/2]);
	}

	traceISR_EXIT();
}

void EXTI15_10_IRQHandler(void)
{
	traceISR_ENTER(); // for SEGGER
	// Clear the interrupt pending bit of the EXTI line (13)
	EXTI_ClearITPendingBit(EXTI_Line13);
	buttonHandler();
	traceISR_EXIT();
}
