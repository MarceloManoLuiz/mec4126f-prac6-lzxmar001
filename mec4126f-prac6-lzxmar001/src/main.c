// Description----------------------------------------------------------------|
/*
 * Initialises a struct with Name and Age data. Displays results on LEDs and
 * LCD.
 */
// DEFINES AND INCLUDES-------------------------------------------------------|

#define STM32F051
//>>> Uncomment line 10 if using System Workbench (SW4STM32) or STM32CubeIDE
//#define SW4STM32

#ifndef SW4STM32
	#define TRUESTUDIO
#endif

#include "stm32f0xx.h"
#include "lcd_stm32f0.h"

// GLOBAL VARIABLES ----------------------------------------------------------|


// FUNCTION DECLARATIONS -----------------------------------------------------|

void main(void);                                                   //COMPULSORY
void init_ADC(void);
void init_external_interrupts(void);
void init_LEDs(void);
void init_buttons(void);


#ifdef TRUESTUDIO												   //COMPULSORY
	void reset_clock_to_48Mhz(void);							   //COMPULSORY
#endif															   //COMPULSORY



// MAIN FUNCTION -------------------------------------------------------------|

void main(void)
{
#ifdef TRUESTUDIO  											 	   //COMPULSORY
	reset_clock_to_48Mhz();										   //COMPULSORY
#endif															   //COMPULSORY

	init_LCD();
	init_ADC();

	if (EXTI->PR &= EXTI_PR_PR3)				//if PA3 is pressed
	{
		ADC1->CR |= ADC_CR_ADSTART;				//START CONVERSION
	}

	init_buttons();
	init_LEDs();
	init_external_interrupts();

	while(1)
	{
	}
}

// OTHER FUNCTIONS -----------------------------------------------------------|

#ifdef TRUESTUDIO												   //COMPULSORY
/* Description:
 * This function resets the STM32 Clocks to 48 MHz
 */
void reset_clock_to_48Mhz(void)									   //COMPULSORY
{																   //COMPULSORY
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)			   //COMPULSORY
	{															   //COMPULSORY
		RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW);					   //COMPULSORY
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);	   //COMPULSORY
	}															   //COMPULSORY

	RCC->CR &= (uint32_t)(~RCC_CR_PLLON);						   //COMPULSORY
	while ((RCC->CR & RCC_CR_PLLRDY) != 0);						   //COMPULSORY
	RCC->CFGR = ((RCC->CFGR & (~0x003C0000)) | 0x00280000);		   //COMPULSORY
	RCC->CR |= RCC_CR_PLLON;									   //COMPULSORY
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);						   //COMPULSORY
	RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL);					   //COMPULSORY
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);		   //COMPULSORY
}																   //COMPULSORY
#endif															   //COMPULSORY

void init_ADC(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;				//PROVIDES CLOCK TO ADC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 				//ENABLE CLOCK FOR PORT A FOR PA5
	GPIOA->MODER |= GPIO_MODER_MODER5;				//CONFIGURES PIN PA5 AS ANALOGUE INPUT
	ADC1->CR |= ADC_CR_ADEN;						//ACTIVATES ADC
	ADC1->CHSELR |= ADC_CHSELR_CHSEL5;				//SETS CHANNEL TO 5
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;					//SETS RESOLUTION TO 8bit
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);

	ADC1->CFGR1 &= ~ADC_CFGR1_CONT;					//SETS ADC TO SINGLE CONVERSION MODE
	while ((ADC1->ISR & ADC_ISR_EOC)==0);
}

void display_on_LCD(void)
{
	uint8_t numbers = ADC1->DR;
	unsigned char value[1];
	sprintf(value, "%d", numbers);
	lcd_command(CLEAR);
	lcd_putstring(value);
}

void init_LEDs(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;				//enable clock for LEDs
	GPIOB->MODER |= GPIO_MODER_MODER0_0; 			//set PB0 to output
	GPIOB->MODER |= GPIO_MODER_MODER1_0; 			//set PB1 to output
	GPIOB->MODER |= GPIO_MODER_MODER2_0; 			//set PB2 to output
	GPIOB->MODER |= GPIO_MODER_MODER3_0; 			//set PB3 to output
	GPIOB->MODER |= GPIO_MODER_MODER4_0;		 	//set PB4 to output
	GPIOB->MODER |= GPIO_MODER_MODER5_0; 			//set PB5 to output
	GPIOB->MODER |= GPIO_MODER_MODER6_0; 			//set PB6 to output
	GPIOB->MODER |= GPIO_MODER_MODER7_0; 			//set PB7 to output
}

void display_on_LEDs(void)
{
	GPIOB->ODR = ADC1->DR;
}

void init_buttons(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;				//enable clock for BUTTONS
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_0;			//set PA3 to pull-up resistor input
}

void init_external_interrupts(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;		//ENABLE CLOCK FOR SYSCONFIG.
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA;	//ROUTE PORT A TO EXTI3
	EXTI->IMR |= EXTI_IMR_MR3;						//Unmask the interrupt on Port A pin 3.
	EXTI->FTSR |= EXTI_FTSR_TR3;					//@FALLING EDGE
	NVIC_EnableIRQ(EXTI2_3_IRQn);					//enable exti2_3 interrupt on nvic
}


// INTERRUPT HANDLERS --------------------------------------------------------|

void EXTI2_3_IRQHandler(void)
{
	if (EXTI->PR &= EXTI_PR_PR3)				//if PA3 is pressed
	{
		display_on_LEDs();						//show result on LEDs
		display_on_LCD();						//show result on LCD
	}
	else
	{
	}
	EXTI->PR |= EXTI_PR_PR3;
}

