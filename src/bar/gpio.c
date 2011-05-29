#include "gpio.h"
#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/event.h"
#include "kernel/systick.h"
#include "kernel/can/can_us.h" // TODO : revoir ce fichier (pas que pour le can)

#define GPIO_US1     0x01  // PC1
#define GPIO_US2     0x02  // PC3
#define GPIO_US3     0x04  // PC5
#define GPIO_US4     0x08  // PA5
#define GPIO_US5     0x10  // PB1

static uint8_t gpio_us;
static portTickType gpio_us_start_time[US_MAX];
static portTickType gpio_us_stop_time[US_MAX];

static int gpio_module_init(void)
{
	gpio_us = 0;

	// activation GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	GPIOA->CRL = ( GPIOA->CRL & ~(
	                GPIO_CRL_MODE4 | GPIO_CRL_CNF4 |
	                GPIO_CRL_MODE5 | GPIO_CRL_CNF5
		         ) ) |
					GPIO_CRL_CNF4_0 | GPIO_CRL_MODE4_1 |     // PA4 sortie collecteur ouvert (US)
					GPIO_CRL_CNF5_0 | GPIO_CRL_MODE5_1 ;     // PA5 entrée input flotante (sur IT) (US)

	// LED warning
	// activation GPIOB
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOB->CRL = ( GPIOB->CRL & ~(
	                GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
	                GPIO_CRL_MODE1 | GPIO_CRL_CNF1
		         ) ) |
					GPIO_CRL_CNF0_0 | GPIO_CRL_MODE0_1 |     // PB0 sortie collecteur ouvert (US)
					GPIO_CRL_CNF1_0 | GPIO_CRL_MODE1_1 ;     // PB1 entrée input flotante (sur IT) (US)

	// PB9 sortie push-pull, 2MHz
	GPIOB->CRH = (GPIOB->CRH & ~GPIO_CRH_MODE9 & ~GPIO_CRH_CNF9) | GPIO_CRH_MODE9_1;

	// activation GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	
	GPIOC->CRL = ( GPIOC->CRL & ~(
	                GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
	                GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |
	                GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
	                GPIO_CRL_MODE3 | GPIO_CRL_CNF3 |
	                GPIO_CRL_MODE4 | GPIO_CRL_CNF4 |
	                GPIO_CRL_MODE5 | GPIO_CRL_CNF5
		         ) ) |
		            GPIO_CRL_CNF0_0 | GPIO_CRL_MODE0_1 |     // PC0 sortie collecteur ouvert (US)
		            GPIO_CRL_CNF1_0 | GPIO_CRL_MODE1_1 |     // PC1 entrée input flotante (sur IT) (US)
		            GPIO_CRL_CNF2_0 | GPIO_CRL_MODE2_1 |     // PC2 sortie collecteur ouvert (US)
					GPIO_CRL_CNF3_0 | GPIO_CRL_MODE3_1 |     // PC3 entrée input flotante (sur IT) (US)
					GPIO_CRL_CNF4_0 | GPIO_CRL_MODE4_1 |     // PC4 sortie collecteur ouvert (US)
					GPIO_CRL_CNF5_0 | GPIO_CRL_MODE5_1 ;     // PC5 entrée input flotante (sur IT) (US)

	// Boutons 1, 2 et 3
	// activation GPIOD
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	// PD8 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE8 & ~GPIO_CRH_CNF8) | GPIO_CRH_CNF8_0;
	// PD9 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE9 & ~GPIO_CRH_CNF9) | GPIO_CRH_CNF9_0;
	// PD10 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10) | GPIO_CRH_CNF10_0;

	// LED sur PE0, PE1, PE2, PE3, PE4, PE5
	// activation GPIOE
	RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
	// PE0 sortie push-pull, 2MHz
	GPIOE->CRL = (GPIOE->CRL & ~GPIO_CRL_MODE0 & ~GPIO_CRL_CNF0) | GPIO_CRL_MODE0_1;
	// PE1 sortie push-pull, 2MHz
	GPIOE->CRL = (GPIOE->CRL & ~GPIO_CRL_MODE1 & ~GPIO_CRL_CNF1) | GPIO_CRL_MODE1_1;
	// PE2 sortie push-pull, 2MHz
	GPIOE->CRL = (GPIOE->CRL & ~GPIO_CRL_MODE2 & ~GPIO_CRL_CNF2) | GPIO_CRL_MODE2_1;
	// PE3 sortie push-pull, 2MHz
	GPIOE->CRL = (GPIOE->CRL & ~GPIO_CRL_MODE3 & ~GPIO_CRL_CNF3) | GPIO_CRL_MODE3_1;
	// PE4 sortie push-pull, 2MHz
	GPIOE->CRL = (GPIOE->CRL & ~GPIO_CRL_MODE4 & ~GPIO_CRL_CNF4) | GPIO_CRL_MODE4_1;
	// PE5 sortie push-pull, 2MHz
	GPIOE->CRL = (GPIOE->CRL & ~GPIO_CRL_MODE5 & ~GPIO_CRL_CNF5) | GPIO_CRL_MODE5_1;

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	// US :
	// PA5 => it sur front montant et front descendant
	// PB1 => it sur front montant et front descendant
	// PC1 => it sur front montant et front descendant
	// PC3 => it sur front montant et front descendant
	// PC5 => it sur front montant et front descendant
	// boutons :
	// PD8 => it sur front montant
	// PD9 => it sur front montant
	// PD10 => it sur front montant
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB | AFIO_EXTICR1_EXTI1_PC | AFIO_EXTICR1_EXTI3_PC;
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PA | AFIO_EXTICR2_EXTI5_PC;
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PD | AFIO_EXTICR3_EXTI9_PD | AFIO_EXTICR3_EXTI10_PD;
	EXTI->IMR  |= EXTI_IMR_MR1  | EXTI_IMR_MR3  | EXTI_IMR_MR5  | EXTI_IMR_MR8 | EXTI_IMR_MR9 | EXTI_IMR_MR10;
	EXTI->RTSR |= EXTI_IMR_MR1  | EXTI_IMR_MR3  | EXTI_IMR_MR5  | EXTI_RTSR_TR8 | EXTI_RTSR_TR9 | EXTI_RTSR_TR10;
	EXTI->FTSR |= EXTI_FTSR_TR1 | EXTI_FTSR_TR3 | EXTI_FTSR_TR5;
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	error_raise(LED_0 | LED_1 | LED_2 | LED_3 | LED_4 | LED_5 | LED_WARNING);

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

void isr_exti1(void)
{
	if( EXTI->PR & EXTI_PR_PR1)
	{
		EXTI->PR |= EXTI_PR_PR1;
		if( gpio_us & GPIO_US1 )
		{
			if( GPIOC->IDR & GPIO_IDR_IDR1 == 0 )
			{
				// front descendant sur PC1
				gpio_us &= ~GPIO_US1;
				gpio_us_stop_time[US_1] = systick_get_time_from_isr();
			}
		}
		else
		{
			if( GPIOC->IDR & GPIO_IDR_IDR1 )
			{
				// front montant sur PC1
				gpio_us |= GPIO_US1;
				gpio_us_start_time[US_1] = systick_get_time_from_isr();
			}
		}
		if( gpio_us & GPIO_US5 )
		{
			if( GPIOB->IDR & GPIO_IDR_IDR1 == 0 )
			{
				// front descendant sur PB1
				gpio_us &= ~GPIO_US5;
				gpio_us_stop_time[US_5] = systick_get_time_from_isr();
			}
		}
		else
		{
			if( GPIOB->IDR & GPIO_IDR_IDR1 )
			{
				// front montant sur PB1
				gpio_us |= GPIO_US5;
				gpio_us_start_time[US_5] = systick_get_time_from_isr();
			}
		}
	}
}

void isr_exti3(void)
{
	if( EXTI->PR & EXTI_PR_PR3)
	{
		EXTI->PR |= EXTI_PR_PR3;
		if( gpio_us & GPIO_US2 )
		{
			if( GPIOC->IDR & GPIO_IDR_IDR3 == 0 )
			{
				// front descendant sur PC3
				gpio_us &= ~GPIO_US2;
				gpio_us_stop_time[US_2] = systick_get_time_from_isr();
			}
		}
		else
		{
			if( GPIOC->IDR & GPIO_IDR_IDR3 )
			{
				// front montant sur PC3
				gpio_us |= GPIO_US2;
				gpio_us_start_time[US_2] = systick_get_time_from_isr();
			}
		}
	}
}

void isr_exit9_5(void)
{
	if( EXTI->PR & EXTI_PR_PR5)
	{
		EXTI->PR |= EXTI_PR_PR5;
		if( gpio_us & GPIO_US3 )
		{
			if( GPIOC->IDR & GPIO_IDR_IDR5 == 0 )
			{
				// front descendant sur PC5
				gpio_us &= ~GPIO_US3;
				gpio_us_stop_time[US_3] = systick_get_time_from_isr();
			}
		}
		else
		{
			if( GPIOC->IDR & GPIO_IDR_IDR5 )
			{
				// front montant sur PC5
				gpio_us |= GPIO_US3;
				gpio_us_start_time[US_3] = systick_get_time_from_isr();
			}
		}
		if( gpio_us & GPIO_US4 )
		{
			if( GPIOA->IDR & GPIO_IDR_IDR5 == 0 )
			{
				// front descendant sur PA5
				gpio_us &= ~GPIO_US4;
				gpio_us_stop_time[US_4] = systick_get_time_from_isr();
			}
		}
		else
		{
			if( GPIOC->IDR & GPIO_IDR_IDR5 )
			{
				// front montant sur PA5
				gpio_us |= GPIO_US4;
				gpio_us_start_time[US_4] = systick_get_time_from_isr();
			}
		}
	}
	if( EXTI->PR & EXTI_PR_PR8)
	{
		EXTI->PR |= EXTI_PR_PR8;
		// TODO action btn1
	}
	if( EXTI->PR & EXTI_PR_PR9)
	{
		EXTI->PR |= EXTI_PR_PR9;
		// TODO action btn2
	}
}

void isr_exit15_10(void)
{
	if( EXTI->PR & EXTI_PR_PR10)
	{
		EXTI->PR |= EXTI_PR_PR10;
		// TODO action btn3
	}
}

void setLed(uint32_t mask)
{
	GPIOE->ODR = (GPIOE->ODR & ~((uint32_t)0x3F)) | (mask & 0x3F);
	GPIOB->ODR = (GPIOB->ODR & ~((uint32_t)LED_WARNING)) | (mask & LED_WARNING);
}
