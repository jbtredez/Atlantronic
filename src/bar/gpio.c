#include "gpio.h"
#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/event.h"
#include "kernel/systick.h"
#include "kernel/rcc.h"
#include <string.h>

#define HALF_SOUND_SPEED        165000LL

static uint8_t gpio_us;
static uint8_t gpio_us_triggered;
static portTickType gpio_us_start_time[US_MAX];
static portTickType gpio_us_stop_time[US_MAX];
static uint16_t gpio_us_distance[US_MAX];

static int gpio_module_init(void)
{
	gpio_us = 0;
	gpio_us_triggered = 0;
	gpio_us_distance[US_RIGHT] = 0;
	gpio_us_distance[US_FRONT] = 0;
	gpio_us_distance[US_BACK] = 0;
	gpio_us_distance[US_NA] = 0;
	gpio_us_distance[US_LEFT] = 0;

	// activation GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	GPIOA->CRL = ( GPIOA->CRL & ~(
	                GPIO_CRL_MODE4 | GPIO_CRL_CNF4 |
	                GPIO_CRL_MODE5 | GPIO_CRL_CNF5
		         ) ) |
					GPIO_CRL_CNF4_0 | GPIO_CRL_MODE4_1 |     // PA4 sortie collecteur ouvert (US)
					GPIO_CRL_CNF5_0                    ;     // PA5 entrée input flotante (sur IT) (US)

	// LED warning
	// activation GPIOB
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	GPIOB->CRL = ( GPIOB->CRL & ~(
	                GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
	                GPIO_CRL_MODE1 | GPIO_CRL_CNF1
		         ) ) |
					GPIO_CRL_CNF0_0 | GPIO_CRL_MODE0_1 |     // PB0 sortie collecteur ouvert (US)
					GPIO_CRL_CNF1_0                   ;     // PB1 entrée input flotante (sur IT) (US)

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
		            GPIO_CRL_CNF1_0                    |     // PC1 entrée input flotante (sur IT) (US)
		            GPIO_CRL_CNF2_0 | GPIO_CRL_MODE2_1 |     // PC2 sortie collecteur ouvert (US)
					GPIO_CRL_CNF3_0                    |     // PC3 entrée input flotante (sur IT) (US)
					GPIO_CRL_CNF4_0 | GPIO_CRL_MODE4_1 |     // PC4 sortie collecteur ouvert (US)
					GPIO_CRL_CNF5_0                    ;     // PC5 entrée input flotante (sur IT) (US)

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
	// TODO Attention, on ne peut pas activer toutes lesl ignes en même temps
	AFIO->EXTICR[0] |= /*AFIO_EXTICR1_EXTI1_PB |*/ AFIO_EXTICR1_EXTI1_PC | AFIO_EXTICR1_EXTI3_PC;
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PA /*| AFIO_EXTICR2_EXTI5_PC*/;
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PD | AFIO_EXTICR3_EXTI9_PD | AFIO_EXTICR3_EXTI10_PD;
	EXTI->IMR  |= EXTI_IMR_MR1  | EXTI_IMR_MR3  | EXTI_IMR_MR5  | EXTI_IMR_MR8 | EXTI_IMR_MR9 | EXTI_IMR_MR10;
	EXTI->RTSR |= EXTI_RTSR_TR1 | EXTI_RTSR_TR3 | EXTI_RTSR_TR5 | EXTI_RTSR_TR8 | EXTI_RTSR_TR9 | EXTI_RTSR_TR10;
	EXTI->FTSR |= EXTI_FTSR_TR1 | EXTI_FTSR_TR3 | EXTI_FTSR_TR5;

	NVIC_SetPriority(EXTI1_IRQn, PRIORITY_IRQ_EXTI1);
	NVIC_SetPriority(EXTI3_IRQn, PRIORITY_IRQ_EXTI3);
	NVIC_SetPriority(EXTI9_5_IRQn, PRIORITY_IRQ_EXTI9_5);
	NVIC_SetPriority(EXTI15_10_IRQn, PRIORITY_IRQ_EXTI15_10);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	setLed(LED_0 | LED_1 | LED_2 | LED_3 | LED_4 | LED_5 | LED_WARNING);

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

void isr_exti1(void)
{
	portSET_INTERRUPT_MASK();

	if( EXTI->PR & EXTI_PR_PR1)
	{
		EXTI->PR |= EXTI_PR_PR1;
		if( GPIOC->IDR & GPIO_IDR_IDR1 )
		{
			// PC1 niveau haut
			if( (gpio_us & US_RIGHT_MASK) == 0 && (gpio_us_triggered & US_RIGHT_MASK))
			{
				// PC1 etait au niveau bas (front montant)
				gpio_us_start_time[US_RIGHT] = systick_get_time_from_isr();
			}
			gpio_us |= US_RIGHT_MASK;
		}
		else
		{
			// PC1 niveau bas
			if( gpio_us & US_RIGHT_MASK && (gpio_us_triggered & US_RIGHT_MASK))
			{
				// front descendant sur PC1
				gpio_us_triggered &= ~US_RIGHT_MASK;
				gpio_us_stop_time[US_RIGHT] = systick_get_time_from_isr();
				gpio_us_distance[US_RIGHT] = (HALF_SOUND_SPEED*(gpio_us_stop_time[US_RIGHT] - gpio_us_start_time[US_RIGHT]))/72000000LL;			
			}
			gpio_us &= ~US_RIGHT_MASK;
		}

		if( GPIOB->IDR & GPIO_IDR_IDR1 )
		{
			if( (gpio_us & US_LEFT_MASK) == 0 && (gpio_us_triggered & US_LEFT_MASK))
			{
				// front montant sur PB1
				gpio_us_start_time[US_LEFT] = systick_get_time_from_isr();
			}
			gpio_us |= US_LEFT_MASK;
		}
		else
		{
			if( gpio_us & US_LEFT_MASK && (gpio_us_triggered & US_LEFT_MASK))
			{
				// front descendant sur PB1
				gpio_us_triggered &= ~US_LEFT_MASK;
				gpio_us_stop_time[US_LEFT] = systick_get_time_from_isr();
				gpio_us_distance[US_LEFT] = (HALF_SOUND_SPEED*(gpio_us_stop_time[US_LEFT] - gpio_us_start_time[US_LEFT]))/72000000LL;
			}
			gpio_us &= ~US_LEFT_MASK;
		}
	}

	portCLEAR_INTERRUPT_MASK();
}

void isr_exti3(void)
{
	portSET_INTERRUPT_MASK();

	if( EXTI->PR & EXTI_PR_PR3)
	{
		EXTI->PR |= EXTI_PR_PR3;

		if( GPIOC->IDR & GPIO_IDR_IDR3 )
		{
			if( (gpio_us & US_FRONT_MASK) == 0 && (gpio_us_triggered & US_FRONT_MASK))
			{
				// front montant sur PC3
				gpio_us_start_time[US_FRONT] = systick_get_time_from_isr();
			}
			gpio_us |= US_FRONT_MASK;
		}
		else
		{
			if( gpio_us & US_FRONT_MASK && (gpio_us_triggered & US_FRONT_MASK))
			{
				// front descendant sur PC3
				gpio_us_triggered &= ~US_FRONT_MASK;
				gpio_us_stop_time[US_FRONT] = systick_get_time_from_isr();
				gpio_us_distance[US_FRONT] = (HALF_SOUND_SPEED*(gpio_us_stop_time[US_FRONT] - gpio_us_start_time[US_FRONT]))/72000000LL;

			}
			gpio_us &= ~US_FRONT_MASK;
		}
	}

	portCLEAR_INTERRUPT_MASK();
}

void isr_exti9_5(void)
{
	portSET_INTERRUPT_MASK();

	if( EXTI->PR & EXTI_PR_PR5)
	{
		EXTI->PR |= EXTI_PR_PR5;
		if( GPIOC->IDR & GPIO_IDR_IDR5 )
		{
			if( (gpio_us & US_NA_MASK) == 0 && (gpio_us_triggered & US_NA_MASK))
			{
				// front montant sur PC5
				gpio_us_start_time[US_NA] = systick_get_time_from_isr();
			}
			gpio_us |= US_NA_MASK;
		}
		else
		{
			if( gpio_us & US_NA_MASK && (gpio_us_triggered & US_NA_MASK))
			{
				// front descendant sur PC5
				gpio_us_triggered &= ~US_NA_MASK;
				gpio_us_stop_time[US_NA] = systick_get_time_from_isr();
				gpio_us_distance[US_NA] = (HALF_SOUND_SPEED*(gpio_us_stop_time[US_NA] - gpio_us_start_time[US_NA]))/72000000LL;
			}
			gpio_us &= ~US_NA_MASK;
		}

		if( GPIOA->IDR & GPIO_IDR_IDR5 )
		{
			if( (gpio_us & US_BACK_MASK) == 0 && (gpio_us_triggered & US_BACK_MASK))
			{
				// front montant sur PA5
				gpio_us_start_time[US_BACK] = systick_get_time_from_isr();
			}
			gpio_us |= US_BACK_MASK;
		}
		else
		{
			if( gpio_us & US_BACK_MASK && (gpio_us_triggered & US_BACK_MASK))
			{
				// front descendant sur PA5
				gpio_us_triggered &= ~US_BACK_MASK;
				gpio_us_stop_time[US_BACK] = systick_get_time_from_isr();
				gpio_us_distance[US_BACK] = (HALF_SOUND_SPEED*(gpio_us_stop_time[US_BACK] - gpio_us_start_time[US_BACK]))/72000000LL;
			}
			gpio_us &= ~US_BACK_MASK;
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

	portSET_INTERRUPT_MASK();
}

void isr_exti15_10(void)
{
	portSET_INTERRUPT_MASK();

	if( EXTI->PR & EXTI_PR_PR10)
	{
		EXTI->PR |= EXTI_PR_PR10;
		// TODO action btn3
	}

	portSET_INTERRUPT_MASK();
}

void setLed(uint32_t mask)
{
	GPIOE->ODR = (GPIOE->ODR & ~((uint32_t)0x3F)) | (mask & 0x3F);
	GPIOB->ODR = (GPIOB->ODR & ~((uint32_t)LED_WARNING)) | (mask & LED_WARNING);
}

void gpio_get_us(uint16_t* us_distance, uint8_t size)
{
	portENTER_CRITICAL();
	if( size > sizeof(gpio_us_distance))
	{
		size = sizeof(gpio_us_distance);
	}
	memcpy(us_distance, gpio_us_distance, size);
	portEXIT_CRITICAL();
}

void gpio_activate_right_us()
{
	AFIO->EXTICR[0] = (AFIO->EXTICR[0] &~AFIO_EXTICR1_EXTI1_PB ) | AFIO_EXTICR1_EXTI1_PC;
}

void gpio_activate_left_us()
{
	AFIO->EXTICR[0] = (AFIO->EXTICR[0] &~AFIO_EXTICR1_EXTI1_PC ) | AFIO_EXTICR1_EXTI1_PB;
}

void gpio_send_us(uint8_t us_mask)
{
	if( us_mask & US_RIGHT_MASK)
	{
		GPIOC->ODR |= GPIO_ODR_ODR0;
		gpio_us_distance[US_RIGHT] = 0;
		gpio_us_triggered |= US_RIGHT_MASK;
	}

	if( us_mask & US_FRONT_MASK)
	{
		GPIOC->ODR |= GPIO_ODR_ODR2;
		gpio_us_distance[US_FRONT] = 0;
		gpio_us_triggered |= US_FRONT_MASK;
	}
	
	if( us_mask & US_BACK_MASK)
	{
		GPIOA->ODR |= GPIO_ODR_ODR4;
		gpio_us_distance[US_BACK] = 0;
		gpio_us_triggered |= US_BACK_MASK;
	}

	if( us_mask & US_NA_MASK)
	{
		GPIOC->ODR |= GPIO_ODR_ODR4;
		gpio_us_distance[US_NA] = 0;
		gpio_us_triggered |= US_NA_MASK;
	}

	if( us_mask & US_LEFT_MASK)
	{
		GPIOB->ODR |= GPIO_ODR_ODR0;
		gpio_us_distance[US_LEFT] = 0;
		gpio_us_triggered |= US_LEFT_MASK;
	}

	int i = 36;
	for( ; i-- ; )
	{
		nop();
	}

	if( us_mask & US_RIGHT_MASK)
	{
		GPIOC->ODR &= ~GPIO_ODR_ODR0;
	}

	if( us_mask & US_FRONT_MASK)
	{
		GPIOC->ODR &= ~GPIO_ODR_ODR2;
	}
	
	if( us_mask & US_BACK_MASK)
	{
		GPIOA->ODR &= ~GPIO_ODR_ODR4;
	}

	if( us_mask & US_NA_MASK)
	{
		GPIOC->ODR &= ~GPIO_ODR_ODR4;
	}

	if( us_mask & US_LEFT_MASK)
	{
		GPIOB->ODR &= ~GPIO_ODR_ODR0;
	}
}
