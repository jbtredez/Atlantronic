#include "arch/arm_cm3/gpio.h"
#include "module.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event.h"
#include "io/systick.h"

volatile uint32_t color;
volatile uint8_t gpio_go;

static int gpio_module_init(void)
{
	color = COLOR_BLUE;
	gpio_go = 0;

	// LED warning
	// activation GPIOB
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	// PE0 sortie push-pull, 2MHz
	GPIOB->CRH = (GPIOB->CRH & ~GPIO_CRH_MODE9 & ~GPIO_CRH_CNF9) | GPIO_CRH_MODE9_1;

	// activation GPIOD
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	// PD8 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE8 & ~ GPIO_CRH_CNF8_1) | GPIO_CRH_CNF8_0;
	// PD9 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE9 & ~ GPIO_CRH_CNF9_1) | GPIO_CRH_CNF9_0;
	// PD10 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE10 & ~ GPIO_CRH_CNF10_1) | GPIO_CRH_CNF10_0;

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

	// PD8 => it sur front montant
	// PD9 => it sur front montant
	// PD10 => it sur front montant
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PD | AFIO_EXTICR3_EXTI9_PD | AFIO_EXTICR3_EXTI10_PD;
	EXTI->IMR |= EXTI_IMR_MR8 | EXTI_IMR_MR9 | EXTI_IMR_MR10;
	EXTI->RTSR |= EXTI_RTSR_TR8 | EXTI_RTSR_TR9 | EXTI_RTSR_TR10;
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	setLed(LED_0 | LED_1 | LED_2 | LED_3 | LED_4 | LED_5 | LED_WARNING);

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

void isr_exit9_5(void)
{
	if( EXTI->PR & EXTI_PR_PR8)
	{
		EXTI->PR |= EXTI_PR_PR8;
		gpio_go = 1;
		setLed(0x23F);
		systick_start_match();
		vTaskSetEventFromISR(EVENT_GO);
	}
	if( EXTI->PR & EXTI_PR_PR9)
	{
		EXTI->PR |= EXTI_PR_PR9;
		if(gpio_go == 0)
		{
			if(color == COLOR_BLUE)
			{
				color = COLOR_RED;
				setLed(0x06);
			}
			else
			{
				color = COLOR_BLUE;
				setLed(0x30);
			}
		}
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