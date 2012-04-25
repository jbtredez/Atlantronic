#include "gpio.h"
#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/event.h"
#include "kernel/systick.h"
#include "kernel/driver/usb.h"

volatile uint32_t color;
volatile uint8_t gpio_go;
volatile uint8_t gpio_recaler;

static void gpio_cmd_go();

static int gpio_module_init(void)
{
	color = COLOR_BLUE;
	gpio_go = 0;
	gpio_recaler = 0;

	// capteur contact bordure droit et gauche, LED warning
	// activation GPIOB
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	// PB0 entrée input flotante
	// PB1 entrée input flotante
	GPIOB->CRL = ( GPIOB->CRL & ~(
	                GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
	                GPIO_CRL_MODE1 | GPIO_CRL_CNF1
		         ) ) |
		            GPIO_CRL_CNF0_0 |
		            GPIO_CRL_CNF1_0 ;

	// PB9 sortie push-pull, 2MHz
	GPIOB->CRH = (GPIOB->CRH & ~GPIO_CRH_MODE9 & ~GPIO_CRH_CNF9) | GPIO_CRH_MODE9_1;

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

	// PE6 : entree input flotante ( bouton go )
	GPIOE->CRL = (GPIOE->CRL & ~GPIO_CRL_MODE6 & ~GPIO_CRL_CNF6) | GPIO_CRL_CNF6_0;

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	// PD8, PD9, PD10 => it sur front montant
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PE;
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PD | AFIO_EXTICR3_EXTI9_PD | AFIO_EXTICR3_EXTI10_PD;
	EXTI->IMR |= EXTI_IMR_MR6 | EXTI_IMR_MR8 | EXTI_IMR_MR9 | EXTI_IMR_MR10;
	EXTI->RTSR |= EXTI_RTSR_TR6 | EXTI_RTSR_TR8 | EXTI_RTSR_TR9 | EXTI_RTSR_TR10;

	NVIC_SetPriority(EXTI9_5_IRQn, PRIORITY_IRQ_EXTI9_5);
	NVIC_SetPriority(EXTI15_10_IRQn, PRIORITY_IRQ_EXTI15_10);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	setLed(LED_0 | LED_1 | LED_2 | LED_3 | LED_4 | LED_5 | LED_WARNING);

	usb_add_cmd(USB_CMD_GO, &gpio_cmd_go);

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

static void gpio_cmd_go()
{
	gpio_go = 1;
	setLed(0x23F);
	systick_start_match();
	vTaskSetEvent(EVENT_GO);
}

static portBASE_TYPE gpio_go_from_isr()
{
	gpio_go = 1;
	setLed(0x23F);
	systick_start_match_from_isr();
	return vTaskSetEventFromISR(EVENT_GO);
}

void isr_exti9_5(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK();

	if( EXTI->PR & EXTI_PR_PR8)
	{
		EXTI->PR |= EXTI_PR_PR8;
		xHigherPriorityTaskWoken = gpio_go_from_isr();
	}
	if( EXTI->PR & EXTI_PR_PR6)
	{
		EXTI->PR |= EXTI_PR_PR6;
		xHigherPriorityTaskWoken = gpio_go_from_isr();
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

	if( xHigherPriorityTaskWoken )
	{
		vPortYieldFromISR();
	}

	portCLEAR_INTERRUPT_MASK();
}

void isr_exti15_10(void)
{
	portSET_INTERRUPT_MASK();

	if( EXTI->PR & EXTI_PR_PR10)
	{
		EXTI->PR |= EXTI_PR_PR10;
		gpio_recaler = 1;
	}

	portCLEAR_INTERRUPT_MASK();
}

void setLed(uint32_t mask)
{
	GPIOE->ODR = (GPIOE->ODR & ~((uint32_t)0x3F)) | (mask & 0x3F);
	GPIOB->ODR = (GPIOB->ODR & ~((uint32_t)LED_WARNING)) | (mask & LED_WARNING);
}
