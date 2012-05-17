#include "gpio.h"
#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/event.h"
#include "kernel/systick.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

volatile uint32_t color;
volatile uint8_t gpio_go;
volatile uint8_t gpio_recaler;
uint8_t gpio_recalage_done;

static void gpio_cmd_go();
static void gpio_cmd_color(void* arg);

static int gpio_module_init(void)
{
	color = COLOR_BLUE;
	gpio_go = 0;
	gpio_recaler = 0;

	// activation de GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	// PA4 entrée input flotante : capteur de fin de course du bas pour le bras
	GPIOA->CRL = (GPIOA->CRL & ~GPIO_CRL_MODE4 & ~GPIO_CRL_CNF4) | GPIO_CRL_CNF4_0;

	// capteur contact bordure droit et gauche, LED warning
	// activation GPIOB
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	// PB0 entrée input flotante
	GPIOB->CRL = (GPIOB->CRL & ~GPIO_CRL_MODE0 & ~GPIO_CRL_CNF0) | GPIO_CRL_CNF0_0;
	// PB1 entrée input flotante
	GPIOB->CRL = (GPIOB->CRL & ~GPIO_CRL_MODE1 & ~GPIO_CRL_CNF1) | GPIO_CRL_CNF1_0;
	// PB9 sortie push-pull, 2MHz
	GPIOB->CRH = (GPIOB->CRH & ~GPIO_CRH_MODE9 & ~GPIO_CRH_CNF9) | GPIO_CRH_MODE9_1;
	// PB12 sortie push-pull, 2MHz (moteur pas à pas - puissance)
	GPIOB->CRH = (GPIOB->CRH & ~GPIO_CRH_MODE12 & ~GPIO_CRH_CNF12) | GPIO_CRH_MODE12_1;
	// PB13 sortie push-pull, 2MHz (moteur pas à pas - sens du moteur)
	GPIOB->CRH = (GPIOB->CRH & ~GPIO_CRH_MODE13 & ~GPIO_CRH_CNF13) | GPIO_CRH_MODE13_1;
	// PB14 sortie push-pull, 2MHz (moteur pas à pas - commande des pas a faire)
	GPIOB->CRH = (GPIOB->CRH & ~GPIO_CRH_MODE14 & ~GPIO_CRH_CNF14) | GPIO_CRH_MODE14_1;

	// Boutons 1, 2 et 3
	// activation GPIOD
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	// PD8 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE8 & ~GPIO_CRH_CNF8) | GPIO_CRH_CNF8_0;
	// PD9 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE9 & ~GPIO_CRH_CNF9) | GPIO_CRH_CNF9_0;
	// PD10 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10) | GPIO_CRH_CNF10_0;

	// activation de GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	// PC4 sortie push pull, 2MHz
	GPIOC->CRL = (GPIOC->CRL & ~GPIO_CRL_MODE4 & ~GPIO_CRL_CNF4) | GPIO_CRL_MODE4_1;

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
	// PB0 et PB1 => it sur front montant et front descendant
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PB | AFIO_EXTICR1_EXTI1_PB;
	AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PE;
	AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PD | AFIO_EXTICR3_EXTI9_PD | AFIO_EXTICR3_EXTI10_PD;
	EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR6 | EXTI_IMR_MR8 | EXTI_IMR_MR9 | EXTI_IMR_MR10;
	EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1 | EXTI_RTSR_TR6 | EXTI_RTSR_TR8 | EXTI_RTSR_TR9 | EXTI_RTSR_TR10;
	EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1;

	NVIC_SetPriority(EXTI0_IRQn, PRIORITY_IRQ_EXTI0);
	NVIC_SetPriority(EXTI1_IRQn, PRIORITY_IRQ_EXTI1);
	NVIC_SetPriority(EXTI9_5_IRQn, PRIORITY_IRQ_EXTI9_5);
	NVIC_SetPriority(EXTI15_10_IRQn, PRIORITY_IRQ_EXTI15_10);
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	setLed(LED_0 | LED_1 | LED_2 | LED_3 | LED_4 | LED_5 | LED_WARNING);

	usb_add_cmd(USB_CMD_GO, &gpio_cmd_go);
	usb_add_cmd(USB_CMD_COLOR, &gpio_cmd_color);

	// on allume les led de la balise
	GPIOC->ODR |= GPIO_ODR_ODR4;

	gpio_recalage_done = 0;

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

static void gpio_cmd_go()
{
	if(gpio_recalage_done)
	{
		gpio_go = 1;
		setLed(0x23F);
		systick_start_match();
		vTaskSetEvent(EVENT_GO);
	}
}

static void gpio_cmd_color(void* arg)
{
	uint8_t new_color = *((uint8_t*) arg);
	if(gpio_go == 0)
	{
		if(new_color == COLOR_RED)
		{
			color = COLOR_RED;
			setLed(0x06);
			log(LOG_INFO, "couleur => rouge");
		}
		else
		{
			color = COLOR_BLUE;
			log(LOG_INFO, "couleur => bleu");
			setLed(0x30);
		}
	}
}

static portBASE_TYPE gpio_go_from_isr()
{
	portBASE_TYPE higher_priority_task_woken = 0;

	if( gpio_recalage_done )
	{
		gpio_go = 1;
		setLed(0x23F);
		systick_start_match_from_isr();
		higher_priority_task_woken = vTaskSetEventFromISR(EVENT_GO);
	}

	return higher_priority_task_woken;
}

void isr_exti0(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK();

	if( EXTI->PR & EXTI_PR_PR0)
	{
		EXTI->PR |= EXTI_PR_PR0;
		xHigherPriorityTaskWoken = vTaskSetEventFromISR(EVENT_SICK);
	}

	if( xHigherPriorityTaskWoken )
	{
		vPortYieldFromISR();
	}

	portCLEAR_INTERRUPT_MASK();
}

void isr_exti1(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK();

	if( EXTI->PR & EXTI_PR_PR1)
	{
		EXTI->PR |= EXTI_PR_PR1;
		xHigherPriorityTaskWoken = vTaskSetEventFromISR(EVENT_SICK);
	}

	if( xHigherPriorityTaskWoken )
	{
		vPortYieldFromISR();
	}

	portCLEAR_INTERRUPT_MASK();
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
		gpio_recalage_done = 1;
	}

	portCLEAR_INTERRUPT_MASK();
}

void setLed(uint32_t mask)
{
	GPIOE->ODR = (GPIOE->ODR & ~((uint32_t)0x3F)) | (mask & 0x3F);
	GPIOB->ODR = (GPIOB->ODR & ~((uint32_t)LED_WARNING)) | (mask & LED_WARNING);
}
