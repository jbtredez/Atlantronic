#include "gpio.h"
#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/FreeRTOS.h"
#include "kernel/queue.h"
#include "kernel/systick.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

volatile uint32_t color;
volatile uint8_t gpio_go;
volatile uint8_t gpio_recaler;
//uint8_t gpio_recalage_done;
static xQueueHandle gpio_queue_go;

static void gpio_cmd_go();
static void gpio_cmd_color(void* arg);

static int gpio_module_init(void)
{
	// io "sorties"
	// LED (carte CPU) sur PD12 PD13 PD14 PD15
	// puissance on/off sur PB2
	// LED (carte led deportee) sur PC15, PC13, PE4, PE2, PB8
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
	gpio_pin_init(GPIOB, 2, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // on/off
	gpio_pin_init(GPIOB, 8, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOC, 13, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOC, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOD, 12, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte CPU
	gpio_pin_init(GPIOD, 13, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte CPU
	gpio_pin_init(GPIOD, 14, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte CPU
	gpio_pin_init(GPIOD, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte CPU
	gpio_pin_init(GPIOE, 2, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOE, 4, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee

	// "io entrees"
	// boutons USR1 et USR2 carte led sur PC14 et PB7
	// bouton go sur PD3
	gpio_pin_init(GPIOB, 7, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton USR2
	gpio_pin_init(GPIOC, 14, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton USR1
	gpio_pin_init(GPIOD, 3, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton go

	setLed(LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE);

	color = COLOR_BLUE;
	gpio_go = 0;
	gpio_recaler = 0;
	gpio_queue_go = xQueueCreate(1, 0);
//	gpio_recalage_done = 0;

	usb_add_cmd(USB_CMD_GO, &gpio_cmd_go);
	usb_add_cmd(USB_CMD_COLOR, &gpio_cmd_color);

	// boutons en IT sur front montant : USR1 et USR2 et GO
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PD;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PB;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PC;
	EXTI->IMR |= EXTI_IMR_MR3 | EXTI_IMR_MR7 | EXTI_IMR_MR14;
	EXTI->RTSR |= EXTI_RTSR_TR3 | EXTI_RTSR_TR7 | EXTI_RTSR_TR14;

	NVIC_SetPriority(EXTI3_IRQn, PRIORITY_IRQ_EXTI3);
	NVIC_SetPriority(EXTI9_5_IRQn, PRIORITY_IRQ_EXTI9_5);
	NVIC_SetPriority(EXTI15_10_IRQn, PRIORITY_IRQ_EXTI15_10);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	gpio_power_on();

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

void gpio_pin_init(GPIO_TypeDef* GPIOx, uint32_t pin, enum gpio_mode mode, enum gpio_speed speed, enum gpio_otype otype, enum gpio_pupd pupd)
{
	GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (2 * pin));
	GPIOx->MODER |= ((uint32_t)mode) << (2 * pin);

	if( mode == GPIO_MODE_OUT || mode == GPIO_MODE_AF )
	{
		GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (2 * pin));
		GPIOx->OSPEEDR |= ((uint32_t)speed) << (2 * pin);

		GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << pin);
		GPIOx->OTYPER |= ((uint16_t)otype) << pin;
	}

	GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
	GPIOx->PUPDR |= ((uint32_t)pupd) << (pin * 2);
}

void gpio_af_config(GPIO_TypeDef* GPIOx, uint32_t pin, uint32_t gpio_af)
{
	uint32_t temp = gpio_af << ((pin & 0x07) * 4);
	GPIOx->AFR[pin >> 0x03] &= ~(0xF << ((pin & 0x07) * 4));
	uint32_t temp_2 = GPIOx->AFR[pin >> 0x03] | temp;
	GPIOx->AFR[pin >> 0x03] = temp_2;
}

void setLed(uint16_t mask)
{
	GPIOD->BSRRL = mask & (LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE);
	GPIOD->BSRRH = (~mask) & (LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE);
}


void gpio_wait_go()
{
	xQueuePeek(gpio_queue_go, NULL, portMAX_DELAY);
}

static void gpio_cmd_go()
{
//	if(gpio_recalage_done)
	{
		gpio_go = 1;
		//setLed(LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE);
		systick_start_match();
		xQueueSend(gpio_queue_go, NULL, 0);
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
			setLed(LED_RED);
			log(LOG_INFO, "couleur => rouge");
		}
		else
		{
			color = COLOR_BLUE;
			setLed(LED_BLUE);
			log(LOG_INFO, "couleur => bleu");
		}
	}
}

void isr_exti3(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR3)
	{
		EXTI->PR = EXTI_PR_PR3;
		//if( gpio_recalage_done )
		{
			gpio_go = 1;
			//setLed(LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE);
			systick_start_match_from_isr();
			xQueueSendFromISR(gpio_queue_go, NULL, &xHigherPriorityTaskWoken);
		}
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti9_5(void)
{
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR7)
	{
		EXTI->PR = EXTI_PR_PR7;
		// TODO action bouton USR2
	}

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti15_10(void)
{
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR14)
	{
		EXTI->PR = EXTI_PR_PR14;
		if(gpio_go == 0)
		{
			if(color == COLOR_BLUE)
			{
				color = COLOR_RED;
				setLed(LED_RED);
			}
			else
			{
				color = COLOR_BLUE;
				setLed(LED_BLUE);
			}
		}
	}

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}
