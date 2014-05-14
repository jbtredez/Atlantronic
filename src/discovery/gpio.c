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
static uint8_t gpio_enable_go;

static xQueueHandle gpio_queue_go;

static void gpio_cmd_go(void* arg);
static void gpio_cmd_color(void* arg);

static int gpio_module_init(void)
{
	// io "sorties"
	// puissance on/off sur PB2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
	gpio_pin_init(GPIOB, 2, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // on/off

	// "io entrees"
	// boutons USR1 et USR2 carte led sur PC14 et PB7
	// bouton go sur PD3
	gpio_pin_init(GPIOB, 7, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton USR2
	gpio_pin_init(GPIOC, 14, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton USR1
	gpio_pin_init(GPIOD, 3, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton go

	// io sur les 5 connecteurs io generiques
	// pull up pour les omron
	// PD11 : IN_1
	// PB13 : IN_2
	// PB12 : IN_3
	// PD10 : IN_4
	// PD7  : IN_5
	// PB11 : IN_6
	// PC11 : IN_7
	// PD6  : IN_8
	// PC9  : IN_9
	// PC8  : IN_10
	// PB14 : IN_11
	// PB15 : IN_12
	// PE6  : IN_13
	// PE5  : IN_14
	gpio_pin_init(GPIOD, 11, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_1
	gpio_pin_init(GPIOB, 13, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_2
	gpio_pin_init(GPIOB, 12, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_3
	gpio_pin_init(GPIOD, 10, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_4
	gpio_pin_init(GPIOD, 7, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_5
	gpio_pin_init(GPIOB, 11, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_6
	gpio_pin_init(GPIOC, 11, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_7
	gpio_pin_init(GPIOD, 6, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_8
	gpio_pin_init(GPIOC, 9, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_9
	gpio_pin_init(GPIOC, 8, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_10
	gpio_pin_init(GPIOB, 14, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_11
	gpio_pin_init(GPIOB, 15, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_12
	gpio_pin_init(GPIOE, 6, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_13
	gpio_pin_init(GPIOE, 5, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_14

	color = COLOR_RED;
	gpio_go = 0;
	gpio_queue_go = xQueueCreate(1, 0);

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

	gpio_power_off();

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

uint32_t gpio_get_state()
{
	uint32_t res = 0;

	res |= gpio_get_pin(GPIOD, 11);      // IN_1
	res |= gpio_get_pin(GPIOB, 13) << 1; // IN_2
	res |= gpio_get_pin(GPIOB, 12) << 2; // IN_3
	res |= gpio_get_pin(GPIOD, 10) << 3; // IN_4
	res |= gpio_get_pin(GPIOD, 7) << 4;  // IN_5
	res |= gpio_get_pin(GPIOB, 11) << 5; // IN_6
	res |= gpio_get_pin(GPIOC, 11) << 6; // IN_7
	res |= gpio_get_pin(GPIOD, 6) << 7;  // IN_8
	res |= gpio_get_pin(GPIOC, 9) << 8;  // IN_9
	res |= gpio_get_pin(GPIOC, 8) << 9;  // IN_10
	res |= gpio_get_pin(GPIOB, 14) << 10;  // IN_11
	res |= gpio_get_pin(GPIOB, 15) << 11;  // IN_12
	res |= gpio_get_pin(GPIOE, 6) << 12;  // IN_13
	res |= gpio_get_pin(GPIOE, 5) << 13;  // IN_14
	res |= gpio_get_pin(GPIOC, 14) << 14;  // IN_BTN1
	res |= gpio_get_pin(GPIOB, 7) << 15;  // IN_BTN2
	res |= gpio_get_pin(GPIOD, 3) << 16; // INGO
	res |= (gpio_go?1:0) << 17; // GO
	res |= (color?1:0) << 18; // color

	return res;
}

void gpio_wait_go()
{
	xQueuePeek(gpio_queue_go, NULL, portMAX_DELAY);
}

static void gpio_cmd_go(void * arg)
{
	struct gpio_cmd_go_arg* cmd_arg = (struct gpio_cmd_go_arg*) arg;

	switch(cmd_arg->cmd)
	{
		case GPIO_CMD_ENABLE_GO:
			gpio_enable_go = 1;
			log(LOG_INFO, "enable GO");
			break;
		case GPIO_CMD_GO:
			if(gpio_enable_go)
			{
				log(LOG_INFO, "usb go");
				gpio_go = 1;
				//setLed(LED_CPU_RED | LED_CPU_BLUE);
				systick_start_match();
				xQueueSend(gpio_queue_go, NULL, 0);
			}
			break;
		default:
			log_format(LOG_ERROR, "unknown go cmd %d", cmd_arg->cmd);
			break;
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
			//setLed(LED_CPU_RED | LED_EXT_RED);
			log(LOG_INFO, "couleur => rouge");
		}
		else
		{
			color = COLOR_YELLOW;
			//setLed(LED_CPU_BLUE | LED_EXT_ORANGE1);
			log(LOG_INFO, "couleur => jaune");
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
		if( gpio_enable_go )
		{
			gpio_go = 1;
			//setLed(LED_CPU_RED | LED_CPU_BLUE);
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
			if(color == COLOR_YELLOW)
			{
				color = COLOR_RED;
				//setLed(LED_CPU_RED | LED_EXT_RED);
			}
			else
			{
				color = COLOR_YELLOW;
				//setLed(LED_CPU_BLUE | LED_EXT_ORANGE1);
			}
		}
	}

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}
