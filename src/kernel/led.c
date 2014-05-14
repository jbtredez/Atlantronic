#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/systick.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "led.h"
#include "gpio.h"

#define LED_STACK_SIZE           100

enum
{
	LED_MODE_BOOT,
	LED_MODE_WAIT_X86,
	LED_MODE_WAIT_COLOR_SELECTION,
	LED_MODE_COLOR_SELECTION,
	LED_MODE_WAIT_INIT,
	LED_MODE_MATCH_RUNNING,
};

static void led_task(void *arg);
static void led_two_half_chaser();
static void led_chaser();
static void led_tetris();

static int led_mode = LED_MODE_BOOT;
static int led_step;

static int led_module_init(void)
{
	// LED (carte CPU) sur PD14 PD15 (led verte et orange sur PD12 et PD13 non utilisables, encodeur dessus !!)
	// LED (carte led deportee) sur PC15, PC13, PE4, PE2, PB8
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
	gpio_pin_init(GPIOB, 8, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOC, 13, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOC, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOD, 14, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED rouge carte CPU
	gpio_pin_init(GPIOD, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED bleue carte CPU
	gpio_pin_init(GPIOE, 2, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOE, 4, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee

	setLed( LED_CPU_RED | LED_CPU_BLUE | LED_EXT_BLUE | LED_EXT_GREEN | LED_EXT_ORANGE1 | LED_EXT_ORANGE2 | LED_EXT_RED);

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(led_task, "led", LED_STACK_SIZE, NULL, PRIORITY_TASK_LED, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_LED;
	}

	led_mode = LED_MODE_WAIT_X86;

	return 0;
}

module_init(led_module_init, INIT_LED);

static void led_task(void *arg)
{
	(void) arg;

	while(1)
	{
		switch(led_mode)
		{
			case LED_MODE_WAIT_X86:
				led_two_half_chaser();
				break;
			case LED_MODE_WAIT_COLOR_SELECTION:
				led_tetris();
				break;
			case LED_MODE_COLOR_SELECTION:
				// TODO
				if(getcolor() == COLOR_RED)
				{
					setLed(LED_EXT_RED);
				}
				else
				{
					setLed(LED_EXT_ORANGE1);
				}
				break;
			case LED_MODE_WAIT_INIT:
				// TODO
				break;
			case LED_MODE_MATCH_RUNNING:
				led_chaser();
				break;
			default:
				break;
		}
		led_step++;
		vTaskDelay(ms_to_tick(100));
	}
}

void setLed(uint32_t mask)
{
	GPIOB->BSRRL = (uint16_t)((mask & LED_EXT_RED) >> 16);
	GPIOB->BSRRH = (uint16_t)(((~mask) & LED_EXT_RED) >> 16);

	GPIOC->BSRRL = (uint16_t)((mask & (LED_EXT_BLUE | LED_EXT_GREEN)) >> 16);
	GPIOC->BSRRH = (uint16_t)(((~mask) & (LED_EXT_BLUE | LED_EXT_GREEN)) >> 16);

	GPIOD->BSRRL = (uint16_t)(mask & (LED_CPU_RED | LED_CPU_BLUE));
	GPIOD->BSRRH = (uint16_t)((~mask) & ( LED_CPU_RED | LED_CPU_BLUE));

	GPIOE->BSRRL = (uint16_t)((mask & (LED_EXT_ORANGE1 | LED_EXT_ORANGE2)) >> 16);
	GPIOE->BSRRH = (uint16_t)(((~mask) & (LED_EXT_ORANGE1 | LED_EXT_ORANGE2)) >> 16);
}

static void led_two_half_chaser()
{
	switch(led_step)
	{
		default:
			led_step = 0;
		case 0:
			setLed(LED_EXT_BLUE | LED_EXT_RED);
			break;
		case 1:
			setLed(LED_EXT_GREEN | LED_EXT_ORANGE2);
			break;
		case 2:
			setLed(LED_EXT_ORANGE1);
			break;
	}
}

static void led_chaser()
{
	switch(led_step)
	{
		default:
			led_step = 0;
		case 0:
			setLed(LED_EXT_BLUE);
			break;
		case 1:
			setLed(LED_EXT_GREEN);
			break;
		case 2:
			setLed(LED_EXT_ORANGE1);
			break;
		case 3:
			setLed(LED_EXT_ORANGE2);
			break;
		case 4:
			setLed(LED_EXT_RED);
			break;
	}
}

static void led_tetris()
{
	switch(led_step)
	{
		default:
			led_step = 0;
		case 0:
			setLed(LED_EXT_RED);
			break;
		case 1:
			setLed(LED_EXT_RED | LED_EXT_ORANGE2);
			break;
		case 2:
			setLed(LED_EXT_RED | LED_EXT_ORANGE2 | LED_EXT_ORANGE1);
			break;
		case 3:
			setLed(LED_EXT_RED | LED_EXT_ORANGE2 | LED_EXT_ORANGE1 | LED_EXT_GREEN);
			break;
		case 4:
			setLed(LED_EXT_RED | LED_EXT_ORANGE2 | LED_EXT_ORANGE1 | LED_EXT_GREEN | LED_EXT_BLUE);
			break;
	}
}
