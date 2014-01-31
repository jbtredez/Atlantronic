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
	// LED sur PD12 PD13 PD14 PD15
	// puissance on/off sur PB2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
	gpio_pin_init(GPIOD, 12, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
	gpio_pin_init(GPIOD, 13, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
	gpio_pin_init(GPIOD, 14, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
	gpio_pin_init(GPIOD, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
	gpio_pin_init(GPIOB, 2, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);

	setLed(LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE);

	color = COLOR_BLUE;
	gpio_go = 0;
	gpio_recaler = 0;
	gpio_queue_go = xQueueCreate(1, 0);
//	gpio_recalage_done = 0;

	usb_add_cmd(USB_CMD_GO, &gpio_cmd_go);
	usb_add_cmd(USB_CMD_COLOR, &gpio_cmd_color);

	// TODO configurer IT sur go et recalage
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
