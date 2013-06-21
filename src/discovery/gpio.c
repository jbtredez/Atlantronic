#include "gpio.h"
#include "kernel/module.h"
#include "kernel/rcc.h"

static int gpio_module_init(void)
{
	// LED sur PD12 PD13 PD14 PD15
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	gpio_pin_init(GPIOD, 12, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
	gpio_pin_init(GPIOD, 13, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
	gpio_pin_init(GPIOD, 14, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
	gpio_pin_init(GPIOD, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);

	setLed(LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE);

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

void gpio_pin_init(GPIO_TypeDef* GPIOx, uint32_t pin, enum gpio_mode mode, enum gpio_speed speed, enum gpio_otype otype, enum gpio_pupd pupd)
{
	GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (2 * pin));
	GPIOx->MODER |= ((uint32_t)mode) << (2 * pin);

	if( mode == GPIO_MODE_OUT )
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
