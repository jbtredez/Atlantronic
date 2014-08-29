#include "gpio.h"
#include "kernel/module.h"
#include "kernel/rcc.h"

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
