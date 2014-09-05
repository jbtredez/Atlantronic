#include "bsp.h"
#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/FreeRTOS.h"
#include "kernel/systick.h"
#include "kernel/driver/gpio.h"
#include "kernel/driver/exti.h"
#include "kernel/log.h"
#include "kernel/match.h"

typedef struct
{
	GPIO_TypeDef* gpio;
	uint32_t pin;
	enum gpio_pupd pupd;
}BspIo;

// io sur les connecteurs io generiques
static const BspIo bsp_io[] =
{
	{GPIOC, 15, GPIO_PUPD_UP}, // IN_0
	{GPIOC, 13, GPIO_PUPD_UP}, // IN_1
	{GPIOE,  3, GPIO_PUPD_UP}, // IN_2
	{GPIOE,  4, GPIO_PUPD_UP}, // IN_3
	{GPIOG, 10, GPIO_PUPD_UP}, // IN_4
	{GPIOG, 11, GPIO_PUPD_UP}, // IN_5
	{GPIOD,  4, GPIO_PUPD_UP}, // IN_6
	{GPIOD,  7, GPIO_PUPD_UP}, // IN_7
	{GPIOD,  3, GPIO_PUPD_UP}, // IN_8
	{GPIOD,  2, GPIO_PUPD_UP}, // IN_9
	{GPIOG,  7, GPIO_PUPD_UP}, // IN_10
	{GPIOG,  6, GPIO_PUPD_UP}, // IN_11
	{GPIOC, 14, GPIO_PUPD_DOWN}, // go
};

static int bsp_module_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOGEN;

	// "io entrees"
	unsigned int i;
	for(i = 0; i < sizeof(bsp_io)/sizeof(bsp_io[0]); i++)
	{
		// pull up pour les omron
		gpio_pin_init(bsp_io[i].gpio, bsp_io[i].pin, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, bsp_io[i].pupd);
	}

	// boutons en IT sur front descendant sur le GO
	exti_register(EXTI_PC, 14, EXTI_TYPE_DOWN, match_go_from_isr);

	return 0;
}

module_init(bsp_module_init, INIT_GPIO);

uint32_t gpio_get_state()
{
	uint32_t res = 0;

	unsigned int i = 0;
	for(i = 0; i < sizeof(bsp_io)/sizeof(bsp_io[0]); i++)
	{
		res |= gpio_get_pin(bsp_io[i].gpio, bsp_io[i].pin) << i;
	}

	res |= (match_get_go()?1:0) << i; // GO

	return res;
}
