#include "io.h"
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
	enum gpio_mode mode;
	enum gpio_pupd pupd;
}IoParam;


// io sur les connecteurs io generiques
static const IoParam io_param[] =
{
	{GPIOC, 15, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_0 - couleur
	{GPIOC, 13, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_1 - recalage
	{GPIOE,  3, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_2 - pull up pour les omron
	{GPIOE,  4, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_3 - pull up pour les omron
	{GPIOG, 10, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_4 - pull up pour les omron
	{GPIOG, 11, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_5
	{GPIOD,  4, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_6
	{GPIOD,  7, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_7
	{GPIOD,  3, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_8
	{GPIOD,  2, GPIO_MODE_IN, GPIO_PUPD_UP}, // IO_9 - switch elevateur
	{GPIOG,  7, GPIO_MODE_OUT, GPIO_PUPD_UP}, // attention pull up 4k7 sur carte cpu - IO_10 - sortie
	{GPIOG,  6, GPIO_MODE_OUT, GPIO_PUPD_UP}, // IO_11 - sortie
	{GPIOC, 14, GPIO_MODE_IN, GPIO_PUPD_DOWN}, // go
};

static int io_module_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOGEN;

	// PARAMETRAGE IO
	unsigned int i;
	for(i = 0; i < sizeof(io_param)/sizeof(io_param[0]); i++)
	{
		// pull up pour les omron
		gpio_pin_init(io_param[i].gpio, io_param[i].pin, io_param[i].mode, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, io_param[i].pupd);
	}

	// boutons en IT sur front descendant sur le GO
	exti_register(EXTI_PC, 14, EXTI_TYPE_DOWN, match_go_from_isr);
	exti_register(EXTI_PC, 15, EXTI_TYPE_DOWN | EXTI_TYPE_UP, match_set_color_from_isr);

	return 0;
}

module_init(io_module_init, INIT_GPIO);

uint32_t gpio_get_state()
{
	uint32_t res = 0;

	unsigned int i = 0;
	for(i = 0; i < sizeof(io_param)/sizeof(io_param[0]); i++)
	{
		res |= gpio_get_pin(io_param[i].gpio, io_param[i].pin) << i;
	}

	res |= (match_get_go()?1:0) << i; // GO

	return res;
}

int gpio_get(Io io)
{
	return gpio_get_pin(io_param[io].gpio, io_param[io].pin) != 0;
}

void gpio_set(Io io)
{
	gpio_set_pin(io_param[io].gpio, io_param[io].pin);
}

void gpio_reset(Io io)
{
	gpio_reset_pin(io_param[io].gpio, io_param[io].pin);
}

