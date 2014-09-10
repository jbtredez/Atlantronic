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
	enum gpio_pupd pupd;
}IoParam;


// io sur les connecteurs io generiques
static const IoParam io_input[] =
{
#if defined(__discovery__)
	{GPIOD, 11, GPIO_PUPD_UP}, // PD11 : IN_1
	{GPIOB, 13, GPIO_PUPD_UP}, // PB13 : IN_2
	{GPIOB, 12, GPIO_PUPD_UP}, // PB12 : IN_3
	{GPIOD, 10, GPIO_PUPD_UP}, // PD10 : IN_4
	{GPIOD,  7, GPIO_PUPD_UP}, // PD7  : IN_5
	{GPIOB, 11, GPIO_PUPD_UP}, // PB11 : IN_6
	{GPIOC, 11, GPIO_PUPD_UP}, // PC11 : IN_7
	{GPIOD,  6, GPIO_PUPD_UP}, // PD6  : IN_8
	{GPIOC,  9, GPIO_PUPD_UP}, // PC9  : IN_9
	{GPIOC,  8, GPIO_PUPD_UP}, // PC8  : IN_10
	{GPIOB, 14, GPIO_PUPD_UP}, // PB14 : IN_11
	{GPIOB, 15, GPIO_PUPD_UP}, // PB15 : IN_12
	{GPIOE,  6, GPIO_PUPD_UP}, // PE6  : IN_13
	{GPIOE,  5, GPIO_PUPD_UP}, // PE5  : IN_14
	{GPIOB,  7, GPIO_PUPD_DOWN}, // bouton USR2
	{GPIOC, 14, GPIO_PUPD_DOWN}, // bouton USR1
	{GPIOD,  3, GPIO_PUPD_DOWN}, // bouton go
#elif defined(__disco__)
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
	{GPIOG,  7, GPIO_PUPD_UP}, // attention pull up 4k7 sur carte cpu - IN_10
	{GPIOG,  6, GPIO_PUPD_UP}, // IN_11
	{GPIOC, 14, GPIO_PUPD_DOWN}, // go
#else
#error unknown card
#endif
};

static int io_module_init(void)
{
#if defined(__discovery__)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
#elif defined(__disco__)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOGEN;
#else
#error unknown card
#endif

	// "io entrees"
	unsigned int i;
	for(i = 0; i < sizeof(io_input)/sizeof(io_input[0]); i++)
	{
		// pull up pour les omron
		gpio_pin_init(io_input[i].gpio, io_input[i].pin, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, io_input[i].pupd);
	}

#if defined(__discovery__)
	// boutons en IT sur front montant : USR1 et USR2
	exti_register(EXTI_PC, 14, EXTI_TYPE_UP, match_set_color_from_isr);
	//exti_register(EXTI_PB,  7, EXTI_TYPE_UP, gpio_usr2_isr);

	// boutons en IT sur front descendant sur le GO
	exti_register(EXTI_PD,  3, EXTI_TYPE_DOWN, match_go_from_isr);
#elif defined(__disco__)
	// boutons en IT sur front descendant sur le GO
	exti_register(EXTI_PC, 14, EXTI_TYPE_DOWN, match_go_from_isr);
#else
#error unknown card
#endif

	return 0;
}

module_init(io_module_init, INIT_GPIO);

uint32_t gpio_get_state()
{
	uint32_t res = 0;

	unsigned int i = 0;
	for(i = 0; i < sizeof(io_input)/sizeof(io_input[0]); i++)
	{
		res |= gpio_get_pin(io_input[i].gpio, io_input[i].pin) << i;
	}

	res |= (match_get_go()?1:0) << i; // GO

	return res;
}
