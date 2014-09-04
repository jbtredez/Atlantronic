#include "bsp.h"
#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/FreeRTOS.h"
#include "kernel/systick.h"
#include "kernel/driver/gpio.h"
#include "kernel/driver/exti.h"
#include "kernel/log.h"
#include "kernel/match.h"

static portBASE_TYPE gpio_usr2_isr(void);

typedef struct
{
	GPIO_TypeDef* gpio;
	uint32_t pin;
	enum gpio_pupd pupd;
}BspIo;


// io sur les connecteurs io generiques
static const BspIo bsp_io[] =
{
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
};

static int gpio_module_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

	// "io entrees"
	unsigned int i;
	for(i = 0; i < sizeof(bsp_io)/sizeof(bsp_io[0]); i++)
	{
		// pull up pour les omron
		gpio_pin_init(bsp_io[i].gpio, bsp_io[i].pin, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, bsp_io[i].pupd);
	}

	// boutons en IT sur front montant : USR1 et USR2
	exti_register(EXTI_PC, 14, EXTI_TYPE_UP, match_set_color_from_isr);
	exti_register(EXTI_PB,  7, EXTI_TYPE_UP, gpio_usr2_isr);

	// boutons en IT sur front descendant sur le GO
	exti_register(EXTI_PD,  3, EXTI_TYPE_DOWN, match_go_from_isr);

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

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

portBASE_TYPE gpio_usr2_isr(void)
{
	return 0;
}

