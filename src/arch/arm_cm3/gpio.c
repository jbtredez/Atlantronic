#include "arch/arm_cm3/gpio.h"
#include "module.h"

static int gpio_module_init(void)
{
	// activation GPIOD
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	// PD9 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE9 & ~ GPIO_CRH_CNF9_1) | GPIO_CRH_CNF9_0;

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);
