#include "arch/arm_cm3/gpio.h"
#include "module.h"

static int gpio_module_init(void)
{
	// activation GPIOD
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	// PD9 entrée input flotante
	GPIOD->CRH = (GPIOD->CRH & ~GPIO_CRH_MODE9 & ~ GPIO_CRH_CNF9_1) | GPIO_CRH_CNF9_0;

	// LED sur PE0, PE1, PE2, PE3, PE4, PE5
	// activation GPIOE
	RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
	// PE0 sortie push-pull, 2MHz
	GPIOE->CRL &= (GPIOE->CRL & ~GPIO_CRL_MODE0 & ~GPIO_CRL_CNF0) | GPIO_CRL_MODE0_1;
	// PE1 sortie push-pull, 2MHz
	GPIOE->CRL &= (GPIOE->CRL & ~GPIO_CRL_MODE1 & ~GPIO_CRL_CNF1) | GPIO_CRL_MODE1_1;
	// PE2 sortie push-pull, 2MHz
	GPIOE->CRL &= (GPIOE->CRL & ~GPIO_CRL_MODE2 & ~GPIO_CRL_CNF2) | GPIO_CRL_MODE2_1;
	// PE3 sortie push-pull, 2MHz
	GPIOE->CRL &= (GPIOE->CRL & ~GPIO_CRL_MODE3 & ~GPIO_CRL_CNF3) | GPIO_CRL_MODE3_1;
	// PE4 sortie push-pull, 2MHz
	GPIOE->CRL &= (GPIOE->CRL & ~GPIO_CRL_MODE4 & ~GPIO_CRL_CNF4) | GPIO_CRL_MODE4_1;
	// PE5 sortie push-pull, 2MHz
	GPIOE->CRL &= (GPIOE->CRL & ~GPIO_CRL_MODE5 & ~GPIO_CRL_CNF5) | GPIO_CRL_MODE5_1;

	setLed(0x3F);

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);
