#include "io/can.h"
#include "module.h"


static int can_module_init(void)
{
	// CAN_RX : PD0
	// CAN_TX : PD1
	// => can 1 remap 3

	// activation clock sur afio pour remaper le can
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
	AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP3;

	// activation GPIOD
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	// RX (PD0) entrée pull-up / pull-down
	GPIOD->CRL = (GPIOD->CRL & ~GPIO_CRL_MODE0 & ~ GPIO_CRL_CNF0) | GPIO_CRL_CNF0_1;
	// TX (PD1) sortie alternate push-pull 50Mhz
	GPIOD->CRL = (GPIOD->CRL & ~GPIO_CRL_MODE1 & ~ GPIO_CRL_CNF1) | GPIO_CRL_CNF1_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE1_0;

	// activation clock sur le can 1
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	// TODO : activer it
	// TODO : CAN->MCR et CAN->BTR

	return 0;
}

module_init(can_module_init, INIT_CAN);