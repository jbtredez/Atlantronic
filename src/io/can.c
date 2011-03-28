#include "io/can.h"
#include "module.h"
#include "io/rcc.h"

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

	// init mode
	CAN1->MCR = CAN_MCR_INRQ;

	#if( RCC_PCLK1 != 36000000)
	#error "remettre RCC_PCLK1 à 36Mhz, sinon c'est le bordel pour recalculer BTR"
	#endif

	// TBS1 = 12 TQ   
	// TBS2 =  5 TQ
	// SJW  =  4 TQ
	// total bit can (1 + TBS1 + TBS2) = 18 TQ
	// SP = (1 + TBS1)/total = 72,22 %
	// vitesse : 500kb => 500000*18*TQ = PCLK = 36Mhz
	// => TQ = PCLK / (18 * 500000) = 4
	CAN1->BTR &= ~ (              CAN_BTR_SJW  |                  CAN_BTR_TS2  |                   CAN_BTR_TS1  |   CAN_BTR_BRP    );
	CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((4-1) & CAN_BTR_BRP);

	// TODO à voir / it... + handler
	//CAN1->IER
	//NVIC_EnableIRQ(CAN1_TX_IRQn);
	//NVIC_EnableIRQ(CAN1_RX0_IRQn);

	// lancement du CAN
	CAN1->MCR &= ~CAN_MCR_INRQ;
	//while (CAN->MSR & CAN_MCR_INRQ) ;

// TODO filtres / messages

	return 0;
}

module_init(can_module_init, INIT_CAN);