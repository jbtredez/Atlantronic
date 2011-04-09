#include "io/can.h"
#include "module.h"
#include "io/rcc.h"

// TODO : ecriture multiple
void can_write(struct can_msg *msg);
void can_set_filter(unsigned int id, unsigned char format);
static unsigned short can_filter_id;

static int can_module_init(void)
{
	can_filter_id = 0;

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
	CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_TMEIE;
	NVIC_EnableIRQ(CAN1_TX_IRQn);
	NVIC_EnableIRQ(CAN1_RX0_IRQn);

	// mode self-test pour le debug
	//CAN1->BTR |= CAN_BTR_SILM | CAN_BTR_LBKM;

	// lancement du CAN
	CAN1->MCR &= ~CAN_MCR_INRQ;
	// TODO attendre que c'est prêt
	//while (CAN1->MSR & CAN_MCR_INRQ) ;

// TODO filtres / messages

	return 0;
}

module_init(can_module_init, INIT_CAN);

void isr_can1_tx(void)
{
	// fin de transmission sur la boite 0
	if(CAN1->TSR & CAN_TSR_RQCP0)
	{
		CAN1->TSR |= CAN_TSR_RQCP0;
		CAN1->IER &= ~CAN_IER_TMEIE;
	}
}

// TODO : tests
struct can_msg msg_rx0;

void isr_can1_rx0(void)
{
	// TODO : tests
	struct can_msg* msg = &msg_rx0;

	// reception sur la FIFO 0
	if(CAN1->RF0R & CAN_RF0R_FMP0)
	{
		if((CAN1->sFIFOMailBox[0].RIR & (uint32_t)0x00000004) == 0)
		{
			// ID standard
			msg->format = CAN_STANDARD_FORMAT;
			msg->id = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
		}
		else
		{
			// ID étendu
			msg->format = CAN_EXTENDED_FORMAT;
			msg->id = (uint32_t)0x0003FFFF & (CAN1->sFIFOMailBox[0].RIR >> 3);
		}

		if ((CAN1->sFIFOMailBox[0].RIR & (uint32_t)0x00000002) == 0)
		{
			msg->type = CAN_DATA_FRAME;
		}
		else
		{
			msg->type = CAN_REMOTE_FRAME;
		}

		msg->size = (uint8_t)0x0000000F & CAN1->sFIFOMailBox[0].RDTR;

		// TODO : faire mieux pour la copie en 2 copie 32 bits
		msg->data[0] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);
		msg->data[1] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
		msg->data[2] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
		msg->data[3] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 24);

		msg->data[4] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR);
		msg->data[5] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
		msg->data[6] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
		msg->data[7] = (uint32_t)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 24);

		CAN1->RF0R |= CAN_RF0R_RFOM0;
	}
}

void can_write(struct can_msg *msg)
{
	CAN1->sTxMailBox[0].TIR  = 0;

	if (msg->format == CAN_STANDARD_FORMAT)
	{
		CAN1->sTxMailBox[0].TIR |= (unsigned int)(msg->id << 21);
	}
	else
	{
		CAN1->sTxMailBox[0].TIR |= (unsigned int)(msg->id <<  3) | 0x04;
	}

	if (msg->type == CAN_REMOTE_FRAME)
	{
		CAN1->sTxMailBox[0].TIR |= 0x02;
	}

	// TODO remplissage par 32 bit
	CAN1->sTxMailBox[0].TDLR = (((unsigned int)msg->data[3] << 24) | ((unsigned int)msg->data[2] << 16) | ((unsigned int)msg->data[1] <<  8) | ((unsigned int)msg->data[0]) );
	CAN1->sTxMailBox[0].TDHR = (((unsigned int)msg->data[7] << 24) | ((unsigned int)msg->data[6] << 16) | ((unsigned int)msg->data[5] <<  8) | ((unsigned int)msg->data[4]) );

	CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
	CAN1->sTxMailBox[0].TDTR |= msg->size & CAN_TDT0R_DLC;

	CAN1->IER |= CAN_IER_TMEIE;
	CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}

void can_set_filter(unsigned int id, unsigned char format)
{
	uint32_t msg_id     = 0;

	// on peux mettre jusqu'a 28 filtres (de 0 à 27)
	if (can_filter_id > 27)
	{
		// TODO code erreur led
		return;
	}
	// Setup identifier information
	if (format == CAN_STANDARD_FORMAT)
	{
		msg_id  |= (uint32_t)(id << 21);
	}
	else
	{
		msg_id  |= (uint32_t)(id <<  3) | 0x04;
	}

	// mode initialisation des filtres
	CAN1->FMR  |=  CAN_FMR_FINIT;
	// desactivation du filtre can_filter_id
	CAN1->FA1R &=  ~(uint32_t)(1 << can_filter_id);

	// init du filtre can_filter_id (32 bits scale conf + deux registres 32 bit id list mode)
	CAN1->FS1R |= (uint32_t)(1 << can_filter_id);
	CAN1->FM1R |= (uint32_t)(1 << can_filter_id);

	CAN1->sFilterRegister[can_filter_id].FR1 = msg_id;
	CAN1->sFilterRegister[can_filter_id].FR2 = msg_id;

	// filtre => FIFO 0 puis activation du filtre
	CAN1->FFA1R &= ~(uint32_t)(1 << can_filter_id);
	CAN1->FA1R  |=  (uint32_t)(1 << can_filter_id);

	// sortie du mode init des filtres
	CAN1->FMR &= ~CAN_FMR_FINIT;

	can_filter_id ++;
}
