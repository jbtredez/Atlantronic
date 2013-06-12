//! @file can.c
//! @brief CAN
//! @author Atlantronic

#include "kernel/event.h"
#include "kernel/driver/can.h"
#include "kernel/semphr.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
//#include "kernel/fault.h"
#include "kernel/driver/usb.h"
#include "kernel/error_codes.h"

static void can_write_mailbox(struct can_msg *msg);
static void can_cmd_write(void* arg);
static void can_cmd_set_baudrate(void* arg);
static void can_set_baudrate(enum can_baudrate speed, int debug);
static int can_set_mask(int id, uint32_t mask);
static void can_write_task(void *arg);
static xQueueHandle can_write_queue;
static xQueueHandle can_read_queue;
static xSemaphoreHandle can_write_sem;
#define CAN_WRITE_STACK_SIZE     80
#define CAN_WRITE_QUEUE_SIZE     80

int can_open(enum can_baudrate baudrate, xQueueHandle _can_read_queue)
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

	can_set_baudrate(baudrate, 0);

	CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_TMEIE;
	NVIC_SetPriority(CAN1_TX_IRQn, PRIORITY_IRQ_CAN1_TX);
	NVIC_SetPriority(CAN1_RX0_IRQn, PRIORITY_IRQ_CAN1_RX0);
	NVIC_EnableIRQ(CAN1_TX_IRQn);
	NVIC_EnableIRQ(CAN1_RX0_IRQn);

	// lancement du CAN
	CAN1->MCR &= ~CAN_MCR_INRQ;
	while (CAN1->MSR & CAN_MCR_INRQ) ;

	can_read_queue = _can_read_queue;

	can_write_queue = xQueueCreate(CAN_WRITE_QUEUE_SIZE, sizeof(struct can_msg));

	if(can_write_queue == 0)
	{
		return ERR_INIT_CAN;
	}

	vSemaphoreCreateBinary(can_write_sem);
	if( can_write_sem == 0 )
	{
		return ERR_INIT_CAN;
	}
	xSemaphoreTake(can_write_sem, 0);

	xTaskHandle xHandle;
	int err = xTaskCreate(can_write_task, "can_write", CAN_WRITE_STACK_SIZE, NULL, PRIORITY_TASK_CAN, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CAN;
	}

	can_set_mask(0, 0x00);

	// commandes can
	usb_add_cmd(USB_CMD_CAN_SET_BAUDRATE, &can_cmd_set_baudrate);
	usb_add_cmd(USB_CMD_CAN_WRITE, &can_cmd_write);

	return 0;
}

static int can_set_mask(int id, uint32_t mask)
{
	if( id > 27 )
	{
		return -1;
	}

	// mode initialisation des filtres
	CAN1->FMR  |=  CAN_FMR_FINIT;

	// desactivation du filtre can_filter_id
	CAN1->FA1R &=  ~(uint32_t)(1 << id);

	// init du filtre can_filter_id (32 bits scale conf + deux registres 32 bit id list mode)
	CAN1->FS1R |= (uint32_t)(1 << id);
	CAN1->FM1R &= ~(uint32_t)(1 << id);

	CAN1->sFilterRegister[id].FR1 = mask;
	CAN1->sFilterRegister[id].FR2 = mask;

	// filtre => FIFO 0 puis activation du filtre
	CAN1->FFA1R &= ~(uint32_t)(1 << id);
	CAN1->FA1R  |=  (uint32_t)(1 << id);

	// sortie du mode init des filtres
	CAN1->FMR &= ~CAN_FMR_FINIT;

	return 0;
}

static void can_set_baudrate(enum can_baudrate speed, int debug)
{
	#if( RCC_PCLK1 != 36000000)
	#error "remettre RCC_PCLK1 à 36Mhz, sinon c'est le bordel pour recalculer BTR"
	#endif

	// init mode
	CAN1->MCR = CAN_MCR_INRQ;

	// TBS1 = 12 TQ
	// TBS2 =  5 TQ
	// SJW  =  4 TQ
	// total bit can (1 + TBS1 + TBS2) = 18 TQ
	// SP = (1 + TBS1)/total = 72,22 %
	// vitesse : 1000kb => 1000000*18*TQ = PCLK = 36Mhz
	// => TQ = PCLK / (18 * 1000000) = 2
	CAN1->BTR &= ~ (              CAN_BTR_SJW  |                  CAN_BTR_TS2  |                   CAN_BTR_TS1  |   CAN_BTR_BRP    );

	switch(speed)
	{
		case CAN_1000:
			CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((2-1) & CAN_BTR_BRP);
			break;
		case CAN_800:
			CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((3-1) & CAN_BTR_BRP);
			break;
		case CAN_500:
			CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((4-1) & CAN_BTR_BRP);
			break;
		case CAN_250:
			CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((8-1) & CAN_BTR_BRP);
			break;
		case CAN_125:
			CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((16-1) & CAN_BTR_BRP);
			break;
		case CAN_50:
			CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((20-1) & CAN_BTR_BRP);
			break;
		case CAN_20:
			CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((50-1) & CAN_BTR_BRP);
			break;
		case CAN_10:
			CAN1->BTR |= (((4-1) << 24) & CAN_BTR_SJW) | (((5-1) << 20) & CAN_BTR_TS2) | (((12-1) << 16) & CAN_BTR_TS1) | ((100-1) & CAN_BTR_BRP);
			break;
		case CAN_RESERVED:
		default:
			break;
	}

	if(debug)
	{
		// mode self-test pour le debug
		CAN1->BTR |= CAN_BTR_SILM | CAN_BTR_LBKM;
	}
	else
	{
		CAN1->BTR &= ~(CAN_BTR_SILM | CAN_BTR_LBKM);
	}

	// lancement du CAN
	CAN1->MCR &= ~CAN_MCR_INRQ;
	while (CAN1->MSR & CAN_MCR_INRQ) ;
}

static void can_write_task(void *arg)
{
	(void) arg;

	struct can_msg req;

	while(1)
	{
		if(xQueueReceive(can_write_queue, &req, portMAX_DELAY))
		{
			can_write_mailbox(&req);
			xSemaphoreTake(can_write_sem, portMAX_DELAY);
// TODO : check error
		}
	}
}

void isr_can1_tx(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	// fin de transmission sur la boite 0
	if(CAN1->TSR & CAN_TSR_RQCP0)
	{
		// transmission ok
		if( CAN1->TSR & CAN_TSR_TXOK0)
		{
			CAN1->IER &= ~CAN_IER_TMEIE;
			xSemaphoreGiveFromISR(can_write_sem, &xHigherPriorityTaskWoken);
		}
		else
		{
			// envoi ko TODO err
			nop();
		}
		CAN1->TSR |= CAN_TSR_RQCP0;
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_can1_rx0(void)
{
	struct can_msg msg;
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( CAN1->RF0R & CAN_RF0R_FOVR0)
	{
		//fault_from_isr(ERR_CAN_READ_FIFO_OVERFLOW, FAULT_ACTIVE);
	}

	// reception sur la FIFO 0
	if(CAN1->RF0R & CAN_RF0R_FMP0)
	{
		if((CAN1->sFIFOMailBox[0].RIR & (uint32_t)0x00000004) == 0)
		{
			// ID standard
			msg.format = CAN_STANDARD_FORMAT;
			msg.id = (uint32_t)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
		}
		else
		{
			// ID étendu
			msg.format = CAN_EXTENDED_FORMAT;
			msg.id = (uint32_t)0x0003FFFF & (CAN1->sFIFOMailBox[0].RIR >> 3);
		}

		if ((CAN1->sFIFOMailBox[0].RIR & (uint32_t)0x00000002) == 0)
		{
			msg.type = CAN_DATA_FRAME;
		}
		else
		{
			msg.type = CAN_REMOTE_FRAME;
		}

		msg.size = (uint8_t)0x0000000F & CAN1->sFIFOMailBox[0].RDTR;
		msg._data.low = CAN1->sFIFOMailBox[0].RDLR;
		msg._data.high = CAN1->sFIFOMailBox[0].RDHR;
		msg.time = systick_get_time_from_isr();

		// attention, la fonction desactive les IT puis les reactive
		if( xQueueSendToBackFromISR(can_read_queue, &msg, &xHigherPriorityTaskWoken) != pdPASS)
		{
			// erreur, file pleine : message perdu
			//fault_from_isr(ERR_CAN_READ_QUEUE_FULL, FAULT_ACTIVE);
		}

		CAN1->RF0R |= CAN_RF0R_RFOM0;
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void can_write_mailbox(struct can_msg *msg)
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

	CAN1->sTxMailBox[0].TDLR = msg->_data.low;
	CAN1->sTxMailBox[0].TDHR = msg->_data.high;

	CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
	CAN1->sTxMailBox[0].TDTR |= msg->size & CAN_TDT0R_DLC;

	CAN1->IER |= CAN_IER_TMEIE;
	CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
}

static void can_cmd_write(void* arg)
{
	struct can_msg* msg = (struct can_msg*) arg;
	can_write(msg, 0);
}

static void can_cmd_set_baudrate(void* arg)
{
	char* buffer = (char*) arg;
	can_set_baudrate(buffer[0], buffer[1]);
}

uint32_t can_write(struct can_msg *msg, portTickType timeout)
{
	uint32_t res = 0;

	if( xQueueSendToBack(can_write_queue, msg, timeout) != pdPASS)
	{
		res = -1;
	}

	return res;
}
