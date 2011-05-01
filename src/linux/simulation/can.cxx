#include "can.h"
#include "log.h"
#include <math.h>
#include <stdio.h>

Can::Can()
{
	MCR  = 0x00010002;
	MSR  = 0x00000C02;
	TSR  = 0x1C000000;
	RF0R = 0x00;
	RF1R = 0x00;
	IER  = 0x00;
	ESR  = 0x00;
	BTR  = 0x01230000;
	memset(sTxMailBox, 0x00, sizeof(sTxMailBox[3]));
	memset(sFIFOMailBox, 0x00, sizeof(sFIFOMailBox[2]));
	FMR   = 0x2A1C0E01;
	FM1R  = 0x00;
	FS1R  = 0x00;
	FFA1R = 0x00;
	FA1R  = 0x00;
	memset(sFilterRegister, 0x00, sizeof(sFilterRegister[28]));  
}

Can::~Can()
{

}

void Can::memory_write(uint64_t offset, uint32_t val)
{
	switch(offset)
	{
		case offsetof(CAN_TypeDef, MCR):
			if( (MCR & CAN_MCR_INRQ) == 0)
			{
				MSR |= CAN_MCR_INRQ;
			}
			break;
		default:
			meslog(_erreur_, "ecriture non supportée offset %#lx, val %#x", offset, val);
			break;
	}
}

uint32_t Can::memory_read(uint64_t offset)
{
	uint32_t rep = 0;

	switch(offset)
	{
		default:
			meslog(_erreur_, "lecture non supportée offset %#lx", offset);
			break;
	}

	return rep;
}
