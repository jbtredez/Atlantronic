#include "usart.h"
#include "log.h"
#include <math.h>
#include <stdio.h>

Usart::Usart(CpuEmu* Cpu, uint32_t It) :
	cpu(Cpu),
	it(It)
{
	SR   = 0xC0;
	DR   = 0x00;
	BRR  = 0x00;
	CR1  = 0x00;
	CR2  = 0x00;
	CR3  = 0x00;
	GTPR = 0x00;
	// TODO
	SR |= USART_SR_TXE;
}

Usart::~Usart()
{

}

void Usart::memory_write(uint64_t offset, uint32_t val)
{
	switch(offset)
	{
		case offsetof(USART_TypeDef, SR):
			SR &= val & 0x3FF; // TODO
			break;
		case offsetof(USART_TypeDef, DR):
			DR = val & 0x3FF;
			if( (CR1 & USART_CR1_UE) && (CR1 & USART_CR1_TE))
			{
				for(unsigned int i=0; i < devices.size(); i++)
				{
					devices[i]->usart_read(DR & 0xFF);
					if(devices[i]->usart_write_request())
					{
						if( CR1 & USART_CR1_RE )
						{
							SR |= USART_SR_RXNE;
						}
					}
				}
			}
			SR |= USART_SR_TXE;
			break;
		case offsetof(USART_TypeDef, BRR):
			BRR = val & 0xFFFF;
			break;
		case offsetof(USART_TypeDef, CR1):
			CR1 = val & 0x3FFF;
			break;
		case offsetof(USART_TypeDef, CR2):
			CR2 = val & 0x7FFF;
			break;
		case offsetof(USART_TypeDef, CR3):
			CR3 = val & 0x7FF;
			break;
		case offsetof(USART_TypeDef, GTPR):
			GTPR = val & 0xFFFF;
			break;
		default:
			meslog(_erreur_, "ecriture non supportée offset %#"PRIx64", val %#"PRIx32, offset, val);
			break;
	}

	// usart actif et transmission activée
	if( (CR1 & USART_CR1_UE) && (CR1 & USART_CR1_TE))
	{
		if( ((CR1 & USART_CR1_TXEIE) && (SR & USART_SR_TXE)) || ( (CR1 & USART_CR1_RE) && (CR1 & USART_CR1_RXNEIE) && (SR & USART_SR_RXNE)) )
		{
			cpu->set_it(it);
		}
	}
}

uint32_t Usart::memory_read(uint64_t offset)
{
	uint32_t rep = 0;

	switch(offset)
	{
		case offsetof(USART_TypeDef, SR):
			rep = SR;
			break;
		case offsetof(USART_TypeDef, DR):
			SR &= ~USART_SR_RXNE;
			// usart activé et reception activée
			if( (CR1 & USART_CR1_UE) && (CR1 & USART_CR1_RE) )
			{
				for(unsigned int i=0; i < devices.size(); i++)
				{
					if (devices[i]->usart_write_request())
					{
						DR = devices[i]->usart_write();
					}
					if (devices[i]->usart_write_request())
					{
						SR |= USART_SR_RXNE;
					}
				}
				if( (CR1 & USART_CR1_RXNEIE) && (SR & USART_SR_RXNE) )
				{
					cpu->set_it(it);
				}
			}
			rep = DR;
			break;
		case offsetof(USART_TypeDef, BRR):
			rep = BRR;
			break;
		case offsetof(USART_TypeDef, CR1):
			rep = CR1;
			break;
		case offsetof(USART_TypeDef, CR2):
			rep = CR2;
			break;
		case offsetof(USART_TypeDef, CR3):
			rep = CR3;
			break;
		case offsetof(USART_TypeDef, GTPR):
			rep = GTPR;
			break;
		default:
			meslog(_erreur_, "lecture non supportée offset %#"PRIx64, offset);
			break;
	}

	return rep;
}

void Usart::connect(UsartDevice* dev)
{
	devices.push_back(dev);
}
