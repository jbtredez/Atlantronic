#include "ArmUsart.h"

ArmUsart::ArmUsart(CpuEmu* Cpu, uint32_t It) :
	cpu(Cpu),
	it(It)
{
	MEM.SR |= USART_SR_TXE;
}

ArmUsart::~ArmUsart()
{

}

uint32_t ArmUsart::mem_read(uint64_t offset)
{
	if( offsetof(typeof(MEM), DR) == offset )
	{
		MEM.SR &= ~USART_SR_RXNE;
	}

	// reception activée
	if( MEM.CR1 & USART_CR1_RE)
	{
		for(unsigned int i=0; i < devices.size(); i++)
		{
			if (devices[i]->usart_write_request())
			{
				MEM.SR |= USART_SR_RXNE;
				MEM.DR = devices[i]->usart_write();
			}

			if( (MEM.CR1 & USART_CR1_RXNEIE) && (MEM.SR & USART_SR_RXNE) )
			{
				cpu->set_it(it);
			}
		}
	}

	return ArmMem<USART_TypeDef>::mem_read(offset);
}

void ArmUsart::mem_write(uint64_t offset, uint32_t val)
{
	ArmMem<USART_TypeDef>::mem_write(offset, val);
	// usart actif
	if( MEM.CR1 & USART_CR1_UE)
	{
		// transmission activée
		if( MEM.CR1 & USART_CR1_TE)
		{
			if( offsetof(typeof(MEM), DR) == offset)
			{
				for(unsigned int i=0; i < devices.size(); i++)
				{
					devices[i]->usart_read(MEM.DR & 0xFF);
					if(devices[i]->usart_write_request())
					{
						if( MEM.CR1 & USART_CR1_RE )
						{
							MEM.SR |= USART_SR_RXNE;
							MEM.DR = devices[i]->usart_write();
						}
					}
				}
				MEM.SR |= USART_SR_TXE;
			}

			if( ((MEM.CR1 & USART_CR1_TXEIE) && (MEM.SR & USART_SR_TXE)) || ( (MEM.CR1 & USART_CR1_RE) && (MEM.CR1 & USART_CR1_RXNEIE) && (MEM.SR & USART_SR_RXNE)) )
			{
				cpu->set_it(it);
			}
		}
	}
}

void ArmUsart::connect(UsartDevice* dev)
{
	devices.push_back(dev);
}
