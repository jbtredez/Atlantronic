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


void ArmUsart::update(uint64_t offset)
{
	// usart actif
	if( MEM.CR1 & USART_CR1_UE)
	{
		// ligne TE active
		if( MEM.CR1 & USART_CR1_TE)
		{
			if( offsetof(typeof(MEM), SR) == offset)
			{
				return;
			}
			if( offsetof(typeof(MEM), DR) == offset)
			{
				for(unsigned int i=0; i < devices.size(); i++)
				{
					devices[i]->usart_read(MEM.DR & 0xFF);
				}
				MEM.SR |= USART_SR_TXE;
			}

			if( (MEM.CR1 & USART_CR1_TXEIE) && (MEM.SR & USART_SR_TXE))
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
