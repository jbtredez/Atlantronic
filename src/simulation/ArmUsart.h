#ifndef ARM_USART_H
#define ARM_USART_H

//! @file ArmUsart.h
//! @brief USART
//! @author Jean-Baptiste Trédez

#include "ArmMem.h"
#include "CpuEmu.h"

class ArmUsart : public ArmMem<USART_TypeDef>
{
public:
	ArmUsart(CpuEmu* cpu, uint32_t it);
	~ArmUsart();

protected:
	void update(uint64_t offset);
	CpuEmu* cpu;
	uint32_t it;
};

#endif
