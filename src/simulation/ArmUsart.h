#ifndef ARM_USART_H
#define ARM_USART_H

//! @file ArmUsart.h
//! @brief USART
//! @author Jean-Baptiste Trédez

#include "ArmMem.h"
#include "CpuEmu.h"
#include "UsartDevice.h"
#include <vector>

class ArmUsart : public ArmMem<USART_TypeDef>
{
public:
	ArmUsart(CpuEmu* cpu, uint32_t it);
	~ArmUsart();

	void connect(UsartDevice* dev);

protected:
	void update(uint64_t offset);
	CpuEmu* cpu;
	uint32_t it;
	std::vector<UsartDevice*> devices;
};

#endif
