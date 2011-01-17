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
	uint32_t mem_read(uint64_t offset);
	void mem_write(uint64_t offset, uint32_t val);

protected:
	CpuEmu* cpu;
	uint32_t it;
	std::vector<UsartDevice*> devices;
};

#endif
