#ifndef USART_H
#define USART_H

//! @file usart.h
//! @brief Module Usart
//! @author Atlantronic

#include "log.h"
#include <stdint.h>
#include "cpu_io_interface.h"
#include "CpuEmu.h"
#include "UsartDevice.h"
#include <vector>

class Usart : public USART_TypeDef, public CpuIoInterface
{
public:
	Usart(CpuEmu* cpu, uint32_t it);
	~Usart();

	void connect(UsartDevice* dev);
	void memory_write(uint64_t offset, uint32_t val);
	uint32_t memory_read(uint64_t offset);

protected:
	CpuEmu* cpu;
	uint32_t it;
	std::vector<UsartDevice*> devices;
};

#endif
