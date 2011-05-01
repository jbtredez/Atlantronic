#ifndef ADC_H
#define ADC_H

//! @file adc.h
//! @brief Module Adc
//! @author Jean-Baptiste Trédez

#include "log.h"
#include <stdint.h>
#include "cpu/cpu.h"
#include "cpu_io_interface.h"

class Adc : public ADC_TypeDef, public CpuIoInterface
{
public:
	Adc();
	~Adc();

	void memory_write(uint64_t offset, uint32_t val);
	uint32_t memory_read(uint64_t offset);
};

#endif
