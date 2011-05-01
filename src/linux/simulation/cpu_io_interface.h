#ifndef CPU_IO_INTERFACE_H
#define CPU_IO_INTERFACE_H

//! @file cpu_io_interface.h
//! @brief 
//! @author Jean-Baptiste Trédez

#include "log.h"
#include <stdint.h>
#include "cpu/cpu.h"

class CpuIoInterface
{
public:
	CpuIoInterface(){};
	virtual ~CpuIoInterface(){};

	virtual void memory_write(uint64_t offset, uint32_t val) = 0;
	virtual uint32_t memory_read(uint64_t offset) = 0;
};

#endif
