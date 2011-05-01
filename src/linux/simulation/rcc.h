#ifndef RCC_H
#define RCC_H

//! @file rcc.h
//! @brief Module RCC
//! @author Jean-Baptiste Trédez

#include "log.h"
#include <stdint.h>
#include "cpu_io_interface.h"

class Rcc : public RCC_TypeDef, public CpuIoInterface
{
public:
	Rcc();
	~Rcc();

	void memory_write(uint64_t offset, uint32_t val);
	uint32_t memory_read(uint64_t offset);

protected:
	uint64_t hsi;
	uint64_t hse;
	uint64_t pll2clk;
	uint64_t pllclk;
	uint64_t sysclk;
	uint64_t hclk;
	uint64_t pclk;
	uint64_t pclk2;

	void write_CR(uint32_t val);
};

#endif
