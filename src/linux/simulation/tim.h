#ifndef TIM_H
#define TIM_H

//! @file tim.h
//! @brief Module Tim
//! @author Jean-Baptiste Trédez

#include "log.h"
#include <stdint.h>
#include "cpu_io_interface.h"

class Tim : public TIM_TypeDef, public CpuIoInterface
{
public:
	Tim();
	~Tim();

	void memory_write(uint64_t offset, uint32_t val);
	uint32_t memory_read(uint64_t offset);

	float getPwm(int num);
	void setEncoder(uint16_t val);
};

#endif
