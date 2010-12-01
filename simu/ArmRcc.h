#ifndef ARM_RCC_H
#define ARM_RCC_H

//! @file ArmRcc.h
//! @brief RCC
//! @author Jean-Baptiste Trédez

#include <stdint.h>

#include <stdint.h>
#define STM32F10X_CL
#include "stm32f10x.h"

#include "Motor.h"

// TODO : voir / envoyer à qemu le system_clock_scale

class ArmRcc
{
public:
	ArmRcc();
	~ArmRcc();

	void mem_write(uint64_t offset, uint32_t val);
	uint32_t mem_read(uint64_t offset);

private:
	int system_clock_scale;
	void update();
	RCC_TypeDef RCC;
};

#endif
