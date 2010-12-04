#ifndef ARM_CM3_H
#define ARM_CM3_H

#include <stdint.h>
#include "CpuEmu.h"
#include "ArmTimMotor.h"
#include "ArmRcc.h"

class ArmCm3 : public CpuEmu
{
public:
	ArmCm3();
	~ArmCm3();

	void mem_write(uint64_t offset, uint32_t val);
	uint32_t mem_read(uint64_t offset);

private:
	ArmRcc RCC;
	ArmTimMotor TIM1;
};

#endif
