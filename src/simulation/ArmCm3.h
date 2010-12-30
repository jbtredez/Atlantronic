#ifndef ARM_CM3_H
#define ARM_CM3_H

#include <stdint.h>
#include "CpuEmu.h"
#include "ArmTim.h"
#include "ArmRcc.h"
#include "ArmGpio.h"
#include "Model.h"

class ArmCm3 : public CpuEmu
{
public:
	ArmCm3(Model *m);
	~ArmCm3();

	void mem_write(uint64_t offset, uint32_t val);
	uint32_t mem_read(uint64_t offset);

	void update_hardware(uint64_t vm_clk);

	pthread_mutex_t io;
	ArmRcc RCC;
	ArmTim TIM1;
	ArmTim TIM3;
	ArmTim TIM4;
	ArmGpio GPIOA;
	ArmGpio GPIOB;
	ArmGpio GPIOC;
	ArmGpio GPIOD;
	ArmGpio GPIOE;
	ArmGpio GPIOF;
	ArmGpio GPIOG;
	Model *model;
};

#endif
