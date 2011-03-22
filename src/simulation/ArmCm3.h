#ifndef ARM_CM3_H
#define ARM_CM3_H

#include <stdint.h>
#include "CpuEmu.h"
#include "ArmTim.h"
#include "ArmGpio.h"
#include "ArmUsart.h"
#include "Model.h"
#include "rcc.h"

class ArmCm3 : public CpuEmu
{
public:
	ArmCm3(Model *m);
	~ArmCm3();

	void update_hardware(uint64_t vm_clk);

	pthread_mutex_t io;
	Rcc rcc;
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
	ArmUsart USART3;
	Model *model;
};

#endif
