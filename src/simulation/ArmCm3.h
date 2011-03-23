#ifndef ARM_CM3_H
#define ARM_CM3_H

#include <stdint.h>
#include "CpuEmu.h"
#include "rcc.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "ArmUsart.h"
#include "Model.h"

class ArmCm3 : public CpuEmu
{
public:
	ArmCm3(Model *m);
	~ArmCm3();

	void update_hardware(uint64_t vm_clk);

	pthread_mutex_t io;
	Rcc rcc;
	Gpio gpioA;
	Gpio gpioB;
	Gpio gpioC;
	Gpio gpioD;
	Gpio gpioE;
	Gpio gpioF;
	Gpio gpioG;
	Tim tim1;
	Tim tim2;
	Tim tim3;
	Tim tim4;
	Tim tim5;
	Tim tim6;
	Tim tim7;
	Adc adc1;
	Adc adc2;
	ArmUsart USART3;
	Model *model;
};

#endif
