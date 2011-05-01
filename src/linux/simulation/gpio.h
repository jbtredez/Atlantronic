#ifndef GPIO_H
#define GPIO_H

//! @file gpio.h
//! @brief Module Gpio
//! @author Jean-Baptiste Trédez

#include "log.h"
#include <stdint.h>
#include "cpu/cpu.h"
#include "cpu_io_interface.h"

class Gpio : public GPIO_TypeDef, public CpuIoInterface
{
public:
	Gpio();
	~Gpio();

	void memory_write(uint64_t offset, uint32_t val);
	uint32_t memory_read(uint64_t offset);

	void setInput(uint32_t val);
	void setInput(uint32_t val, uint32_t pin); //!< pour pin, utiliser les constantes GPIO_IDR_IDR0, ... GPIO_IDR_IDR15

	uint32_t getInput();
	uint32_t getInput(uint32_t pin); //!< 0 si la patte est à 0, pin sinon. Pour pin, utiliser les constantes GPIO_IDR_IDR0, ... GPIO_IDR_IDR15

	void setOutput(uint32_t val);
	void setOutput(uint32_t val, uint32_t pin);//!< pour pin, utiliser les constantes GPIO_ODR_ODR0, ... GPIO_ODR_ODR15

	uint32_t getOutput();
	uint32_t getOutput(uint32_t pin); //!< 0 si la patte est à 0, pin sinon. Pour pin, utiliser les constantes GPIO_ODR_ODR0, ... GPIO_ODR_ODR15
};

#endif
