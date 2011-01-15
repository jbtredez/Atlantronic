#ifndef ARM_GPIO_H
#define ARM_GPIO_H

//! @file ArmGpio.h
//! @brief GPIO
//! @author Jean-Baptiste Trédez

#include "ArmMem.h"

class ArmGpio : public ArmMem<GPIO_TypeDef>
{
public:
	ArmGpio();
	~ArmGpio();

	void update(uint64_t offset);

	void setInput(uint32_t val);
	void setInput(uint32_t val, uint32_t pin); //!< pour pin, utiliser les constantes GPIO_IDR_IDR0, ... GPIO_IDR_IDR15

	uint32_t getInput();
	uint32_t getInput(uint32_t pin); //!< 0 si la patte est à 0, pin sinon. Pour pin, utiliser les constantes GPIO_IDR_IDR0, ... GPIO_IDR_IDR15

	void setOutput(uint32_t val);
	void setOutput(uint32_t val, uint32_t pin);//!< pour pin, utiliser les constantes GPIO_IDR_ODR0, ... GPIO_IDR_ODR15

	uint32_t getOutput();
	uint32_t getOutput(uint32_t pin); //!< 0 si la patte est à 0, pin sinon. Pour pin, utiliser les constantes GPIO_IDR_ODR0, ... GPIO_IDR_ODR15
};

#endif
