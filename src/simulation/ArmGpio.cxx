#include "ArmGpio.h"

ArmGpio::ArmGpio()
{

}

ArmGpio::~ArmGpio()
{

}


void ArmGpio::update()
{
	// TODO : pas de vérif de la config
}

void ArmGpio::setInput(uint32_t val)
{
	MEM.IDR = val;
}

uint32_t ArmGpio::getInput()
{
	return MEM.IDR;
}

void ArmGpio::setOutput(uint32_t val)
{
	MEM.ODR = val;
}

uint32_t ArmGpio::getOutput()
{
	return MEM.ODR;
}

void ArmGpio::setInput(uint32_t val, uint32_t pin)
{
	if( val)
	{
		MEM.IDR |= pin;
	}
	else
	{
		MEM.IDR &= ~pin;
	}
}

uint32_t ArmGpio::getInput(uint32_t pin)
{
	return MEM.IDR & pin;
}

void ArmGpio::setOutput(uint32_t val, uint32_t pin)
{
	if(val)
	{
		MEM.ODR |= pin;
	}
	else
	{
		MEM.ODR &= ~pin;
	}
}

uint32_t ArmGpio::getOutput(uint32_t pin)
{
	return MEM.ODR & pin;
}
