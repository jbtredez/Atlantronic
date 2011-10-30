#include "gpio.h"
#include "log.h"
#include <math.h>
#include <stdio.h>

Gpio::Gpio()
{
	CRL  = 0x44444444;
	CRH  = 0x44444444;
	IDR  = 0x00;
	ODR  = 0x00;
	BSRR = 0x00;
	BRR  = 0x00;
	LCKR = 0x00;
}

Gpio::~Gpio()
{

}

void Gpio::memory_write(uint64_t offset, uint32_t val)
{
	switch(offset)
	{
		case offsetof(GPIO_TypeDef, CRL):
			CRL = val;
			break;
		case offsetof(GPIO_TypeDef, CRH):
			CRH = val;
			break;
		case offsetof(GPIO_TypeDef, IDR):
			meslog(_erreur_, "GPIO - IDR : en lecture seule, impossible d'écrire %#"PRIx32, val);
			break;
		case offsetof(GPIO_TypeDef, ODR):
			ODR = val & 0xFFFF;
			break;
		case offsetof(GPIO_TypeDef, BSRR):
			BSRR = val;
			break;
		case offsetof(GPIO_TypeDef, BRR):
			BRR = val & 0xFFFF;
			break;
		case offsetof(GPIO_TypeDef, LCKR):
			LCKR = val & 0x1FFFF;
			break;
		default:
			meslog(_erreur_, "ecriture non supportée offset %#"PRIx64", val %#"PRIx32, offset, val);
			break;
	}
}

uint32_t Gpio::memory_read(uint64_t offset)
{
	uint32_t rep = 0;

	switch(offset)
	{
		case offsetof(GPIO_TypeDef, CRL):
			rep = CRL;
			break;
		case offsetof(GPIO_TypeDef, CRH):
			rep = CRH;
			break;
		case offsetof(GPIO_TypeDef, IDR):
			rep = IDR;
			break;
		case offsetof(GPIO_TypeDef, ODR):
			rep = ODR;
			break;
		case offsetof(GPIO_TypeDef, BSRR):
			meslog(_erreur_, "GPIO - BSRR : en ecriture seule, impossible de lire une valeur correcte");
			break;
		case offsetof(GPIO_TypeDef, BRR):
			meslog(_erreur_, "GPIO - BRR : en ecriture seule, impossible de lire une valeur correcte");
			break;
		case offsetof(GPIO_TypeDef, LCKR):
			rep = LCKR;
			break;
		default:
			meslog(_erreur_, "lecture non supportée offset %#"PRIx64, offset);
			break;
	}

	return rep;
}

void Gpio::setInput(uint32_t val)
{
	IDR = val;
}

uint32_t Gpio::getInput()
{
	return IDR;
}

void Gpio::setOutput(uint32_t val)
{
	ODR = val;
}

uint32_t Gpio::getOutput()
{
	return ODR;
}

void Gpio::setInput(uint32_t val, uint32_t pin)
{
	if( val)
	{
		IDR |= pin;
	}
	else
	{
		IDR &= ~pin;
	}
}

uint32_t Gpio::getInput(uint32_t pin)
{
	return IDR & pin;
}

void Gpio::setOutput(uint32_t val, uint32_t pin)
{
	if(val)
	{
		ODR |= pin;
	}
	else
	{
		ODR &= ~pin;
	}
}

uint32_t Gpio::getOutput(uint32_t pin)
{
	return ODR & pin;
}
