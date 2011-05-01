#include "rcc.h"
#include "log.h"
#include <math.h>
#include <stdio.h>

Rcc::Rcc()
{
	// configuration de la carte :
	// hsi = 8MHz
	// hse = 25 Mhz
	hsi = 8000000;
	hse = 25000000;

	CR = 0x00000083;
	CFGR = 0x00;
	CIR = 0x00;
	APB2RSTR = 0x00;
	APB1RSTR = 0x00;
	AHBENR = 0x14;
	APB2ENR = 0x00;
	APB1ENR = 0x00;
	BDCR = 0x00;
	CSR = 0x0C000000;
	AHBRSTR = 0x00;
	CFGR2 = 0x00;

	// TODO
	pll2clk = 0;
	pllclk = 0;
	sysclk = 0;
	hclk = 0;
	pclk = 0;
	pclk2 = 0;
}

Rcc::~Rcc()
{

}

void Rcc::write_CR(uint32_t val)
{
	uint64_t write_mask = 0x1509FF01;
	if(! (CR & RCC_CR_HSION))
	{
		write_mask |= RCC_CR_HSEBYP;
	}

	CR = (CR & ~write_mask) | (val & write_mask);

	if(CR & RCC_CR_HSION)
	{
		// horloge déjà prête
		CR |= RCC_CR_HSIRDY;
	}
	else
	{
		CR &= ~RCC_CR_HSIRDY;
	}

	if(CR & RCC_CR_HSEON)
	{
		// horloge déjà prête
		CR |= RCC_CR_HSERDY;
	}
	else
	{
		CR &= ~RCC_CR_HSERDY;
	}

	if(CR & RCC_CR_PLLON)
	{
		// PLL prête
		CR |= RCC_CR_PLLRDY;
	}
	else
	{
		CR &= ~RCC_CR_PLLRDY;
	}

	if(CR & RCC_CR_PLL2ON)
	{
		// PLL2 prête
		CR |= RCC_CR_PLL2RDY;
	}
	else
	{
		CR &= ~RCC_CR_PLL2RDY;
	}

	if(CR & RCC_CR_PLL3ON)
	{
		// PLL3 prête
		CR |= RCC_CR_PLL3RDY;
	}
	else
	{
		CR &= ~RCC_CR_PLL3RDY;
	}
}

void Rcc::memory_write(uint64_t offset, uint32_t val)
{
	switch(offset)
	{
		case offsetof(RCC_TypeDef, CR):
			write_CR(val);
			break;
		case offsetof(RCC_TypeDef, CFGR):
			CFGR = val; // TODO
			if(val & RCC_CFGR_SW_PLL)
			{
				// PLL utilisée pour SYSCLK
				CFGR &= ~RCC_CFGR_SWS;
				CFGR |= RCC_CFGR_SWS_1;
			}
			break;
		case offsetof(RCC_TypeDef, CIR):
			CIR = val; // TODO
			break;
		case offsetof(RCC_TypeDef, APB2RSTR):
			APB2RSTR = val; // TODO
			break;
		case offsetof(RCC_TypeDef, APB1RSTR):
			APB1RSTR = val; // TODO
			break;
		case offsetof(RCC_TypeDef, AHBENR):
			AHBENR = val; // TODO
			break;
		case offsetof(RCC_TypeDef, APB1ENR):
			APB1ENR = val; // TODO
			break;
		case offsetof(RCC_TypeDef, APB2ENR):
			APB2ENR = val; // TODO
			break;
		case offsetof(RCC_TypeDef, BDCR):
			BDCR = val; // TODO
			if(val & RCC_BDCR_LSEON)
			{
				meslog(_erreur_, "pas de LSE");
			}
			break;
		case offsetof(RCC_TypeDef, CSR):
			CSR = val; // TODO
			break;
		case offsetof(RCC_TypeDef, AHBRSTR):
			AHBRSTR = val; // TODO
			break;
		case offsetof(RCC_TypeDef, CFGR2):
			CFGR2 = val; // TODO
			break;
		default:
			meslog(_erreur_, "ecriture non supportée offset %#lx, val %#x", offset, val);
			break;
	}
}

uint32_t Rcc::memory_read(uint64_t offset)
{
	uint32_t rep = 0;

	switch(offset)
	{
		case offsetof(RCC_TypeDef, CR):
			rep = CR & (uint32_t)0xFF0FFFFB;
			break;
		case offsetof(RCC_TypeDef, CFGR):
			rep = CFGR; // TODO
			break; 
		case offsetof(RCC_TypeDef, CIR):
			rep = CIR; // TODO
			break;
		case offsetof(RCC_TypeDef, APB2RSTR):
			rep = APB2RSTR; // TODO
			break;
		case offsetof(RCC_TypeDef, APB1RSTR):
			rep = APB1RSTR; // TODO
			break;
		case offsetof(RCC_TypeDef, AHBENR):
			rep = AHBENR; // TODO
			break;
		case offsetof(RCC_TypeDef, APB1ENR):
			rep = APB1ENR; // TODO
			break;
		case offsetof(RCC_TypeDef, APB2ENR):
			rep = APB2ENR; // TODO
			break;
		case offsetof(RCC_TypeDef, BDCR):
			rep = BDCR; // TODO
			break;
		case offsetof(RCC_TypeDef, CSR):
			rep = CSR; // TODO
			break;
		case offsetof(RCC_TypeDef, AHBRSTR):
			rep = AHBRSTR; // TODO
			break;
		case offsetof(RCC_TypeDef, CFGR2):
			rep = CFGR2; // TODO
			break;
		default:
			meslog(_erreur_, "lecture non supportée offset %#lx", offset);
			break;
	}

	return rep;
}
