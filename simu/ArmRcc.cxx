#include "ArmRcc.h"
#include "log.h"
#include <math.h>
#include <stdio.h>

ArmRcc::ArmRcc()
{

}

ArmRcc::~ArmRcc()
{

}

void ArmRcc::mem_write(uint64_t offset, uint32_t val)
{
	if(offset > sizeof(RCC_TypeDef) - sizeof(uint32_t))
	{
		meslog(_erreur_, "RCC - write - non supporté - offset %i\n", offset);
		return;
	}
	else
	{
		*((uint32_t*)(((unsigned char*)(&RCC)) + offset)) = val;
		update();
	}
}

uint32_t ArmRcc::mem_read(uint64_t offset)
{
	uint32_t rep = 0;
	if(offset > sizeof(RCC_TypeDef) - sizeof(uint32_t))
	{
		meslog(_erreur_, "RCC - read - non supporté - offset %i\n", offset);
	}
	else
	{
		rep = *((uint32_t*)(((unsigned char*)(&RCC)) + offset));
	}

	return rep;
}

void ArmRcc::update()
{
	if(RCC.BDCR & RCC_BDCR_LSEON)
	{
		meslog(_erreur_, "pas de LSE");
	}

	if(RCC.CR & RCC_CR_HSEON)
	{
		// horloge déjà prête
		RCC.CR |= RCC_CR_HSERDY;
	}

	if(RCC.CR & RCC_CR_PLLON)
	{
		// PLL prête
		RCC.CR |= RCC_CR_PLLRDY;

		if(RCC.CFGR & RCC_CFGR_SW_PLL)
		{
			// PLL => SYSCLK
			RCC.CFGR &= ~RCC_CFGR_SWS;
			RCC.CFGR |= RCC_CFGR_SWS_1;
		}
	}

	if( ( RCC.CR & RCC_CR_HSEON ) && (RCC.CR & RCC_CR_PLLON ) && ( RCC.CFGR & RCC_CFGR_SW_PLL) )
	{
		// PLL, on gère juste la multiplication (pas besoin de la division dans notre cas)
		// TODO gérer les différents cas (PLL1 + PLL2 et diviseurs)
		float mult = 2;
		if(RCC.CFGR & RCC_CFGR_PLLMULL4) mult = 4;
		if(RCC.CFGR & RCC_CFGR_PLLMULL5) mult = 5;
		if(RCC.CFGR & RCC_CFGR_PLLMULL6) mult = 6;
		if(RCC.CFGR & RCC_CFGR_PLLMULL6_5) mult = 6.5;
		if(RCC.CFGR & RCC_CFGR_PLLMULL7) mult = 7;
		if(RCC.CFGR & RCC_CFGR_PLLMULL8) mult = 8;
		if(RCC.CFGR & RCC_CFGR_PLLMULL9) mult = 9;

		float freq = 8000000 * mult; // en Hz
		system_clock_scale = ceil((1000000000.0f / freq));
		printf("rcc conf => %f Mhz\n", 1000.0f/((float)system_clock_scale));
	}
}
