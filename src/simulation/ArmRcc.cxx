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

void ArmRcc::update(uint64_t offset)
{
	if(MEM.BDCR & RCC_BDCR_LSEON)
	{
		meslog(_erreur_, "pas de LSE");
	}

	if(MEM.CR & RCC_CR_HSEON)
	{
		// horloge déjà prête
		MEM.CR |= RCC_CR_HSERDY;
	}

	if(MEM.CR & RCC_CR_PLLON)
	{
		// PLL prête
		MEM.CR |= RCC_CR_PLLRDY;

		if(MEM.CFGR & RCC_CFGR_SW_PLL)
		{
			// PLL => SYSCLK
			MEM.CFGR &= ~RCC_CFGR_SWS;
			MEM.CFGR |= RCC_CFGR_SWS_1;
		}
	}

	if( ( MEM.CR & RCC_CR_HSEON ) && (MEM.CR & RCC_CR_PLLON ) && ( MEM.CFGR & RCC_CFGR_SW_PLL) )
	{
		// PLL, on gère juste la multiplication (pas besoin de la division dans notre cas)
		// TODO gérer les différents cas (PLL1 + PLL2 et diviseurs)
		float mult = 2;
		if(MEM.CFGR & RCC_CFGR_PLLMULL4) mult = 4;
		if(MEM.CFGR & RCC_CFGR_PLLMULL5) mult = 5;
		if(MEM.CFGR & RCC_CFGR_PLLMULL6) mult = 6;
		if(MEM.CFGR & RCC_CFGR_PLLMULL6_5) mult = 6.5;
		if(MEM.CFGR & RCC_CFGR_PLLMULL7) mult = 7;
		if(MEM.CFGR & RCC_CFGR_PLLMULL8) mult = 8;
		if(MEM.CFGR & RCC_CFGR_PLLMULL9) mult = 9;

		float freq = 8000000 * mult; // en Hz
		meslog(_info_, "rcc conf => %f Mhz", freq/1000000.0f);
	}
}
