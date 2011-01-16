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

	if(MEM.CR & RCC_CR_PLL2ON)
	{
		// PLL2 prête
		MEM.CR |= RCC_CR_PLL2RDY;
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
}
