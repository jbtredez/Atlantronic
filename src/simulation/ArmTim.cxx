#include "ArmTim.h"

ArmTim::ArmTim()
{

}

ArmTim::~ArmTim()
{

}

int32_t ArmTim::getPwm(int num)
{
	int32_t val = 0;

	if(	MEM.CR1 & TIM_CR1_CEN )
	{
		if(num == 0)
		{
			val = MEM.CCR1;
		}
		else if(num == 1)
		{
			val = MEM.CCR2;
		}
		else if(num == 2)
		{
			val = MEM.CCR3;
		}
		else if(num == 3)
		{
			val = MEM.CCR4;
		}
		else
		{
			// TODO : erreur
		}
	}

	return val;
}

void ArmTim::setEncoder(uint16_t val)
{
// FIXME: temporaire. Ce n'est pas correct.
	MEM.CCR1 = val;
}
