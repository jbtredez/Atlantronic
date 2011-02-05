#include "ArmTim.h"

ArmTim::ArmTim()
{

}

ArmTim::~ArmTim()
{

}

float ArmTim::getPwm(int num)
{
	float val = 0;

	if(	MEM.CR1 & TIM_CR1_CEN && MEM.ARR > 0)
	{
		if(num == 0)
		{
			val = ((float) MEM.CCR1) / ((float) MEM.ARR);
		}
		else if(num == 1)
		{
			val = ((float) MEM.CCR2) / ((float) MEM.ARR);
		}
		else if(num == 2)
		{
			val = ((float) MEM.CCR3) / ((float) MEM.ARR);
		}
		else if(num == 3)
		{
			val = ((float) MEM.CCR4) / ((float) MEM.ARR);
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
