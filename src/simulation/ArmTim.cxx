#include "ArmTim.h"

ArmTim::ArmTim()
{
	MEM.RESERVED0 = 0;
}

ArmTim::~ArmTim()
{

}


void ArmTim::update(uint64_t offset)
{

}


int32_t ArmTim::getPwm(int num)
{
// FIXME: temporaire. Ce n'est pas correct.
	int32_t val = 0;

	if(	MEM.RESERVED0 == 1)
	{
		if(num == 0)
		{
			val = ((int16_t) MEM.CCR1 ) *2;
		}
		else if(num == 1)
		{
			val = ((int16_t) MEM.CCR2 ) *2;
		}
		else if(num == 2)
		{
			val = ((int16_t) MEM.CCR3 ) *2;
		}
		else if(num == 2)
		{
			val = ((int16_t) MEM.CCR4 ) *2;
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
