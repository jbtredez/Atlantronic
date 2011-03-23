#include "tim.h"
#include "log.h"
#include <math.h>
#include <stdio.h>

Tim::Tim()
{
  CR1   = 0x00;
  CR2   = 0x00;
  SMCR  = 0x00;
  DIER  = 0x00;
  SR    = 0x00;
  EGR   = 0x00;
  CCMR1 = 0x00;
  CCMR2 = 0x00;
  CCER  = 0x00;
  CNT   = 0x00;
  PSC   = 0x00;
  ARR   = 0x00;
  RCR   = 0x00;
  CCR1  = 0x00;
  CCR2  = 0x00;
  CCR3  = 0x00;
  CCR4  = 0x00;
  BDTR  = 0x00;
  DCR   = 0x00;
  DMAR  = 0x00;
}

Tim::~Tim()
{

}

void Tim::memory_write(uint64_t offset, uint32_t val)
{
	switch(offset)
	{
		case offsetof(TIM_TypeDef, CCR1):
			CCR1 = val; // TODO
			break;
		case offsetof(TIM_TypeDef, CCR2):
			CCR2 = val; // TODO
			break;
		default:
			meslog(_erreur_, "ecriture non supportée offset %#lx, val %#x", offset, val);
			break;
	}
}

uint32_t Tim::memory_read(uint64_t offset)
{
	uint32_t rep = 0;

	switch(offset)
	{
		case offsetof(TIM_TypeDef, CNT):
			rep = CNT;
			// TODO
			break;
		default:
			meslog(_erreur_, "lecture non supportée offset %#lx", offset);
			break;
	}

	return rep;
}

float Tim::getPwm(int num)
{
	float val = 0;

	if(	CR1 & TIM_CR1_CEN && ARR > 0)
	{
		if(num == 0)
		{
			val = ((float) CCR1) / ((float) ARR);
		}
		else if(num == 1)
		{
			val = ((float) CCR2) / ((float) ARR);
		}
		else if(num == 2)
		{
			val = ((float) CCR3) / ((float) ARR);
		}
		else if(num == 3)
		{
			val = ((float) CCR4) / ((float) ARR);
		}
		else
		{
			// TODO : erreur
		}
	}

	return val;
}

void Tim::setEncoder(uint16_t val)
{
	CNT = val;
}
