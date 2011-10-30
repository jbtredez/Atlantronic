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
	// TODO implementer les veriff accès mem + fonctionalités
	switch(offset)
	{
		case offsetof(TIM_TypeDef, CR1):
			CR1 = val & 0x3FF;
			break;
		case offsetof(TIM_TypeDef, CR2):
			CR2 = val & 0x7FFD;
			break;
		case offsetof(TIM_TypeDef, SMCR):
			SMCR = val;
			break;
		case offsetof(TIM_TypeDef, DIER):
			DIER = val & 0x7FFF;
			break;
		case offsetof(TIM_TypeDef, SR):
			SR = val;
			break;
		case offsetof(TIM_TypeDef, EGR):
			EGR = val & 0xFF;
			break;
		case offsetof(TIM_TypeDef, CCMR1):
			CCMR1 = val;
			break;
		case offsetof(TIM_TypeDef, CCMR2):
			CCMR2 = val;
			break;
		case offsetof(TIM_TypeDef, CCER):
			CCER = val;
			break;
		case offsetof(TIM_TypeDef, CNT):
			CNT = val;
			break;
		case offsetof(TIM_TypeDef, PSC):
			PSC = val;
			break;
		case offsetof(TIM_TypeDef, ARR):
			ARR = val;
			break;
		case offsetof(TIM_TypeDef, RCR):
			RCR = val;
			break;
		case offsetof(TIM_TypeDef, CCR1):
			CCR1 = val;
			break;
		case offsetof(TIM_TypeDef, CCR2):
			CCR2 = val;
			break;
		case offsetof(TIM_TypeDef, CCR3):
			CCR3 = val;
			break;
		case offsetof(TIM_TypeDef, CCR4):
			CCR4 = val;
			break;
		case offsetof(TIM_TypeDef, BDTR):
			BDTR = val;
			break;
		case offsetof(TIM_TypeDef, DCR):
			DCR = val;
			break;
		case offsetof(TIM_TypeDef, DMAR):
			DMAR = val;
			break;
		default:
			meslog(_erreur_, "ecriture non supportée offset %#"PRIx64", val %#"PRIx32, offset, val);
			break;
	}
}

uint32_t Tim::memory_read(uint64_t offset)
{
	uint32_t rep = 0;
// TODO : registres interdit en lecture
	switch(offset)
	{
		case offsetof(TIM_TypeDef, CR1):
			rep = CR1;
			break;
		case offsetof(TIM_TypeDef, CR2):
			rep = CR2;
			break;
		case offsetof(TIM_TypeDef, SMCR):
			rep = SMCR;
			break;
		case offsetof(TIM_TypeDef, DIER):
			rep = DIER;
			break;
		case offsetof(TIM_TypeDef, SR):
			rep = SR;
			break;
		case offsetof(TIM_TypeDef, EGR):
			rep = EGR;
			break;
		case offsetof(TIM_TypeDef, CCMR1):
			rep = CCMR1;
			break;
		case offsetof(TIM_TypeDef, CCMR2):
			rep = CCMR2;
			break;
		case offsetof(TIM_TypeDef, CCER):
			rep = CCER;
			break;
		case offsetof(TIM_TypeDef, CNT):
			rep = CNT;
			break;
		case offsetof(TIM_TypeDef, PSC):
			rep = PSC;
			break;
		case offsetof(TIM_TypeDef, ARR):
			rep = ARR;
			break;
		case offsetof(TIM_TypeDef, RCR):
			rep = RCR;
			break;
		case offsetof(TIM_TypeDef, CCR1):
			rep = CCR1;
			break;
		case offsetof(TIM_TypeDef, CCR2):
			rep = CCR2;
			break;
		case offsetof(TIM_TypeDef, CCR3):
			rep = CCR3;
			break;
		case offsetof(TIM_TypeDef, CCR4):
			rep = CCR4;
			break;
		case offsetof(TIM_TypeDef, BDTR):
			rep = BDTR;
			break;
		case offsetof(TIM_TypeDef, DCR):
			rep = DCR;
			break;
		case offsetof(TIM_TypeDef, DMAR):
			rep = DMAR;
			break;
		default:
			meslog(_erreur_, "lecture non supportée offset %#"PRIx64, offset);
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
