#include "adc.h"
#include "log.h"
#include <math.h>
#include <stdio.h>

Adc::Adc()
{
	SR    = 0x00;
	CR1   = 0x00;
	CR2   = 0x00;
	SMPR1 = 0x00;
	SMPR2 = 0x00;
	JOFR1 = 0x00;
	JOFR2 = 0x00;
	JOFR3 = 0x00;
	JOFR4 = 0x00;
	HTR   = 0x0FFF;
	LTR   = 0x00;
	SQR1  = 0x00;
	SQR2  = 0x00;
	SQR3  = 0x00;
	JSQR  = 0x00;
	JDR1  = 0x00;
	JDR2  = 0x00;
	JDR3  = 0x00;
	JDR4  = 0x00;
	DR    = 0x00;
}

Adc::~Adc()
{

}

void Adc::memory_write(uint64_t offset, uint32_t val)
{
	switch(offset)
	{
		case offsetof(ADC_TypeDef, SR):
			meslog(_erreur_, "ADC - SR : en lecture seule, impossible d'écrire %#x", val);
			break;
		case offsetof(ADC_TypeDef, CR1):
			CR1 = val & 0xCFFFFF;
			break;
		case offsetof(ADC_TypeDef, CR2):
			CR2 = val & 0xFEFF0F;
			break;
		case offsetof(ADC_TypeDef, SMPR1):
			SMPR1 = val & 0xFFFFFF;
			break;
		case offsetof(ADC_TypeDef, SMPR2):
			SMPR2 = val & 0x3FFFFFFF;
			break;
		case offsetof(ADC_TypeDef, JOFR1):
			JOFR1 = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, JOFR2):
			JOFR2 = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, JOFR3):
			JOFR3 = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, JOFR4):
			JOFR4 = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, HTR):
			HTR = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, LTR):
			LTR = val & 0x0FFF;
			break;
		case offsetof(ADC_TypeDef, SQR1):
			SQR1 = val & 0xFFFFFF;
			break;
		case offsetof(ADC_TypeDef, SQR2):
			SQR2 = val & 0x3FFFFFFF;
			break;
		case offsetof(ADC_TypeDef, SQR3):
			SQR3 = val & 0x3FFFFFFF;
			break;
		case offsetof(ADC_TypeDef, JSQR):
			JSQR = val & 0x3FFFFF;
			break;
		case offsetof(ADC_TypeDef, JDR1):
			meslog(_erreur_, "ADC - JDR1 : en lecture seule, impossible d'écrire %#x", val);
			break;
		case offsetof(ADC_TypeDef, JDR2):
			meslog(_erreur_, "ADC - JDR2 : en lecture seule, impossible d'écrire %#x", val);
			break;
		case offsetof(ADC_TypeDef, JDR3):
			meslog(_erreur_, "ADC - JDR3 : en lecture seule, impossible d'écrire %#x", val);
			break;
		case offsetof(ADC_TypeDef, JDR4):
			meslog(_erreur_, "ADC - JDR4 : en lecture seule, impossible d'écrire %#x", val);
			break;
		case offsetof(ADC_TypeDef, DR):
			meslog(_erreur_, "ADC - DR : en lecture seule, impossible d'écrire %#x", val);
			break;
		default:
			meslog(_erreur_, "ecriture non supportée offset %#lx, val %#x", offset, val);
			break;
	}
}

uint32_t Adc::memory_read(uint64_t offset)
{
	uint32_t rep = 0;

	switch(offset)
	{
		case offsetof(ADC_TypeDef, SR):
			rep = SR;
			break;
		case offsetof(ADC_TypeDef, CR1):
			rep = CR1;
			break;
		case offsetof(ADC_TypeDef, CR2):
			rep = CR2;
			break;
		case offsetof(ADC_TypeDef, SMPR1):
			rep = SMPR1;
			break;
		case offsetof(ADC_TypeDef, SMPR2):
			rep = SMPR2;
			break;
		case offsetof(ADC_TypeDef, JOFR1):
			rep = JOFR1;
			break;
		case offsetof(ADC_TypeDef, JOFR2):
			rep = JOFR2;
			break;
		case offsetof(ADC_TypeDef, JOFR3):
			rep = JOFR3;
			break;
		case offsetof(ADC_TypeDef, JOFR4):
			rep = JOFR4;
			break;
		case offsetof(ADC_TypeDef, HTR):
			rep = HTR;
			break;
		case offsetof(ADC_TypeDef, LTR):
			rep = LTR;
			break;
		case offsetof(ADC_TypeDef, SQR1):
			rep = SQR1;
			break;
		case offsetof(ADC_TypeDef, SQR2):
			rep = SQR2;
			break;
		case offsetof(ADC_TypeDef, SQR3):
			rep = SQR3;
			break;
		case offsetof(ADC_TypeDef, JSQR):
			rep = JSQR;
			break;
		case offsetof(ADC_TypeDef, JDR1):
			rep = JDR1;
			break;
		case offsetof(ADC_TypeDef, JDR2):
			rep = JDR2;
			break;
		case offsetof(ADC_TypeDef, JDR3):
			rep = JDR3;
			break;
		case offsetof(ADC_TypeDef, JDR4):
			rep = JDR4;
			break;
		case offsetof(ADC_TypeDef, DR):
			rep = DR;
			break;
		default:
			meslog(_erreur_, "lecture non supportée offset %#lx", offset);
			break;
	}

	return rep;
}

