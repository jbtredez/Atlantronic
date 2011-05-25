#ifndef ADC_H
#define ADC_H

//! @file adc.h
//! @brief ADC
//! @author Atlantronic

#include <stdint.h>

struct adc_an
{
	uint16_t potard_right;
	uint16_t potard_left;
	uint16_t i_right;
	uint16_t i_left;
	uint16_t i3;
	uint16_t i4;
	uint16_t vBat1;
	uint16_t vBatAru;
};

void adc_get(struct adc_an* an);

#endif
