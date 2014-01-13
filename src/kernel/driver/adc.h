#ifndef ADC_H
#define ADC_H

//! @file adc.h
//! @brief ADC
//! @author Atlantronic

#include <stdint.h>

struct adc_an
{
	uint16_t i[4];
	uint16_t vBatAru;
};

void adc_get(struct adc_an* an);

#endif
