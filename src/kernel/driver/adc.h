#ifndef ADC_H
#define ADC_H

//! @file adc.h
//! @brief ADC
//! @author Atlantronic

#include <stdint.h>

#define ADC_CURRENT_MAX                 4


#ifdef __cplusplus
extern "C" {
#endif

struct adc_anf
{
	float i[ADC_CURRENT_MAX];
	float vBat;
};

void adc_update();

extern struct adc_anf adc_filtered_data;

#ifdef __cplusplus
}
#endif

#endif
