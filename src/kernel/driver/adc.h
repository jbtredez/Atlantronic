#ifndef ADC_H
#define ADC_H

//! @file adc.h
//! @brief ADC
//! @author Atlantronic

#include <stdint.h>

#define ADC_CURRENT_MAX                 4
#define ADC_VBAT_UNDERVOLTAGE          18
#define ADC_VBAT_UNDERVOLTAGE_CLEAR    19
#define ADC_VBAT_AU                     3
#define ADC_VBAT_AU_CLEAR               5

struct adc_anf
{
	float i[ADC_CURRENT_MAX];
	float vBat;
	float an[4];
};

void adc_update();

extern struct adc_anf adc_filtered_data;

#endif
