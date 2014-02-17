#ifndef ADC_H
#define ADC_H

//! @file adc.h
//! @brief ADC
//! @author Atlantronic

#include <stdint.h>

struct adc_an
{
	uint16_t i[4];
	uint16_t vBat;
};

//! facteur multiplicatif pour compenser l'erreur entre la pratique et le gain theorique
#define VBAT_CALIBRATION              0.985f
#define VBAT_GAIN            (VBAT_CALIBRATION*13*3/4096.0f)
#define IPWM_GAIN            (3/(4096.0f * 0.377))

extern volatile struct adc_an adc_data;

#endif
