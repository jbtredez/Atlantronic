//! @file adc.c
//! @brief ADC
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/driver/power.h"
#include "kernel/systick.h"
#include "gpio.h"
#include "adc.h"
#include <string.h>

//! facteur multiplicatif pour compenser l'erreur entre la pratique et le gain theorique
#define VBAT_CALIBRATION              0.984f
#define VBAT_GAIN            (VBAT_CALIBRATION*13*3/4096.0f)
#define IPWM_GAIN            (3/(4096.0f * 0.377f))
#define ADC_MAX_DATA                  128
#define ADC_FILTER_GAIN        0.9539466f// exp(-period/tau) avec period = 1/(4.242kHz) et tau = 5ms
#define ADC_UNDERVOLTAGE_DELAY         10

struct adc_an
{
	uint16_t i[ADC_CURRENT_MAX];
	uint16_t vBat;
} __attribute__((packed));

static volatile struct adc_an adc_data[ADC_MAX_DATA];
struct adc_anf adc_filtered_data;
static int adc_startId;
static systime adc_undervoltage_time;
static int adc_undervoltage;

int adc_module_init()
{
	// activation GPIOB et GPIOC et DMA2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;

	gpio_pin_init(GPIOB, 0, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // AN8  - MOT4
	gpio_pin_init(GPIOB, 1, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // AN9  - MOT1
	gpio_pin_init(GPIOC, 1, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // AN11 - MOT2
	gpio_pin_init(GPIOC, 2, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // AN12 - MOT3
	gpio_pin_init(GPIOC, 5, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // AN15 - VBAT_ARU

	// activation clock adc 1
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// reset adc 1 et adc 2
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;


	// mode scan continu
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_CONT;

	ADC1->SQR1 = 0x00;
	ADC1->SQR2 = 0x00;
	ADC1->SQR3 = ADC_SQR3_SQ1_3 |                                   ADC_SQR3_SQ1_0 | // 1001 : AN9 (intensité moteur 1)
				 ADC_SQR3_SQ2_3 |                  ADC_SQR3_SQ2_1 | ADC_SQR3_SQ2_0 | // 1011 : AN11  (intensité moteur 2)
				 ADC_SQR3_SQ3_3 | ADC_SQR3_SQ3_2                                   | // 1100 : AN12 (intensité moteur 3)
				 ADC_SQR3_SQ4_3                                                    | // 1000 : AN8 (intensité moteur 4)
				 ADC_SQR3_SQ5_3 | ADC_SQR3_SQ5_2 | ADC_SQR3_SQ5_1 | ADC_SQR3_SQ5_0 ; // 1111 : AN15 (vBatAru)

	// ADCCLK = PCLK2/8 (10.5 MHz)
	ADC->CCR = ADC_CCR_ADCPRE_1 | ADC_CCR_ADCPRE_0;
	ADC1->SMPR2 = ADC_SMPR2_SMP8  | // 480 cycle à ADCCLK pour l'échantillonage de AN8 (intensité moteur 4)
				  ADC_SMPR2_SMP9  ; // 480 cycle à ADCCLK pour l'échantillonage de AN9 (intensité moteur 1)
	ADC1->SMPR1 = ADC_SMPR1_SMP11 | // 480 cycle à ADCCLK pour l'échantillonage de AN11 (intensité moteur 2)
				  ADC_SMPR1_SMP12 | // 480 cycle à ADCCLK pour l'échantillonage de AN12 (intensité moteur 3)
				  ADC_SMPR1_SMP15 ; // 480 cycle à ADCCLK pour l'échantillonage de AN15 (vBatAru)

	// temps total de conversion : 480 cycles + 15 cycles (resolution 12 bit)
	// => frequence adc de 10.5MhZ / (480+15) = 21.212kHz
	// => frequence de 4.242kHz par mesure

	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L) | ADC_SQR1_L_2; // 5 conversions - 1

	// dma
	// taille mémoire d'une donnée : 16 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 16 bits
	// incrément automatique mémoire périph : 0
	// mode circulaire
	// transfert : mem périph => mem
	// 5 donnees a transferer avant bouclage
	// double buffer
	// DMA2_Stream4 - chan 0
	DMA2_Stream4->CR = DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE;// | DMA_SxCR_DBM;
	DMA2_Stream4->NDTR = sizeof(adc_data) / sizeof(uint16_t);
	DMA2_Stream4->PAR = (uint32_t) &ADC1->DR;
	DMA2_Stream4->M0AR = (uint32_t) adc_data;

	// activation dma et mode continu
	DMA2_Stream4->CR |= DMA_SxCR_EN;
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;

	// enable adc 1 et adc 2
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->CR2 |= ADC_CR2_SWSTART;

	int i;
	for(i = 0; i < ADC_CURRENT_MAX; i++)
	{
		adc_filtered_data.i[0] = 0;
	}
	adc_filtered_data.vBat = 0;

	return 0;
}

module_init(adc_module_init, INIT_ADC);

void adc_update()
{
	int end = (sizeof(adc_data) - DMA2_Stream4->NDTR) / sizeof(adc_data[0]);
	int count = end - adc_startId;
	if( count < 0 )
	{
		count += ADC_MAX_DATA;
	}

	int i = 0;
	for(i = 0; i < count; i++)
	{
		adc_filtered_data.vBat = ADC_FILTER_GAIN * adc_filtered_data.vBat + (1-ADC_FILTER_GAIN) * VBAT_GAIN * adc_data[adc_startId].vBat;
		adc_filtered_data.i[0] = ADC_FILTER_GAIN * adc_filtered_data.i[0] + (1-ADC_FILTER_GAIN) * IPWM_GAIN * adc_data[adc_startId].i[0];
		adc_filtered_data.i[1] = ADC_FILTER_GAIN * adc_filtered_data.i[1] + (1-ADC_FILTER_GAIN) * IPWM_GAIN * adc_data[adc_startId].i[1];
		adc_filtered_data.i[2] = ADC_FILTER_GAIN * adc_filtered_data.i[2] + (1-ADC_FILTER_GAIN) * IPWM_GAIN * adc_data[adc_startId].i[2];
		adc_filtered_data.i[3] = ADC_FILTER_GAIN * adc_filtered_data.i[3] + (1-ADC_FILTER_GAIN) * IPWM_GAIN * adc_data[adc_startId].i[3];
		adc_startId = (adc_startId + 1) % ADC_MAX_DATA;
	}

	if( adc_filtered_data.vBat < ADC_VBAT_AU)
	{
		power_set(POWER_OFF_AU);
	}
	else if ( adc_filtered_data.vBat > ADC_VBAT_AU_CLEAR)
	{
		power_clear(POWER_OFF_AU);
	}

	if( adc_filtered_data.vBat < ADC_VBAT_UNDERVOLTAGE)
	{
		systime t = systick_get_time();
		if( ! adc_undervoltage )
		{
			adc_undervoltage = 1;
			adc_undervoltage_time = t;
		}

		if( (adc_undervoltage_time - t).ms > 1000*ADC_UNDERVOLTAGE_DELAY )
		{
			power_set(POWER_OFF_UNDERVOLTAGE);
		}
	}
	else if( adc_filtered_data.vBat > ADC_VBAT_UNDERVOLTAGE_CLEAR)
	{
		power_clear(POWER_OFF_UNDERVOLTAGE);
		adc_undervoltage = 0;
	}
}

