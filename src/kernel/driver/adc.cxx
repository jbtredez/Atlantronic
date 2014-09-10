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

typedef struct
{
	GPIO_TypeDef* gpio;
	uint8_t pin;
	uint8_t an;
}AnParam;

#if defined(__discovery__)
#define ADC_DMA_CHAN               DMA2_Stream4
AnParam anParam[] =
{
	{GPIOB, 1, 9},  // MOT1
	{GPIOC, 1, 11}, // MOT2
	{GPIOC, 2, 12}, // MOT3
	{GPIOB, 0, 8},  // MOT4
	{GPIOC, 5, 15}, // VBAT_ARU
};

struct adc_an
{
	uint16_t i[ADC_CURRENT_MAX];
	uint16_t vBat;
} __attribute__((packed));

#elif defined(__disco__)
#define ADC_DMA_CHAN               DMA2_Stream0
AnParam anParam[] =
{
	{GPIOA, 4, 4},  // MOT1
	{GPIOA, 5, 5},  // MOT2
	{GPIOA, 6, 6},  // MOT3
	{GPIOB, 1, 9},  // MOT4
	{GPIOB, 0, 8},  // VBAT_ARU
	{GPIOA, 7, 7},
	{GPIOC, 3, 13},
	{GPIOC, 4, 14},
	{GPIOC, 5, 15},
};

struct adc_an
{
	uint16_t i[ADC_CURRENT_MAX];
	uint16_t vBat;
	uint16_t an[4];
} __attribute__((packed));

#else
#error unknown card
#endif

static volatile struct adc_an adc_data[ADC_MAX_DATA];
struct adc_anf adc_filtered_data;
static int adc_startId;
static systime adc_undervoltage_time;
static int adc_undervoltage;

static void adc_init(uint8_t id, GPIO_TypeDef* gpio, uint8_t pin, uint8_t an);

int adc_module_init()
{
#if defined(__discovery__)
	// activation GPIOB et GPIOC et DMA2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;
#elif defined(__disco__)
	// activation GPIOA, GPIOB, GPIOC et DMA2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;
#else
#error unknown card
#endif

	// activation clock adc 1
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// reset adc 1
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;

	// mode scan continu
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_CONT;

	ADC1->SQR1 = 0;
	ADC1->SQR2 = 0;
	ADC1->SQR3 = 0;

	for(unsigned int i = 0; i < sizeof(anParam) / sizeof(anParam[0]); i++)
	{
		adc_init(i, anParam[i].gpio, anParam[i].pin, anParam[i].an);
	}

	// ADCCLK = PCLK2/8 (10.5 MHz)
	ADC->CCR = ADC_CCR_ADCPRE_1 | ADC_CCR_ADCPRE_0;

	// temps total de conversion : 480 cycles + 15 cycles (resolution 12 bit)
	// => frequence adc de 10.5MhZ / (480+15) = 21.212kHz
	// => frequence de 4.242kHz par mesure
	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L) | (ADC_SQR1_L_0 * (sizeof(anParam) / sizeof(anParam[0]) - 1)); // x conversions - 1

	// dma
	// taille mémoire d'une donnée : 16 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 16 bits
	// incrément automatique mémoire périph : 0
	// mode circulaire
	// transfert : mem périph => mem
	// 5 donnees a transferer avant bouclage
	// double buffer
	// DMA2_Stream0 - chan 0
	ADC_DMA_CHAN->CR = DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE;// | DMA_SxCR_DBM;
	ADC_DMA_CHAN->NDTR = sizeof(adc_data) / sizeof(uint16_t);
	ADC_DMA_CHAN->PAR = (uint32_t) &ADC1->DR;
	ADC_DMA_CHAN->M0AR = (uint32_t) adc_data;

	// activation dma et mode continu
	ADC_DMA_CHAN->CR |= DMA_SxCR_EN;
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;

	// enable adc 1
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

static void adc_init(uint8_t id, GPIO_TypeDef* gpio, uint8_t pin, uint8_t an)
{
	gpio_pin_init(gpio, pin, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);

	int sqrx = id / 6;
	int sqrx_shift = 5*(id - 6 * sqrx);
	int smrx = an / 10;
	int smrx_shift = 3*(an - 10 * smrx);

	switch(sqrx)
	{
		case 0:
			ADC1->SQR3 &= ~(0x1f << sqrx_shift);
			ADC1->SQR3 |= an << sqrx_shift;
			break;
		case 1:
			ADC1->SQR2 &= ~(0x1f << sqrx_shift);
			ADC1->SQR2 |= an << sqrx_shift;
			break;
		case 2:
			ADC1->SQR1 &= ~(0x1f << sqrx_shift);
			ADC1->SQR1 |= an << sqrx_shift;
			break;
		default:
			break;
	}

	switch(smrx)
	{
		case 0:
			ADC1->SMPR2 |= 0x07 << smrx_shift; // 480 cycles
			break;
		case 1:
			ADC1->SMPR1 |= 0x07 << smrx_shift; // 480 cycles
			break;
		default:
			break;
	}
}

void adc_update()
{
	int end = (sizeof(adc_data) - ADC_DMA_CHAN->NDTR) / sizeof(adc_data[0]);
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

	if( adc_filtered_data.vBat < ADC_VBAT_UNDERVOLTAGE && adc_filtered_data.vBat > ADC_VBAT_AU_CLEAR)
	{
		systime t = systick_get_time();
		if( ! adc_undervoltage )
		{
			adc_undervoltage = 1;
			adc_undervoltage_time = t;
		}

		if( (t - adc_undervoltage_time).ms > 1000*ADC_UNDERVOLTAGE_DELAY )
		{
			power_set(POWER_OFF_UNDERVOLTAGE);
		}
	}
	else if( adc_filtered_data.vBat > ADC_VBAT_UNDERVOLTAGE_CLEAR || adc_filtered_data.vBat < ADC_VBAT_AU)
	{
		power_clear(POWER_OFF_UNDERVOLTAGE);
		adc_undervoltage = 0;
	}
}

