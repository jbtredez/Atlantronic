//! @file adc.c
//! @brief ADC
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/rcc.h"
#include "gpio.h"
#include "adc.h"
#include <string.h>

#define ADC_STACK_SIZE       75

static void adc_task(void* arg);

// adc en double buffer : un en cours de remplissage par dma, un utilisable à tout moment
struct adc_an adc_1;
struct adc_an adc_2;
struct adc_an* adc_dma;
struct adc_an* adc_current;

int adc_module_init()
{
	adc_dma = &adc_1;
	adc_current = &adc_2;

	// activation GPIOB et GPIOC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

	gpio_pin_init(GPIOB, 0, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // AN8  - MOT4
	gpio_pin_init(GPIOB, 1, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // AN9  - MOT1
	gpio_pin_init(GPIOC, 1, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // AN11 - MOT2
	gpio_pin_init(GPIOC, 2, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // AN12 - MOT3
	gpio_pin_init(GPIOC, 5, GPIO_MODE_AN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // AN15 - VBAT_ARU

	// activation clock adc 1
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// reset adc 1 et adc 2
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;


	// mode scan
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = 0x00;//ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTTRIG;

	ADC1->SQR1 = 0x00;
	ADC1->SQR2 = 0x00;
	ADC1->SQR3 = ADC_SQR3_SQ1_3 |                                   ADC_SQR3_SQ1_0 | // 1001 : AN9 (intensité moteur 1)
				 ADC_SQR3_SQ2_3 |                  ADC_SQR3_SQ2_1 | ADC_SQR3_SQ2_0 | // 1011 : AN11  (intensité moteur 2)
				 ADC_SQR3_SQ3_3 | ADC_SQR3_SQ3_2                                   | // 1100 : AN12 (intensité moteur 3)
				 ADC_SQR3_SQ4_3                                                    | // 1000 : AN8 (intensité moteur 4)
				 ADC_SQR3_SQ5_3 | ADC_SQR3_SQ5_2 | ADC_SQR3_SQ5_1 | ADC_SQR3_SQ5_0 ; // 1100 : AN15 (vBatAru)

	// ADCCLK = PCLK2/8 (10.5 MHz)
	ADC->CCR = ADC_CCR_ADCPRE_1 | ADC_CCR_ADCPRE_0;
	ADC1->SMPR2 = ADC_SMPR2_SMP8  | // 480 cycle à ADCCLK pour l'échantillonage de AN8 (intensité moteur 4)
				  ADC_SMPR2_SMP9  ; // 480 cycle à ADCCLK pour l'échantillonage de AN9 (intensité moteur 1)
	ADC1->SMPR1 = ADC_SMPR1_SMP11 | // 480 cycle à ADCCLK pour l'échantillonage de AN11 (intensité moteur 2)
				  ADC_SMPR1_SMP12 | // 480 cycle à ADCCLK pour l'échantillonage de AN12 (intensité moteur 3)
				  ADC_SMPR1_SMP15 ; // 480 cycle à ADCCLK pour l'échantillonage de AN15 (vBatAru)


	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L) | ADC_SQR1_L_2; // 5 conversions - 1

	// dma
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// taille mémoire d'une donnée : 16 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 16 bits
	// incrément automatique mémoire périph : 0
	// mode circulaire
	// transfert : mem périph => mem
	// 5 donnees a transferer avant bouclage
	// DMA2_Stream4 - chan 0
	DMA2_Stream4->CR = DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_TCIE | DMA_SxCR_CHSEL_0;
	DMA2_Stream4->NDTR = 0x05;
	DMA2_Stream4->PAR = (uint32_t) &ADC1->DR;
	DMA2_Stream4->M0AR = (uint32_t) adc_dma;

	// activation dma
	DMA2_Stream4->CR |= DMA_SxCR_EN;
	ADC1->CR2 |= ADC_CR2_DMA;

	// enable adc 1 et adc 2
	ADC1->CR2 |= ADC_CR2_ADON;

	NVIC_SetPriority(DMA2_Stream4_IRQn, PRIORITY_IRQ_DMA2_STREAM4);
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);

	portBASE_TYPE err = xTaskCreate(adc_task, "adc", ADC_STACK_SIZE, NULL, PRIORITY_TASK_ADC, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_ADC;
	}

	return 0;
}

module_init(adc_module_init, INIT_ADC);

static inline void scan_AN()
{
	DMA2_Stream4->M0AR = (uint32_t) adc_dma;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void isr_dma2_stream4(void)
{
	if( DMA2->LISR | DMA_LISR_TCIF0)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		// on change les pointeurs
		struct adc_an* tmp = adc_dma;
		adc_dma = adc_current;
		adc_current = tmp;
	}
}

static void adc_task(void* arg)
{
	(void) arg;

	uint32_t wake_time = 0;

	while(1)
	{
		scan_AN();

		// TODO val + bateries
		//if(adc_dma.vBatAru == 0)
		//{
		//	setLed(ERR_ARU);
		//}

		vTaskDelayUntil(&wake_time, 5);
	}
}

void adc_get(struct adc_an* an)
{
	portENTER_CRITICAL();
	// copie du contenu
	*an = *adc_current;
	portEXIT_CRITICAL();
}
