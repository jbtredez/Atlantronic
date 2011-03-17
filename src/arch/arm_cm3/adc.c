#include "module.h"
#include "FreeRTOS.h"
#include "task.h"

#define ADC_STACK_SIZE       50

static void adc_task(void* arg);

struct
{
	volatile uint32_t aru;
	volatile uint32_t potard1;
	volatile uint32_t potard2;
} adc_dma ;

int adc_module_init()
{
	// ARU
	// activation de GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	// PA5 entrée AN5
	GPIOA->CRL = (GPIOA->CRL & ~GPIO_CRL_MODE0 & ~GPIO_CRL_CNF0);

	// Potard 1 et 2 sur PC1 (AN11) et PC3 (AN13)
	// activation de GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	// PC1 entrée AN11
	GPIOC->CRL = (GPIOC->CRL & ~GPIO_CRL_MODE1 & ~GPIO_CRL_CNF1);
	// PC3 entrée AN13
	GPIOC->CRL = (GPIOC->CRL & ~GPIO_CRL_MODE3 & ~GPIO_CRL_CNF3);

	// activation clock adc 1 et adc 2
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;

	// reset adc 1 et adc 2
	RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
//	RCC->APB2RSTR |= RCC_APB2RSTR_ADC2RST;
//	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC2RST;

	// mode scan avec selection SWSTART
	ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR2 = ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTTRIG;

	ADC1->SQR1 = 0x00;
	ADC1->SQR2 = 0x00;
	ADC1->SQR3 =                  ADC_SQR3_SQ1_2 |                  ADC_SQR3_SQ1_0 | // 0101 : AN5
	 			 ADC_SQR3_SQ2_3 |                  ADC_SQR3_SQ2_1 | ADC_SQR3_SQ2_0 | // 1011 : AN11
	 			 ADC_SQR3_SQ3_3 | ADC_SQR3_SQ3_2                  | ADC_SQR3_SQ3_0;  // 1101 : AN13

	ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP5); // 1.5 cycle (à 12Mhz) pour l'échantillonage de AN5
	ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP11); // 1.5 cycle (à 12Mhz) pour l'échantillonage de AN11
	ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP13); // 1.5 cycle (à 12Mhz) pour l'échantillonage de AN13

	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L) | ADC_SQR1_L_1; // 3 conversions - 1

	// dma
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	DMA1->IFCR = 0x00;
	// taille mémoire d'une donnée : 32 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 32 bits
	// incrément automatique mémoire périph : 0
	// mode circulaire
	// transfert : mem périph => mem
	DMA1_Channel1->CCR = DMA_CCR1_MSIZE_1 | DMA_CCR1_PSIZE_1 | DMA_CCR1_MINC | DMA_CCR1_CIRC | DMA_CCR1_TCIE;
	DMA1_Channel1->CNDTR = (0x03) & DMA_CNDTR1_NDT;
	DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;
	DMA1_Channel1->CMAR = (uint32_t) & adc_dma;

	// activation dma
	DMA1_Channel1->CCR |= DMA_CCR1_EN;
	ADC1->CR2 |= ADC_CR2_DMA;

	// enable adc 1 et adc 2
	ADC1->CR2 |= ADC_CR2_ADON;
//	ADC2->CR2 |= ADC_CR2_ADON;

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(adc_task, "adc", ADC_STACK_SIZE, NULL, PRIORITY_TASK_ADC, &xHandle);

	if(err != pdPASS)
	{
		return -1; // TODO err code
	}

	return 0;
}

module_init(adc_module_init, INIT_ADC);

static inline void scan_AN()
{
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void isr_dma1_channel1(void)
{
	if( DMA1->ISR | DMA_ISR_TCIF1)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF1;
		if(adc_dma.aru == 0)// TODO val + bateries
		{
			setLed(ERR_ARU);
		}
	}
}

static void adc_task(void* arg)
{
	(void) arg;

	while(1)
	{
		scan_AN();

		vTaskDelay(720000);
	}
}
