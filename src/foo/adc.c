//! @file adc.c
//! @brief ADC
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
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
	// activation de GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRL = GPIOA->CRL & ~ (
	                  GPIO_CRL_MODE4 | GPIO_CRL_CNF4 |  // PA4 entrée AN4
	                  GPIO_CRL_MODE5 | GPIO_CRL_CNF5    // PA5 entrée AN5
	             );

	// activation de GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

	GPIOC->CRL = GPIOC->CRL & ~(
	                  GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |  // PC0 entrée AN10
	                  GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |  // PC1 entrée AN11
	                  GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |  // PC2 entrée AN12
	                  GPIO_CRL_MODE3 | GPIO_CRL_CNF3 |  // PC3 entrée AN13
	                  GPIO_CRL_MODE4 | GPIO_CRL_CNF4 |  // PC4 entrée AN14
	                  GPIO_CRL_MODE5 | GPIO_CRL_CNF5    // PC5 entrée AN15
                 );

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
	ADC1->SQR3 =                  ADC_SQR3_SQ1_2                                   | // 0100 : AN4  (potentiomètre 1)
	 			 ADC_SQR3_SQ2_3 | ADC_SQR3_SQ2_2 | ADC_SQR3_SQ2_1                  | // 1110 : AN14 (potentiomètre 2)
 	 			 ADC_SQR3_SQ3_3 | ADC_SQR3_SQ3_2 | ADC_SQR3_SQ3_1 | ADC_SQR3_SQ3_0 | // 1111 : AN15 (intensité moteur 1)
 	 			                  ADC_SQR3_SQ4_2 |                  ADC_SQR3_SQ4_0 | // 0101 : AN5  (intensité moteur 2)
 	 			 ADC_SQR3_SQ5_3 | ADC_SQR3_SQ5_2 |                  ADC_SQR3_SQ5_0 | // 1101 : AN13 (intensité moteur 3)
 	 			 ADC_SQR3_SQ6_3 |                  ADC_SQR3_SQ6_1 | ADC_SQR3_SQ6_0 ; // 1011 : AN11 (intensité moteur 4)

	ADC1->SQR2 = ADC_SQR2_SQ7_3 | ADC_SQR2_SQ7_2                                   | // 1100 : AN12 (vBat1)
	             ADC_SQR2_SQ8_3 |                  ADC_SQR2_SQ8_1                  ; // 1010 : AN10 (vBatAru)

	ADC1->SMPR2 = ADC_SMPR2_SMP4  | // 239.5 cycle (à 12Mhz) pour l'échantillonage de AN4 (potentiomètre 1)
	              ADC_SMPR2_SMP5  ; // 239.5 cycle (à 12Mhz) pour l'échantillonage de AN5 (intensité moteur 2)
	ADC1->SMPR1 = ADC_SMPR1_SMP10 | // 239.5 cycle (à 12Mhz) pour l'échantillonage de AN10 (vBatAru)
	              ADC_SMPR1_SMP11 | // 239.5 cycle (à 12Mhz) pour l'échantillonage de AN11 (intensité moteur 4)
	              ADC_SMPR1_SMP12 | // 239.5 cycle (à 12Mhz) pour l'échantillonage de AN12 (vBat1)
	              ADC_SMPR1_SMP13 | // 239.5 cycle (à 12Mhz) pour l'échantillonage de AN13 (intensité moteur 3)
	              ADC_SMPR1_SMP14 | // 239.5 cycle (à 12Mhz) pour l'échantillonage de AN14 (potentiomètre 2)
	              ADC_SMPR1_SMP15 ; // 239.5 cycle (à 12Mhz) pour l'échantillonage de AN15 (intensité moteur 1)

	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L) | ADC_SQR1_L_2 | ADC_SQR1_L_1 | ADC_SQR1_L_0; // 8 conversions - 1

	// dma
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// taille mémoire d'une donnée : 16 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 16 bits
	// incrément automatique mémoire périph : 0
	// mode circulaire
	// transfert : mem périph => mem
	DMA1_Channel1->CCR = DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0 | DMA_CCR1_MINC | DMA_CCR1_CIRC | DMA_CCR1_TCIE;
	DMA1_Channel1->CNDTR = (0x08) & DMA_CNDTR1_NDT;
	DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;
	DMA1_Channel1->CMAR = (uint32_t) adc_dma;

	// activation dma
	DMA1_Channel1->CCR |= DMA_CCR1_EN;
	ADC1->CR2 |= ADC_CR2_DMA;

	// enable adc 1 et adc 2
	ADC1->CR2 |= ADC_CR2_ADON;
//	ADC2->CR2 |= ADC_CR2_ADON;

	NVIC_SetPriority(DMA1_Channel1_IRQn, PRIORITY_IRQ_DMA1_CHANNEL1);
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
	DMA1_Channel1->CMAR = (uint32_t) adc_dma;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void isr_dma1_channel1(void)
{
//	portSET_INTERRUPT_MASK();

	if( DMA1->ISR | DMA_ISR_TCIF1)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF1;
		// on change les pointeurs
		struct adc_an* tmp = adc_dma;
		adc_dma = adc_current;
		adc_current = tmp;
//		vTaskSetEventFromISR(EVENT_ADC_READY);
	}

//	portCLEAR_INTERRUPT_MASK();
}

static void adc_task(void* arg)
{
	(void) arg;

	portTickType wake_time = systick_get_time();

	while(1)
	{
//		vTaskClearEvent(EVENT_ADC_READY);
		scan_AN();
//		vTaskWaitEvent(EVENT_ADC_READY, ms_to_tick(5));

		// TODO val + bateries
		//if(adc_dma.vBatAru == 0)
		//{
		//	setLed(ERR_ARU);
		//}

		wake_time += ms_to_tick(5);
		vTaskDelayUntil(wake_time);
	}
}

void adc_get(struct adc_an* an)
{
	portENTER_CRITICAL();
	// copie du contenu
	*an = *adc_current;
	portEXIT_CRITICAL();
}
