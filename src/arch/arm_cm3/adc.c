#include "module.h"
#include "FreeRTOS.h"
#include "task.h"

#define ADC_STACK_SIZE       50

volatile uint32_t adc_aru;

static void adc_task(void* arg);

int adc_module_init()
{
	adc_aru = 0;

	// ARU
	// activation de GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	// PA5 entrée AN5
	GPIOA->CRL = (GPIOA->CRL & ~GPIO_CRL_MODE0 & ~GPIO_CRL_CNF0);


	// activation clock adc 1 et adc 2
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;

	// reset adc 1 et adc 2
	RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
//	RCC->APB2RSTR |= RCC_APB2RSTR_ADC2RST;
//	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC2RST;

	ADC1->CR1 = 0x00;
	ADC1->CR1 |= ADC_CR1_EOCIE | ADC_CR1_SCAN;
	ADC1->CR2 = 0x00;
	ADC1->CR2 |= ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTTRIG;

	ADC1->SQR1 = 0x00;
	ADC1->SQR1 = 0x00;
	ADC1->SQR1 = 0x00;
	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR3_SQ1) | ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2; // AN5
	ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP5); // 1.5 cycle (à 12Mhz) pour l'échantillonage de AN5

	// enable adc 1 et adc 2
	ADC1->CR2 |= ADC_CR2_ADON;
//	ADC2->CR2 |= ADC_CR2_ADON;

	NVIC_EnableIRQ(ADC1_2_IRQn);

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

void isr_adc(void)
{
	adc_aru = ADC1->DR;
	if(adc_aru == 0)// TODO val + bateries
	{
		setLed(ERR_ARU);
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
