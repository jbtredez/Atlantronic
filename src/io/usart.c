#include "io/usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "io/rcc.h"
#include "error.h"
#include "io/gpio.h"
#include "event.h"

static void usart_set_frequency(USART_TypeDef* usart, enum usart_frequency frequency)
{
	#if( RCC_PCLK1 != 36000000)
	#error usart->BRR à recalculer
	#endif
	// PCLK = 36 Mhz
	// usart : v                   | 1Mb/s             | 750kb/s   | 19.2kb/s
	// USARTDIV = PCLK / (16 * v)  | 2.25              |    3      | 117,1875
	// mantisse sur 12 bits        | 0x02              |   0x03    | 117 = 0x75
	// fraction sur 4 bits :       | 0.25 * 16 = 0x04  |   0x00    | 16 * 0.1875 = 0x03
	// erreur de fréquence :       |  0%               |    0%     | 0%
	
	switch(frequency)
	{
		case USART_1000000:
			usart->BRR = (((uint16_t)0x02) << 4) | (uint16_t)0x04;
			break;
		case USART_750000:
			usart->BRR = (((uint16_t)0x03) << 4);
			break;
		case USART_19200:
			usart->BRR = (((uint16_t)0x75) << 4) | (uint16_t)0x03;
			break;
		default:
			// TODO erreur
			break;
	}
}

void usart_open( enum usart_id id, enum usart_frequency frequency)
{
	switch(id)
	{
		case USART3_HALF_DUPLEX:
			// USART3 (remapage partiel) => Tx = PC10, (Rx = PC11 pas utilisé en half duplex)
			RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

			// remap PC10 (et PC11)
			AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
			AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_PARTIALREMAP;

			// GPIOC utilisee, configuration de PC10
			RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;
			GPIOC->CRH = ( GPIOC->CRH & ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10 ) | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; // Tx = PC10 : alternate output push-pull, 50MHz

			// DMA
			RCC->AHBENR |= RCC_AHBENR_DMA1EN;

			// taille mémoire d'une donnée : 8 bits
			// incrément automatique mémoire : 1
			// taille mémoire périph d'une donnée : 8 bits
			// incrément automatique mémoire périph : 0
			// transfert : mem => mem périph
			// chan 2 => écriture
			// chan 3 => lecture
			DMA1_Channel2->CCR = DMA_CCR2_DIR | DMA_CCR2_MINC | DMA_CCR2_TCIE;
			DMA1_Channel2->CPAR = (uint32_t) &USART3->DR;

			DMA1_Channel3->CCR = DMA_CCR3_MINC | DMA_CCR3_TCIE;
			DMA1_Channel3->CPAR = (uint32_t) &USART3->DR;

			RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // usart3 clock enable

			usart_set_frequency(USART3, frequency);
			
			// 1 start bit, 8 bits data, 1 stop bit, pas de parité
			// activation l'envoi et la reception
			USART3->CR1 = USART_CR1_RE | USART_CR1_TE;
			USART3->CR2 = 0x00;

			// activation des IT d'erreur (nécessaire avec le DMA)
			// mode half duplex
			// DMA en transmission
			// DMA en reception
			USART3->CR3 = USART_CR3_EIE | USART_CR3_HDSEL | USART_CR3_DMAT | USART_CR3_DMAR;

			NVIC_EnableIRQ(USART3_IRQn);
			NVIC_EnableIRQ(DMA1_Channel2_IRQn);
			NVIC_EnableIRQ(DMA1_Channel3_IRQn);

			USART3->CR1 |= USART_CR1_UE;
			break;
		case USART2_RXTX:
			// TODO
			break;
		default:
			// TODO erreur
			break;
	}
}

void isr_usart3(void)
{
	// affichage de l'erreur
	if( USART3->SR & USART_SR_FE)
	{
		setLed(ERR_USART3_READ_SR_FE);
	}

	if( USART3->SR & USART_SR_ORE)
	{
		setLed(ERR_USART3_READ_SR_ORE);
	}

	if( USART3->SR & USART_SR_NE)
	{
		setLed(ERR_USART3_READ_SR_NE);
	}

	DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
	USART3->DR; // lecture de DR au cas ou pour effacer les flag d'erreurs
	vTaskSetEventFromISR(EVENT_USART3_ERROR);
}

void isr_dma1_channel2(void)
{
	if( DMA1->ISR | DMA_ISR_TCIF2)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF2;
		DMA1_Channel2->CCR &= ~DMA_CCR2_EN;
	}
}

void isr_dma1_channel3(void)
{
	if( DMA1->ISR | DMA_ISR_TCIF3)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
		vTaskSetEventFromISR(EVENT_DMA3_TC);	
	}
}

void usart_set_read_dma_buffer(unsigned char* buf)
{
	DMA1_Channel3->CMAR = (uint32_t) buf;
}

void usart_set_read_dma_size(uint16_t size)
{
	vTaskClearEvent(EVENT_DMA3_TC | EVENT_USART3_ERROR);
	DMA1_Channel3->CNDTR = size;
	DMA1_Channel3->CCR |= DMA_CCR3_EN;
}

// TODO : timeout
int8_t usart_wait_read()
{
	vTaskWaitEvent(EVENT_DMA3_TC | EVENT_USART3_ERROR, portMAX_DELAY);
	if( vTaskGetEvent() & EVENT_USART3_ERROR)
	{
		return -1;
	}
	
	return 0;
}

void usart_set_write_dma_buffer(unsigned char* buf)
{
	DMA1_Channel2->CMAR = (uint32_t) buf;
}

void usart_send_dma_buffer(uint16_t size)
{
	DMA1_Channel2->CNDTR = size;
	USART3->SR &= ~USART_SR_TC;
	DMA1_Channel2->CCR |= DMA_CCR2_EN;
}

