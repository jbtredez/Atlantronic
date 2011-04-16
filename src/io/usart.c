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
		case USART3_FULL_DUPLEX:
			// USART3 (pas de remap) : TX = PB10, Rx = PB11
			RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
			AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_USART3_REMAP) | AFIO_MAPR_USART3_REMAP_NOREMAP;
			// TODO
			break;
		case UART4_HALF_DUPLEX:
			// UART4 Tx = PC10, (Rx = PC11 pas utilisé en half duplex)

			// GPIOC utilisee, configuration de PC10
			RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;
			GPIOC->CRH = ( GPIOC->CRH & ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10 ) | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; // Tx = PC10 : alternate output push-pull, 50MHz

			// DMA
			RCC->AHBENR |= RCC_AHBENR_DMA2EN;

			// taille mémoire d'une donnée : 8 bits
			// incrément automatique mémoire : 1
			// taille mémoire périph d'une donnée : 8 bits
			// incrément automatique mémoire périph : 0
			// transfert : mem => mem périph
			// chan 5 => écriture
			// chan 3 => lecture
			DMA2_Channel5->CCR = DMA_CCR5_DIR | DMA_CCR5_MINC | DMA_CCR5_TCIE;
			DMA2_Channel5->CPAR = (uint32_t) &UART4->DR;

			DMA2_Channel3->CCR = DMA_CCR3_MINC | DMA_CCR3_TCIE;
			DMA2_Channel3->CPAR = (uint32_t) &UART4->DR;

			RCC->APB1ENR |= RCC_APB1ENR_UART4EN; // uart4 clock enable

			usart_set_frequency(UART4, frequency);
			
			// 1 start bit, 8 bits data, 1 stop bit, pas de parité
			// activation l'envoi et la reception
			UART4->CR1 = USART_CR1_RE | USART_CR1_TE;
			UART4->CR2 = 0x00;

			// activation des IT d'erreur (nécessaire avec le DMA)
			// mode half duplex
			// DMA en transmission
			// DMA en reception
			UART4->CR3 = USART_CR3_EIE | USART_CR3_HDSEL | USART_CR3_DMAT | USART_CR3_DMAR;

			NVIC_EnableIRQ(UART4_IRQn);
			NVIC_EnableIRQ(DMA2_Channel3_IRQn);
			NVIC_EnableIRQ(DMA2_Channel5_IRQn);

			UART4->CR1 |= USART_CR1_UE;
			break;
		default:
			// TODO erreur
			break;
	}
}

void isr_uart4(void)
{
	// affichage de l'erreur
	if( UART4->SR & USART_SR_FE)
	{
		setLed(ERR_UART4_READ_SR_FE);
	}

	if( UART4->SR & USART_SR_ORE)
	{
		setLed(ERR_UART4_READ_SR_ORE);
	}

	if( UART4->SR & USART_SR_NE)
	{
		setLed(ERR_UART4_READ_SR_NE);
	}

	DMA2_Channel3->CCR &= ~DMA_CCR3_EN;
	// lecture de DR pour effacer les flag d'erreurs (fait en hard si on lis SR puis DR)
	UART4->DR;
	vTaskSetEventFromISR(EVENT_UART4_ERROR);
}

void isr_dma2_channel5(void)
{
	if( DMA2->ISR | DMA_ISR_TCIF5)
	{
		DMA2->IFCR |= DMA_IFCR_CTCIF5;
		DMA2_Channel5->CCR &= ~DMA_CCR5_EN;
	}
}

void isr_dma2_channel3(void)
{
	if( DMA2->ISR | DMA_ISR_TCIF3)
	{
		DMA2->IFCR |= DMA_IFCR_CTCIF3;
		DMA2_Channel3->CCR &= ~DMA_CCR3_EN;
		vTaskSetEventFromISR(EVENT_DMA2_3_TC);	
	}
}

void usart_set_read_dma_buffer(unsigned char* buf)
{
	DMA2_Channel3->CMAR = (uint32_t) buf;
}

void usart_set_read_dma_size(uint16_t size)
{
	vTaskClearEvent(EVENT_DMA2_3_TC | EVENT_UART4_ERROR);
	DMA2_Channel3->CNDTR = size;
	DMA2_Channel3->CCR |= DMA_CCR3_EN;
}

int8_t usart_wait_read(portTickType timeout)
{
	vTaskWaitEvent(EVENT_DMA2_3_TC | EVENT_UART4_ERROR, timeout);
	uint32_t ev = vTaskGetEvent();

	DMA2_Channel3->CCR &= ~DMA_CCR3_EN;

	if( ev & EVENT_DMA2_3_TC)
	{
		return 0;
	}
	
	return -1;
}

void usart_set_write_dma_buffer(unsigned char* buf)
{
	DMA2_Channel5->CMAR = (uint32_t) buf;
}

void usart_send_dma_buffer(uint16_t size)
{
	DMA2_Channel5->CNDTR = size;
	UART4->SR &= ~USART_SR_TC;
	DMA2_Channel5->CCR |= DMA_CCR5_EN;
}

