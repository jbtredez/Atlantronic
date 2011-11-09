#include "kernel/driver/usart.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/rcc.h"
#include "kernel/error.h"
#include "gpio.h"
#include "kernel/event.h"

#define USART_MAX_DEVICE      2

struct usart_device
{
	USART_TypeDef* const usart;
	DMA_Channel_TypeDef * const dma_read;
	DMA_Channel_TypeDef * const dma_write;
	const uint32_t dma_read_event;
	const uint32_t error_event;
	uint32_t last_error;
};

struct usart_device usart_device[USART_MAX_DEVICE] =
{
	{ USART3, DMA1_Channel3, DMA1_Channel2, EVENT_DMA1_3_TC, EVENT_USART3_ERROR, 0 },
	{ UART4,  DMA2_Channel3, DMA2_Channel5, EVENT_DMA2_3_TC, EVENT_UART4_ERROR,  0 }
};

void usart_set_frequency(enum usart_id id, uint32_t frequency)
{
#ifdef DEBUG
	if(id >= USART_MAX_DEVICE)
	{
		error_raise(ERR_USART_UNKNOWN_DEVICE);
		return;
	}
#endif

	// TODO : pour USART1 : prendre PCLK2
	usart_device[id].usart->BRR = ((RCC_PCLK1*2) / (frequency) + 1) / 2;
}

void usart_open( enum usart_id id, uint32_t frequency)
{
	switch(id)
	{
		case USART3_FULL_DUPLEX:
			// USART3 (pas de remap) : TX = PB10, Rx = PB11
			RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
			AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_USART3_REMAP) | AFIO_MAPR_USART3_REMAP_NOREMAP;

			// GPIOB utilisee, configuration de PB10 et PB11
			RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;
			GPIOB->CRH = ( GPIOB->CRH & ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10 ) | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; // Tx = PB10 : alternate output push-pull, 50MHz
			GPIOB->CRH = ( GPIOB->CRH & ~GPIO_CRH_MODE11 & ~GPIO_CRH_CNF11 ) | GPIO_CRH_CNF11_0; // Rx = PB11 : alternate floating input

			// DMA1 clock enable
			RCC->AHBENR |= RCC_AHBENR_DMA1EN;

			// usart3 clock enable
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

			// 1 start bit, 8 bits data, 1 stop bit, pas de parité
			// activation l'envoi et la reception
			USART3->CR1 = USART_CR1_RE | USART_CR1_TE;
			USART3->CR2 = 0x00;

			// activation des IT d'erreur (nécessaire avec le DMA)
			// DMA en transmission
			// DMA en reception
			USART3->CR3 = USART_CR3_EIE | USART_CR3_DMAT | USART_CR3_DMAR;

			NVIC_SetPriority(USART3_IRQn, PRIORITY_IRQ_USART3);
			NVIC_SetPriority(DMA1_Channel2_IRQn, PRIORITY_IRQ_DMA1_CHANNEL2);
			NVIC_SetPriority(DMA1_Channel3_IRQn, PRIORITY_IRQ_DMA1_CHANNEL3);
			NVIC_EnableIRQ(USART3_IRQn);
			NVIC_EnableIRQ(DMA1_Channel2_IRQn);
			NVIC_EnableIRQ(DMA1_Channel3_IRQn);

			break;
		case UART4_HALF_DUPLEX:
			// UART4 Tx = PC10, (Rx = PC11 pas utilisé en half duplex)

			// GPIOC utilisee, configuration de PC10
			RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;
			GPIOC->CRH = ( GPIOC->CRH & ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10 ) | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; // Tx = PC10 : alternate output push-pull, 50MHz

			// DMA2 clock enable
			RCC->AHBENR |= RCC_AHBENR_DMA2EN;

			// uart4 clock enable
			RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
			
			// 1 start bit, 8 bits data, 1 stop bit, pas de parité
			// activation l'envoi et la reception
			UART4->CR1 = USART_CR1_RE | USART_CR1_TE;
			UART4->CR2 = 0x00;

			// activation des IT d'erreur (nécessaire avec le DMA)
			// mode half duplex
			// DMA en transmission
			// DMA en reception
			UART4->CR3 = USART_CR3_EIE | USART_CR3_HDSEL | USART_CR3_DMAT | USART_CR3_DMAR;

			NVIC_SetPriority(UART4_IRQn, PRIORITY_IRQ_UART4);
			NVIC_SetPriority(DMA2_Channel3_IRQn, PRIORITY_IRQ_DMA2_CHANNEL3);
			NVIC_SetPriority(DMA2_Channel5_IRQn, PRIORITY_IRQ_DMA2_CHANNEL5);
			NVIC_EnableIRQ(UART4_IRQn);
			NVIC_EnableIRQ(DMA2_Channel3_IRQn);
			NVIC_EnableIRQ(DMA2_Channel5_IRQn);
			break;
		default:
			error_raise(ERR_USART_UNKNOWN_DEVICE);
			return;
			break;
	}

	// taille mémoire d'une donnée : 8 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 8 bits
	// incrément automatique mémoire périph : 0
	// transfert (écriture) : mem => mem périph
	// transfert (lecture) : mem => mem périph
	usart_device[id].dma_write->CCR = DMA_CCR1_DIR | DMA_CCR1_MINC | DMA_CCR1_TCIE;
	usart_device[id].dma_write->CPAR = (uint32_t) &usart_device[id].usart->DR;

	usart_device[id].dma_read->CCR = DMA_CCR1_MINC | DMA_CCR1_TCIE;
	usart_device[id].dma_read->CPAR = (uint32_t) &usart_device[id].usart->DR;

	usart_set_frequency(id, frequency);

	// activation usart
	usart_device[id].usart->CR1 |= USART_CR1_UE;
}

void isr_usart3(void)
{
	portSET_INTERRUPT_MASK();

	// affichage de l'erreur
	if( USART3->SR & USART_SR_FE)
	{
		usart_device[USART3_FULL_DUPLEX].last_error = ERR_USART3_READ_SR_FE;
		error_raise(ERR_USART3_READ_SR_FE);
	}

	if( USART3->SR & USART_SR_ORE)
	{
		usart_device[USART3_FULL_DUPLEX].last_error = ERR_USART3_READ_SR_ORE;
		error_raise(ERR_USART3_READ_SR_ORE);
	}

	if( USART3->SR & USART_SR_NE)
	{
		usart_device[USART3_FULL_DUPLEX].last_error = ERR_USART3_READ_SR_NE;
		error_raise(ERR_USART3_READ_SR_NE);
	}

	DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
	// lecture de DR pour effacer les flag d'erreurs (fait en hard si on lis SR puis DR)
	USART3->DR;
	vTaskSetEventFromISR(EVENT_USART3_ERROR);

	portCLEAR_INTERRUPT_MASK();
}

void isr_uart4(void)
{
	portSET_INTERRUPT_MASK();

	// affichage de l'erreur
	if( UART4->SR & USART_SR_FE)
	{
		usart_device[UART4_HALF_DUPLEX].last_error = ERR_UART4_READ_SR_FE;
		error_raise(ERR_UART4_READ_SR_FE);
	}

	if( UART4->SR & USART_SR_ORE)
	{
		usart_device[UART4_HALF_DUPLEX].last_error = ERR_UART4_READ_SR_ORE;
		error_raise(ERR_UART4_READ_SR_ORE);
	}

	if( UART4->SR & USART_SR_NE)
	{
		usart_device[UART4_HALF_DUPLEX].last_error = ERR_UART4_READ_SR_NE;
		error_raise(ERR_UART4_READ_SR_NE);
	}

	DMA2_Channel3->CCR &= ~DMA_CCR3_EN;
	// lecture de DR pour effacer les flag d'erreurs (fait en hard si on lis SR puis DR)
	UART4->DR;
	vTaskSetEventFromISR(EVENT_UART4_ERROR);

	portCLEAR_INTERRUPT_MASK();
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
	portSET_INTERRUPT_MASK();

	if( DMA1->ISR | DMA_ISR_TCIF3)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF3;
		DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
		vTaskSetEventFromISR(EVENT_DMA1_3_TC);
	}

	portCLEAR_INTERRUPT_MASK();
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
	portSET_INTERRUPT_MASK();

	if( DMA2->ISR | DMA_ISR_TCIF3)
	{
		DMA2->IFCR |= DMA_IFCR_CTCIF3;
		DMA2_Channel3->CCR &= ~DMA_CCR3_EN;
		vTaskSetEventFromISR(EVENT_DMA2_3_TC);	
	}

	portCLEAR_INTERRUPT_MASK();
}

void usart_set_read_dma_buffer(enum usart_id id, unsigned char* buf)
{
#ifdef DEBUG
	if(id >= USART_MAX_DEVICE)
	{
		error_raise(ERR_USART_UNKNOWN_DEVICE);
		return;
	}
#endif

	usart_device[id].dma_read->CMAR = (uint32_t) buf;
}

void usart_set_read_dma_size(enum usart_id id, uint16_t size)
{
#ifdef DEBUG
	if(id >= USART_MAX_DEVICE)
	{
		error_raise(ERR_USART_UNKNOWN_DEVICE);
		return;
#endif

	vTaskClearEvent(usart_device[id].dma_read_event | usart_device[id].error_event);
	usart_device[id].dma_read->CNDTR = size;
	// note : DMA_CCR1_EN == DMA_CCRX_EN
	usart_device[id].dma_read->CCR |= DMA_CCR1_EN;
}

uint32_t usart_wait_read(enum usart_id id, portTickType timeout)
{
	uint32_t res = 0;
	uint32_t ev;

#ifdef DEBUG
	if(id >= USART_MAX_DEVICE)
	{
		error_raise(ERR_USART_UNKNOWN_DEVICE);
		return ERR_USART_UNKNOWN_DEVICE;
	}
#endif

	ev = vTaskWaitEvent(usart_device[id].dma_read_event | usart_device[id].error_event, timeout);

	// note : DMA_CCR1_EN == DMA_CCRX_EN
	usart_device[id].dma_read->CCR &= ~DMA_CCR1_EN;
	if(ev & usart_device[id].error_event)
	{
		res = usart_device[id].last_error;
	}
	else if(! (ev & usart_device[id].dma_read_event))
	{
		res = ERR_USART_TIMEOUT;
	}

	return res;
}

void usart_set_write_dma_buffer(enum usart_id id, unsigned char* buf)
{
#ifdef DEBUG
	if(id >= USART_MAX_DEVICE)
	{
		res = ERR_USART_UNKNOWN_DEVICE;
		error_raise(ERR_USART_UNKNOWN_DEVICE);
		return;
	}
#endif

	usart_device[id].dma_write->CMAR = (uint32_t) buf;
}

void usart_send_dma_buffer(enum usart_id id, uint16_t size)
{
#ifdef DEBUG
	if(id >= USART_MAX_DEVICE)
	{
		res = ERR_USART_UNKNOWN_DEVICE;
		error_raise(ERR_USART_UNKNOWN_DEVICE);
		return;
	}
#endif

	usart_device[id].dma_write->CNDTR = size;
	usart_device[id].usart->SR &= ~USART_SR_TC;
	// note : DMA_CCR1_EN == DMA_CCRX_EN
	usart_device[id].dma_write->CCR |= DMA_CCR2_EN;
}