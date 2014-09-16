#include "kernel/module.h"
#include "kernel/driver/usart.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "gpio.h"

struct usart_device
{
	USART_TypeDef* const usart;
	DMA_Stream_TypeDef* const dma_read;
	DMA_Stream_TypeDef* const dma_write;
	xSemaphoreHandle sem;
	uint32_t error;
};

struct usart_device usart_device[USART_MAX_DEVICE] =
{
	{ USART1, DMA2_Stream2, DMA2_Stream7, 0, 0},
	{ USART2, DMA1_Stream5, DMA1_Stream6, 0, 0},
	{ USART3, DMA1_Stream1, DMA1_Stream3, 0, 0},
	{ UART4, DMA1_Stream2, DMA1_Stream4, 0, 0},
	{ UART5, DMA1_Stream0, DMA1_Stream7, 0, 0},
	//{ USART6, DMA2_Stream1, DMA2_Stream7, 0, 0},
};

__OPTIMIZE_SIZE__ void usart_set_frequency(enum usart_id id, uint32_t frequency)
{
	uint32_t pclk;
	// USART1 et USART6 sur PCLK2
	// USART2, USART3, UART4, UART5 sur PCLK1
	switch(id)
	{
		case USART2_FULL_DUPLEX:
		case USART3_FULL_DUPLEX:
		case UART4_FULL_DUPLEX:
		case UART5_HALF_DUPLEX:
			pclk = 1000000*RCC_PCLK1_MHZ;
			break;
		case USART1_FULL_DUPLEX:
		//case USART6_HALF_DUPLEX:
			pclk = 1000000*RCC_PCLK2_MHZ;
			break;
		default:
			// erreur, l'id n'existe pas
			return;
	}

	// formule:
	// USARTDIV = PCLK / (oversampling * baudrate)
	// BRR = partieEntiere(USARTDIV) << 4 + (partiedecimale(USARTDIV)*oversampling+0.5) &0x07

	uint32_t div100;
	// div100 = 100 * PCLK / (oversampling * frequency)
	if( usart_device[id].usart->CR1 & USART_CR1_OVER8)
	{
		// oversampling = 8
		div100  = (pclk * 25) / (2 * frequency);
	}
	else
	{
		// oversampling = 16
		div100  = (pclk * 25) / (4 * frequency);
	}

	uint32_t brr = (div100 / 100) << 4; // on met la partie entiere dans BRR
	uint32_t frac100 = div100 - 100 * (brr >> 4); // 100 * partie decimale
	if( usart_device[id].usart->CR1 & USART_CR1_OVER8)
	{
		// oversampling = 8
		brr |= (((frac100 * 8) + 50)/100) & 0x07;
	}
	else
	{
		// oversampling = 16
		brr |= (((frac100 * 16) + 50)/100) & 0x0f;
	}
	usart_device[id].usart->BRR = brr;
}

__OPTIMIZE_SIZE__ static void usart_init_pin(GPIO_TypeDef* tx_gpio, uint32_t tx_pin, GPIO_TypeDef* rx_gpio, uint32_t rx_pin, uint32_t gpio_af_type)
{
	if( tx_gpio )
	{
		gpio_pin_init(tx_gpio, tx_pin, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
		gpio_af_config(tx_gpio, tx_pin, gpio_af_type);
	}
	if( rx_gpio )
	{
		gpio_pin_init(rx_gpio, rx_pin, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
		gpio_af_config(rx_gpio, rx_pin, gpio_af_type);
	}
}

__OPTIMIZE_SIZE__ int usart_open(enum usart_id id, uint32_t frequency)
{
	switch(id)
	{
		case USART1_FULL_DUPLEX:
			// USART1 : Tx = PA9, Rx = PA10
			// activation USART1
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

			// activation dma2 et GPIOA
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_GPIOAEN;
			usart_init_pin(GPIOA, 9, GPIOA, 10, GPIO_AF_USART1);

			// reset USART1
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
			break;
		case USART2_FULL_DUPLEX:
			// USART2 : Tx = PD5, Rx = PD6
			// activation USART2
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

			// activation dma1 et GPIOD
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_GPIODEN;
			usart_init_pin(GPIOD, 5, GPIOD, 6, GPIO_AF_USART2);

			// reset USART2
			RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
			break;
		case USART3_FULL_DUPLEX:
			// USART3 : Tx = PB10, Rx = PB11
			// activation USART3
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

			// activation dma1 et GPIOB
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_GPIOBEN;
			usart_init_pin(GPIOB, 10, GPIOB, 11, GPIO_AF_USART3);

			// reset USART3
			RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
			break;
		case UART4_FULL_DUPLEX:
			// UART4 : Tx = PC10, Rx = PC11
			// activation UART4
			RCC->APB1ENR |= RCC_APB1ENR_UART4EN;

			// activation dma1 et GPIOC
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_GPIOCEN;
			usart_init_pin(GPIOC, 10, GPIOC, 11, GPIO_AF_UART4);

			// reset USART4
			RCC->APB1RSTR |= RCC_APB1RSTR_UART4RST;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_UART4RST;
			break;
		case UART5_HALF_DUPLEX:
			// UART5 : Tx = PC12, (Rx pas utilisé en half duplex)
			// activation USART5
			RCC->APB1ENR |= RCC_APB1ENR_UART5EN;

			// activation dma1, GPIOC
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_GPIOCEN;
			usart_init_pin(GPIOC, 12, NULL, 0, GPIO_AF_UART5);

			// reset USART5
			RCC->APB1RSTR |= RCC_APB1RSTR_UART5RST;
			RCC->APB1RSTR &= ~RCC_APB1RSTR_UART5RST;
			break;
/*		case USART6_HALF_DUPLEX:
			// UART6 Tx = PC6, Rx = PG9
			// activation USART6
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

			// activation dma2 et GPIOC
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOGEN;
			usart_init_pin(GPIOC, 6, GPIOG, 9, GPIO_AF_USART6);

			// reset USART6
			RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST;
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
			break;*/
		default:
			log_format(LOG_ERROR, "unknown usart id %d", id);
			return -1;
			break;
	}

	// 1 start bit, 8 bits data, 1 stop bit, pas de parité
	// activation l'envoi et la reception
	usart_device[id].usart->CR1 = USART_CR1_RE | USART_CR1_TE;
	usart_device[id].usart->CR2 = 0x00;

	// activation des IT d'erreur (nécessaire avec le DMA)
	// DMA en transmission
	// DMA en reception
	uint32_t cr3 = USART_CR3_EIE | USART_CR3_DMAR | USART_CR3_DMAT;
	if( id == UART5_HALF_DUPLEX )
	{
		// mode half duplex
		cr3 |= USART_CR3_HDSEL;
	}
	usart_device[id].usart->CR3 = cr3;

	// taille mémoire d'une donnée : 8 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 8 bits
	// incrément automatique mémoire périph : 0
	// transfert (écriture) : mem => mem périph
	// transfert (lecture) : mem périph => mem
	usart_device[id].dma_write->CR = DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	usart_device[id].dma_write->PAR = (uint32_t) &usart_device[id].usart->DR;

	usart_device[id].dma_read->CR = DMA_SxCR_MINC | DMA_SxCR_TCIE;
	usart_device[id].dma_read->PAR = (uint32_t) &usart_device[id].usart->DR;

	switch(id)
	{
		case USART1_FULL_DUPLEX:
			usart_device[id].dma_write->CR |= DMA_SxCR_CHSEL_2; // chan4
			usart_device[id].dma_read->CR |= DMA_SxCR_CHSEL_2; // chan4
			NVIC_SetPriority(USART1_IRQn, PRIORITY_IRQ_USART1);
			NVIC_SetPriority(DMA2_Stream2_IRQn, PRIORITY_IRQ_DMA2_STREAM2);
			NVIC_SetPriority(DMA2_Stream7_IRQn, PRIORITY_IRQ_DMA2_STREAM7);
			NVIC_EnableIRQ(USART1_IRQn);
			NVIC_EnableIRQ(DMA2_Stream2_IRQn);
			NVIC_EnableIRQ(DMA2_Stream7_IRQn);
			break;
		case USART2_FULL_DUPLEX:
			usart_device[id].dma_write->CR |= DMA_SxCR_CHSEL_2; // chan4
			usart_device[id].dma_read->CR |= DMA_SxCR_CHSEL_2; // chan4
			NVIC_SetPriority(USART2_IRQn, PRIORITY_IRQ_USART2);
			NVIC_SetPriority(DMA1_Stream5_IRQn, PRIORITY_IRQ_DMA1_STREAM5);
			NVIC_SetPriority(DMA1_Stream6_IRQn, PRIORITY_IRQ_DMA1_STREAM6);
			NVIC_EnableIRQ(USART2_IRQn);
			NVIC_EnableIRQ(DMA1_Stream5_IRQn);
			NVIC_EnableIRQ(DMA1_Stream6_IRQn);
			break;
		case USART3_FULL_DUPLEX:
			usart_device[id].dma_write->CR |= DMA_SxCR_CHSEL_2; // chan4
			usart_device[id].dma_read->CR |= DMA_SxCR_CHSEL_2; // chan4
			NVIC_SetPriority(USART3_IRQn, PRIORITY_IRQ_USART3);
			NVIC_SetPriority(DMA1_Stream1_IRQn, PRIORITY_IRQ_DMA1_STREAM1);
			NVIC_SetPriority(DMA1_Stream3_IRQn, PRIORITY_IRQ_DMA1_STREAM3);
			NVIC_EnableIRQ(USART3_IRQn);
			NVIC_EnableIRQ(DMA1_Stream1_IRQn);
			NVIC_EnableIRQ(DMA1_Stream3_IRQn);
			break;
		case UART4_FULL_DUPLEX:
			usart_device[id].dma_write->CR |= DMA_SxCR_CHSEL_2; // chan4
			usart_device[id].dma_read->CR |= DMA_SxCR_CHSEL_2; // chan4
			NVIC_SetPriority(UART4_IRQn, PRIORITY_IRQ_UART4);
			NVIC_SetPriority(DMA1_Stream2_IRQn, PRIORITY_IRQ_DMA1_STREAM2);
			NVIC_SetPriority(DMA1_Stream4_IRQn, PRIORITY_IRQ_DMA1_STREAM4);
			NVIC_EnableIRQ(UART4_IRQn);
			NVIC_EnableIRQ(DMA1_Stream2_IRQn);
			NVIC_EnableIRQ(DMA1_Stream4_IRQn);
			break;
		case UART5_HALF_DUPLEX:
			usart_device[id].dma_write->CR |= DMA_SxCR_CHSEL_2; // chan4
			usart_device[id].dma_read->CR |= DMA_SxCR_CHSEL_2; // chan4
			NVIC_SetPriority(UART5_IRQn, PRIORITY_IRQ_UART5);
			NVIC_SetPriority(DMA1_Stream0_IRQn, PRIORITY_IRQ_DMA1_STREAM0);
			NVIC_SetPriority(DMA1_Stream7_IRQn, PRIORITY_IRQ_DMA1_STREAM7);
			NVIC_EnableIRQ(UART5_IRQn);
			NVIC_EnableIRQ(DMA1_Stream0_IRQn);
			NVIC_EnableIRQ(DMA1_Stream7_IRQn);
			break;
		/*case USART6_FULL_DUPLEX:
			usart_device[id].dma_write->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0; // chan5
			usart_device[id].dma_read->CR |= DMA_SxCR_CHSEL_2 | DMA_SxCR_CHSEL_0; // chan5
			NVIC_SetPriority(USART6_IRQn, PRIORITY_IRQ_USART6);
			NVIC_SetPriority(DMA2_Stream1_IRQn, PRIORITY_IRQ_DMA2_STREAM1);
			NVIC_SetPriority(DMA2_Stream7_IRQn, PRIORITY_IRQ_DMA2_STREAM7);
			NVIC_EnableIRQ(USART6_IRQn);
			NVIC_EnableIRQ(DMA2_Stream1_IRQn);
			NVIC_EnableIRQ(DMA2_Stream7_IRQn);
			break;*/
		default:
			// pas de log, deja verifie dans le switch du dessus
			return -1;
			break;
	}

	if( usart_device[id].sem == 0)
	{
		vSemaphoreCreateBinary(usart_device[id].sem);
		xSemaphoreTake(usart_device[id].sem, 0);
	}
	usart_set_frequency(id, frequency);

	// activation usart
	usart_device[id].usart->CR1 |= USART_CR1_UE;

	return 0;
}

static void isr_usart_generic(enum usart_id id)
{
	// affichage de l'erreur
	if( usart_device[id].usart->SR & USART_SR_FE)
	{
		usart_device[id].error |= ERR_USART_READ_SR_FE;
	}

	if( usart_device[id].usart->SR & USART_SR_ORE)
	{
		usart_device[id].error |= ERR_USART_READ_SR_ORE;
	}

	if( usart_device[id].usart->SR & USART_SR_NE)
	{
		usart_device[id].error |= ERR_USART_READ_SR_NE;
	}

	// on desactive le DMA de reception
	usart_device[id].dma_read->CR &= ~DMA_SxCR_EN;
	// lecture de DR pour effacer les flag d'erreurs (fait en hard si on lis SR puis DR)
	usart_device[id].usart->DR;

	// pas de xSemaphoreGiveFromISR(usart_device[id].sem) pour attendre le timeout
	// permet de recevoir les octets qui trainent avant d'entammer une nouvelle communication
}

void isr_usart1(void)
{
	isr_usart_generic(USART1_FULL_DUPLEX);
}

void isr_usart2(void)
{
	isr_usart_generic(USART2_FULL_DUPLEX);
}

void isr_usart3(void)
{
	isr_usart_generic(USART3_FULL_DUPLEX);
}

void isr_uart4(void)
{
	isr_usart_generic(UART4_FULL_DUPLEX);
}

void isr_uart5(void)
{
	isr_usart_generic(UART5_HALF_DUPLEX);
}

/*void isr_usart6(void)
{
	isr_usart_generic(USART6_FULL_DUPLEX);
}*/

static void isr_usart_generic_dma_read(enum usart_id id, volatile uint32_t* dma_xISR, volatile uint32_t* dma_xIFCR, uint32_t dma_chan_tcif)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( *dma_xISR | dma_chan_tcif)
	{
		*dma_xIFCR |= dma_chan_tcif; // note : DMA_xIFCR_CTCIFx = DMA_xISR_TCIFx
		usart_device[id].dma_read->CR &= ~DMA_SxCR_EN;
		xSemaphoreGiveFromISR(usart_device[id].sem, &xHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_dma1_stream0(void)
{
	isr_usart_generic_dma_read(UART5_HALF_DUPLEX, &DMA1->LISR, &DMA1->LIFCR, DMA_LISR_TCIF0);
}

void isr_dma1_stream1(void)
{
	isr_usart_generic_dma_read(USART3_FULL_DUPLEX, &DMA1->LISR, &DMA1->LIFCR, DMA_LISR_TCIF1);
}

void isr_dma1_stream2(void)
{
	isr_usart_generic_dma_read(UART4_FULL_DUPLEX, &DMA1->LISR, &DMA1->LIFCR, DMA_LISR_TCIF2);
}

void isr_dma1_stream3(void)
{
	if( DMA1->LISR | DMA_LISR_TCIF3)
	{
		DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
		DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	}
}

void isr_dma1_stream4(void)
{
	if( DMA1->HISR | DMA_HISR_TCIF4)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
		DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	}
}

void isr_dma1_stream5(void)
{
	isr_usart_generic_dma_read(USART2_FULL_DUPLEX, &DMA1->HISR, &DMA1->HIFCR, DMA_HISR_TCIF5);
}

void isr_dma1_stream6(void)
{
	if( DMA1->HISR | DMA_HISR_TCIF6)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
		DMA1_Stream6->CR &= ~DMA_SxCR_EN;
	}
}

void isr_dma1_stream7(void)
{
	if( DMA1->HISR | DMA_HISR_TCIF7)
	{
		DMA1->HIFCR |= DMA_HIFCR_CTCIF7;
		DMA1_Stream7->CR &= ~DMA_SxCR_EN;
	}
}

/*void isr_dma2_stream1(void)
{
	isr_usart_generic_dma_read(USART6_HALF_DUPLEX, &DMA2->LISR, &DMA2->LIFCR, DMA_LISR_TCIF1);
}*/

void isr_dma2_stream2(void)
{
	isr_usart_generic_dma_read(USART1_FULL_DUPLEX, &DMA2->LISR, &DMA2->LIFCR, DMA_LISR_TCIF2);
}

void isr_dma2_stream7(void)
{
	if( DMA2->HISR | DMA_HISR_TCIF7)
	{
		DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
		DMA2_Stream7->CR &= ~DMA_SxCR_EN;
	}
}

void usart_set_read_dma_buffer(enum usart_id id, unsigned char* buf)
{
	usart_device[id].dma_read->M0AR = (uint32_t) buf;
}

void usart_set_read_dma_size(enum usart_id id, uint16_t size)
{
	usart_device[id].error = 0;
	xSemaphoreTake(usart_device[id].sem, 0); // on met a 0 si ce n'est pas le cas
	usart_device[id].dma_read->NDTR = size;
	usart_device[id].dma_read->CR |= DMA_SxCR_EN;
}

uint32_t usart_wait_read(enum usart_id id, portTickType timeout)
{
	uint32_t res = 0;

	if( xSemaphoreTake(usart_device[id].sem, timeout) == pdFALSE )
	{
		res |= ERR_USART_TIMEOUT;;
	}

	usart_device[id].dma_read->CR &= ~DMA_SxCR_EN;

	res |= usart_device[id].error;

	return res;
}

void usart_set_write_dma_buffer(enum usart_id id, unsigned char* buf)
{
	usart_device[id].dma_write->M0AR = (uint32_t) buf;
}

void usart_send_dma_buffer(enum usart_id id, uint16_t size)
{
	usart_device[id].dma_write->NDTR = size;
	usart_device[id].usart->SR &= ~USART_SR_TC;
	usart_device[id].dma_write->CR |= DMA_SxCR_EN;
}
