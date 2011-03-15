#include "io/usart.h"
#include "module.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "io/rcc.h"

#define USART_WRITE_BUF_SIZE     256
#define USART_READ_BUF_SIZE      256

#if USART_WRITE_BUF_SIZE < 2
#error USART_WRITE_BUF_SIZE trop petit
#elif ((USART_WRITE_BUF_SIZE & (USART_WRITE_BUF_SIZE-1)) != 0)
#error USART_WRITE_BUF_SIZE doit être une puissance de 2.
#endif

#if USART_READ_BUF_SIZE < 2
#error USART_READ_BUF_SIZE trop petit
#elif ((USART_READ_BUF_SIZE & (USART_READ_BUF_SIZE-1)) != 0)
#error USART_READ_BUF_SIZE doit être une puissance de 2.
#endif

static uint8_t  usart_write_buf[USART_WRITE_BUF_SIZE];
static volatile uint32_t usart_write_buf_in;
static volatile uint32_t usart_write_buf_out;
static uint8_t  usart_read_buf[USART_READ_BUF_SIZE];
static volatile uint32_t usart_read_buf_in;
static volatile uint32_t usart_read_buf_out;

static int usart_module_init(void)
{
	usart_write_buf_in = 0;
	usart_write_buf_out = 0;
	usart_read_buf_in = 0;
	usart_read_buf_out = 0;

	// USART3 (remapage partiel) => Tx = PC10, (Rx = PC11 pas utilisé en half duplex)
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	// remap PC10 (et PC11)
	AFIO->MAPR &= ~AFIO_MAPR_USART3_REMAP;
	AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_PARTIALREMAP;

	// GPIOC utilisee, configuration de PC10
	RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;
	GPIOC->CRH = ( GPIOC->CRH & ~GPIO_CRH_MODE10 & ~GPIO_CRH_CNF10 ) | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1; // Tx = PC10 : alternate output push-pull, 50MHz

	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // usart3 clock enable

	// usart : v = 1Mb/s, PCLK = 36 Mhz
	// USARTDIV = PCLK / (16 * v) = 2.25
	// mantisse sur 12 bits : 0x02
	// fraction sur 4 bits : 0.25 * 16 = 0x04
	// erreur de fréquence : 0% (pas d'arrondi)
	#if( RCC_PCLK1 != 36000000)
	#error usart3->BRR à recalculer
	#endif
	USART3->BRR = (((uint16_t)0x02) << 4) | (uint16_t)0x04;

	// 1 start bit, 8 bits data, 1 stop bit, pas de parité
	// interruption sur TXE (transmit data register empty)
	USART3->CR1 = 0x00;
	USART3->CR1 |= USART_CR1_TXEIE | USART_CR1_RXNEIE;
// TODO ; voir si on active d'autres it ( TCIE, IDLEIE)
	USART3->CR2 = 0x00;
	USART3->CR3 = 0x00;

	USART3->CR1 |= (USART_CR1_RE | USART_CR1_TE);  // activation l'envoi et la reception

	// passage en mode half duplex
	USART3->CR3 |= USART_CR3_HDSEL;

	USART3->CR1 |= USART_CR1_UE;

	NVIC_EnableIRQ(USART3_IRQn);

	return 0;
}

module_init(usart_module_init, INIT_USART);

void isr_usart3(void)
{
	// lecture : un octet est arrivé
	if(USART3->SR & USART_SR_RXNE)
	{
		if( ((usart_read_buf_in - usart_read_buf_out) & ~(USART_READ_BUF_SIZE-1)) == 0)
		{
			usart_read_buf[usart_read_buf_in & (USART_READ_BUF_SIZE-1)] = USART3->DR & 0xFF;
			usart_read_buf_in++;
		}
		// else
		// {
		// problemes, reception reportée si on ne fait pas USART3->SR &= ~USART_SR_RXNE; et en cas de débordement, TODO gestion erreur
		// }
	}

	// écriture : pas d'octets dans le registre d'envoi
	if(USART3->SR & USART_SR_TXE)
	{
		if(usart_write_buf_in != usart_write_buf_out)
		{
			USART3->DR = usart_write_buf[usart_write_buf_out & (USART_WRITE_BUF_SIZE -1)] & ((uint16_t)0x01FF);
			usart_write_buf_out++;
		}
		else
		{
			// on désactive l'it si on a plus rien a envoyer
			USART3->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}

void usart_write(unsigned char* buf, uint16_t size)
{
	for( ; size--; )
	{
		// cas d'overflow de in et pas de out non géré (out > in) mais on va pas déborder le uint32_t (4Go sur l'usart...)
		if( (usart_write_buf_in - usart_write_buf_out) < USART_WRITE_BUF_SIZE )
		{
			usart_write_buf[usart_write_buf_in & (USART_WRITE_BUF_SIZE -1)] = *buf;
			buf++;
			usart_write_buf_in++;
		}
		else
		{
			// TODO remonter une erreur (log, led...)
			USART3->CR1 |= USART_CR1_TXEIE;
			size++; // on n'a finalement pas écris notre octet
			vTaskDelay(2*72000);
		}
	}
	USART3->CR1 |= USART_CR1_TXEIE;
}

uint16_t usart_read(unsigned char* buf, uint16_t size)
{
	uint16_t s = usart_read_buf_in - usart_read_buf_out;

	// on limite la copie si nécessaire
	if( s > size)
	{
		s = size;
	}
	else
	{
		size = s;
	}

	for( ; s-- ; )
	{
		*buf = usart_read_buf[usart_read_buf_out & (USART_READ_BUF_SIZE -1)];
		usart_read_buf_out++;
		buf++;
	}

	return size;
}
