#include "spi.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "gpio.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/fault.h"
#include <math.h>

#define SPI_STACK_SIZE             300

static xSemaphoreHandle spi_sem;
static xSemaphoreHandle tim_sem;
static void(*spi_callback[SPI_DEVICE_MAX])(void);

static void spi_task(void* arg);

int spi_module_init()
{
	// activation du SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// activation dma2
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// activation GPIOA (SCK, MOSI et MISO sur le port A)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// activation GPIOE (CS de l'accelero)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	gpio_pin_init(GPIOA, 5, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // SCK
	gpio_pin_init(GPIOA, 6, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // MISO
	gpio_pin_init(GPIOA, 7, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // MOSI
	gpio_pin_init(GPIOE, 3, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // CS0 accelero
	gpio_pin_init(GPIOE, 7, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // CS1 gyro
//	gpio_pin_init(GPIOA, 8, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // CS2

	// on ne selectionne rien
	gpio_set_pin(GPIOE, 3);
	gpio_set_pin(GPIOE, 7);
//	gpio_set_pin(GPIOA, 8);

	gpio_af_config(GPIOA, 5, GPIO_AF_SPI1);
	gpio_af_config(GPIOA, 6, GPIO_AF_SPI1);
	gpio_af_config(GPIOA, 7, GPIO_AF_SPI1);


	// reset SPI1
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1;

	// spi full duplex, lignes dediees a sens unique
	// datasize = 8
	// clock polarity : low
	// clock phase : edge
	// selection esclave (NSS) : soft (=> SSM = 1 et SSI = 1)
	// prescaler : 16 (SPI_CR1_BR_0 | SPI_CR1_BR_1) => PCLk2/16 = 5.25 MHz
	// format LSBFIRST
	// spi maitre
	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;

	// activation du mode SPI
	SPI1->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

	// crc polynomial
	SPI1->CRCPR = 7;

	// enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;

	// activation DMA sur SPI1 et de l'IT d'erreur
	SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;

	// taille mémoire d'une donnée : 8 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 8 bits
	// incrément automatique mémoire périph : 0
	// transfert (écriture) : mem => mem périph
	// transfert (lecture) : mem périph => mem
	// SPI1_RX : DMA2, stream0 sur chan3
	// SPI1_TX : DMA2, stream3 sur chan3
	DMA2_Stream0->CR = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA2_Stream0->PAR = (uint32_t) &SPI1->DR;
	DMA2_Stream3->CR = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
	DMA2_Stream3->PAR = (uint32_t) &SPI1->DR;

	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	NVIC_SetPriority(SPI1_IRQn, PRIORITY_IRQ_SPI);
	NVIC_SetPriority(DMA2_Stream0_IRQn, PRIORITY_IRQ_DMA2_STREAM0);
	NVIC_SetPriority(DMA2_Stream3_IRQn, PRIORITY_IRQ_DMA2_STREAM3);

	vSemaphoreCreateBinary(spi_sem);
	xSemaphoreTake(spi_sem, 0);

	int i = 0;
	for(i = 0; i < SPI_DEVICE_MAX; i++)
	{
		spi_callback[i] = nop_function;
	}

	xTaskCreate(spi_task, "spi", SPI_STACK_SIZE, NULL, PRIORITY_TASK_SPI, NULL);

	//////////// TESTS SPI sur timer
	// activation timer 6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->CR1 = 0x00;//TIM_CR1_ARPE;

	// TIM6_CLK = 84MHz / (PSC + 1) = 21Mhz
	TIM6->PSC = 3;

	// IT tout les TIM6_CLK / ARR = 485 Hz
	TIM6->ARR = 43299;
	TIM6->DIER = TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_SetPriority(TIM6_DAC_IRQn, PRIORITY_IRQ_SPI);

	vSemaphoreCreateBinary(tim_sem);
	xSemaphoreTake(tim_sem, 0);

	// activation
	TIM6->CR1 |= TIM_CR1_CEN;

	return 0;
}

module_init(spi_module_init, INIT_SPI);

int spi_register_callback(enum spi_device device, void(*callback)(void))
{
	int res = -1;

	if( device < SPI_DEVICE_MAX)
	{
		spi_callback[device] = callback;
		res = 0;
	}

	return res;
}

void isr_dma2_stream0()
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( DMA2->LISR | DMA_LISR_TCIF0)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
		xSemaphoreGiveFromISR(spi_sem, &xHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_dma2_stream3()
{
	if( DMA2->LISR | DMA_LISR_TCIF3)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	}
}

void isr_spi1(void)
{
	int status = SPI1->SR;
	if( status & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR) )
	{
		// TODO : remonter erreur + log erreur

		// erreur MODF qui ne doit pas arriver (NSS soft et SSI = 1)
		// erreur CRC qui ne doit pas arriver (pas active)

		// on desactive le DMA de reception
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;

		// clear overrun : lecture DR puis lecture SR
		SPI1->DR;
		SPI1->SR;
	}
}

static void spi_set_read_dma_buffer(unsigned char* buf)
{
	DMA2_Stream0->M0AR = (uint32_t) buf;
}

static void spi_set_read_dma_size(uint16_t size)
{
	DMA2_Stream0->NDTR = size;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
}

static void spi_set_write_dma_buffer(unsigned char* buf)
{
	DMA2_Stream3->M0AR = (uint32_t) buf;
}

static void spi_send_dma_buffer(uint16_t size)
{
	DMA2_Stream3->NDTR = size;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
}

//! @return -1 si timeout
//! @return 0 sinon
int spi_transaction(enum spi_device device, uint8_t* tx_buffer, uint8_t* rx_buffer, uint8_t size)
{
	int res = 0;
	GPIO_TypeDef* gpio = NULL;
	uint32_t pin;

	switch(device)
	{
		case SPI_DEVICE_ACCELERO:
			gpio = GPIOE;
			pin = 3;
			break;
		case SPI_DEVICE_GYRO:
			gpio = GPIOE;
			pin = 7;
			break;
		case SPI_DEVICE_UNUSED:
			gpio = GPIOA;
			pin = 8;
			break;
		case SPI_DEVICE_MAX:
		default:
			return -1;
			break;
	}

	gpio_reset_pin(gpio, pin);
	xSemaphoreTake(spi_sem, 0);
	spi_set_read_dma_buffer(rx_buffer);
	spi_set_read_dma_size(size);
	spi_set_write_dma_buffer(tx_buffer);
	spi_send_dma_buffer(size);
	if( xSemaphoreTake(spi_sem, 2) == pdFALSE )
	{
		res = -1;
	}
	gpio_set_pin(gpio, pin);

	return res;
}

void isr_tim6(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( TIM6->SR | TIM_SR_UIF )
	{
		TIM6->SR &= ~TIM_SR_UIF;
		xSemaphoreGiveFromISR(tim_sem, &xHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

static void spi_task(void* arg)
{
	(void) arg;
	int i = 0;

	while(1)
	{
		for(i = 0; i < SPI_DEVICE_MAX; i++)
		{
			spi_callback[i]();
		}

		xSemaphoreTake(tim_sem, portMAX_DELAY);
		//vTaskDelay(1);
	}
}

