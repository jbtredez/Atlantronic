#include "spi.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/log.h"
#include "gpio.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"

#define SPI_STACK_SIZE             100

// interface accelero
#define LIS302DL_READ_CMD             0x80
#define LIS302DL_MULTIPLEBYTE_CMD     0x40
#define LIS302DL_CTRL_REG1_ADDR       0x20
#define LIS302DL_OUT_X_ADDR           0x29
#define LIS302DL_OUT_Y_ADDR           0x2B
#define LIS302DL_OUT_Z_ADDR           0x2D
#define LIS302DL_DATARATE_400         0x80
#define LIS302DL_LOWPOWERMODE_ACTIVE  0x40
#define LIS302DL_XYZ_ENABLE           0x07

struct spi_accel_dma_xyz
{
	unsigned char reserved0;
	signed char x;
	unsigned char reserved1;
	signed char y;
	unsigned char reserved2;
	signed char z;
} __attribute__((packed));


static unsigned char spi_tx_buffer[16];
static unsigned char spi_rx_buffer[16];
static struct spi_accel_dma_xyz spi_accel_dma_xyz;

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
	gpio_pin_init(GPIOE, 3, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // CS accelero

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

	xTaskCreate(spi_task, "spi", SPI_STACK_SIZE, NULL, PRIORITY_TASK_SPI, NULL);

	return 0;
}

module_init(spi_module_init, INIT_SPI);

void isr_dma2_stream0()
{
	if( DMA2->LISR | DMA_LISR_TCIF0)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	}
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

static void spi_write(uint8_t* buffer, uint8_t size)
{
	spi_set_write_dma_buffer(buffer);
	spi_send_dma_buffer(size);
}

static void spi_init_accelero()
{
	spi_set_read_dma_buffer(spi_rx_buffer);
	spi_set_read_dma_size(2);

	spi_tx_buffer[0] = LIS302DL_CTRL_REG1_ADDR;
	spi_tx_buffer[1] = LIS302DL_DATARATE_400 | LIS302DL_LOWPOWERMODE_ACTIVE | LIS302DL_XYZ_ENABLE;
	gpio_reset_pin(GPIOE, 3);
	spi_write(spi_tx_buffer, 2);
	vTaskDelay(1);
	gpio_set_pin(GPIOE, 3);
}

static void spi_task(void* arg)
{
	(void) arg;

	spi_init_accelero();

	while(1)
	{
		spi_set_read_dma_buffer((void*)&spi_accel_dma_xyz);
		spi_set_read_dma_size(6);
		spi_tx_buffer[0] = LIS302DL_OUT_X_ADDR | LIS302DL_READ_CMD | LIS302DL_MULTIPLEBYTE_CMD;
		gpio_reset_pin(GPIOE, 3);
		spi_write(spi_tx_buffer, 6);
		vTaskDelay(1);
		gpio_set_pin(GPIOE, 3);
	}
}
