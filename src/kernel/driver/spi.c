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

#define SPI_DEVICE_MAX        3

typedef struct
{
	GPIO_TypeDef* gpio_cs;
	uint32_t pin_cs;
} SpiDevice;

typedef struct
{
	SPI_TypeDef* reg;
	DMA_Stream_TypeDef* txDma;
	DMA_Stream_TypeDef* rxDma;
	SpiDevice devices[SPI_DEVICE_MAX];
	xSemaphoreHandle sem;
	xSemaphoreHandle mutex;
} SpiDriver;

static SpiDriver spi_driver[SPI_DRIVER_MAX];

static void spi_register_device(const enum spi_device id, GPIO_TypeDef* gpio_cs, uint32_t pin_cs);
static void spi_driver_init(const enum spi_driver id, SPI_TypeDef* spi_reg, GPIO_TypeDef* gpio_sck, uint32_t pin_sck, GPIO_TypeDef* gpio_miso, uint32_t pin_miso, GPIO_TypeDef* gpio_mosi, uint32_t pin_mosi, uint32_t gpio_af, DMA_Stream_TypeDef* txDma, int txChan, DMA_Stream_TypeDef* rxDma, int rxChan);

int spi_module_init()
{
	// activation du SPI5 et SPI6
	RCC->APB2ENR |= RCC_APB2ENR_SPI5EN /*| RCC_APB2ENR_SPI6EN*/;

	// activation GPIOC (CS du gyro et CS du lcd), GPIOF (SCK, MOSI et MISO sur le port F) et dma2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_DMA2EN;

	// reset SPI5 et SPI6
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI5RST | RCC_APB2RSTR_SPI6RST;
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI5RST | RCC_APB2RSTR_SPI6RST);

	spi_register_device(SPI_DEVICE_GYRO, GPIOC, 1);
	spi_register_device(SPI_DEVICE_LCD, GPIOC, 2);
	//spi_register_device(SPI_DEVICE_UNUSED_SPI5, GPIOF, 10);
	spi_driver_init(SPI_DRIVER_5, SPI5, GPIOF, 7, GPIOF, 8, GPIOF, 9, GPIO_AF_SPI5, DMA2_Stream4, 2, DMA2_Stream3, 2);
	spi_register_device(SPI_DEVICE_ESP8266, GPIOB, 3);
	//spi_register_device(SPI_DEVICE_UNUSED2_SPI6, GPIOE, 2);
	spi_driver_init(SPI_DRIVER_6, SPI6, GPIOG, 13, GPIOG, 12, GPIOG, 14, GPIO_AF_SPI6, DMA2_Stream5, 1, DMA2_Stream6, 1);

	NVIC_EnableIRQ(SPI5_IRQn);
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	NVIC_SetPriority(SPI5_IRQn, PRIORITY_IRQ_SPI);
	NVIC_SetPriority(DMA2_Stream3_IRQn, PRIORITY_IRQ_DMA2_STREAM3);
	NVIC_SetPriority(DMA2_Stream4_IRQn, PRIORITY_IRQ_DMA2_STREAM4);

	NVIC_EnableIRQ(SPI6_IRQn);
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
	NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	NVIC_SetPriority(SPI6_IRQn, PRIORITY_IRQ_SPI);
	NVIC_SetPriority(DMA2_Stream5_IRQn, PRIORITY_IRQ_DMA2_STREAM5);
	NVIC_SetPriority(DMA2_Stream6_IRQn, PRIORITY_IRQ_DMA2_STREAM6);

	return 0;
}

module_init(spi_module_init, INIT_SPI);

static void spi_driver_init(const enum spi_driver id, SPI_TypeDef* spi_reg, GPIO_TypeDef* gpio_sck, uint32_t pin_sck, GPIO_TypeDef* gpio_miso, uint32_t pin_miso, GPIO_TypeDef* gpio_mosi, uint32_t pin_mosi, uint32_t gpio_af, DMA_Stream_TypeDef* txDma, int txChan, DMA_Stream_TypeDef* rxDma, int rxChan)
{
	if( id >= SPI_DRIVER_MAX )
	{
		return;
	}

	gpio_pin_init(gpio_sck, pin_sck, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // SCK
	gpio_pin_init(gpio_miso, pin_miso, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // MISO
	gpio_pin_init(gpio_mosi, pin_mosi, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // MOSI
	gpio_af_config(gpio_sck, pin_sck, gpio_af);
	gpio_af_config(gpio_miso, pin_miso, gpio_af);
	gpio_af_config(gpio_mosi, pin_mosi, gpio_af);
	spi_driver[id].reg = spi_reg;
	spi_driver[id].txDma = txDma;
	spi_driver[id].rxDma = rxDma;
	vSemaphoreCreateBinary(spi_driver[id].sem);
	xSemaphoreTake(spi_driver[id].sem, 0);
	spi_driver[id].mutex = xSemaphoreCreateMutex();

	int i;
	for(i = 0; i < SPI_DEVICE_MAX; i++)
	{
		SpiDevice* dev = &spi_driver[id].devices[i];
		if( dev->gpio_cs )
		{
			// init pin cs
			gpio_pin_init(dev->gpio_cs, dev->pin_cs, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN);

			// on ne selectionne rien
			gpio_set_pin(dev->gpio_cs, dev->pin_cs);
		}
	}

#if( RCC_PCLK2_MHZ != 96)
#error revoir calcul frequence spi
#endif
	// spi full duplex, lignes dediees a sens unique
	// datasize = 8
	// clock polarity : low
	// clock phase : edge
	// selection esclave (NSS) : soft (=> SSM = 1 et SSI = 1)
	// prescaler : 16 (SPI_CR1_BR_0 | SPI_CR1_BR_1) => PCLk2/16 = 6 MHz
	// note : freq max :
	// - adxrs453 : 8MHz
	// - l3gd20   : 10MHz
	// - ili9341  : 6.66Mhz lecture, 10Mhz ecriture
	// format MSBFIRST
	// spi maitre
	spi_reg->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;

	// activation du mode SPI
	spi_reg->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

	// crc polynomial
	spi_reg->CRCPR = 7;

	// enable SPI
	spi_reg->CR1 |= SPI_CR1_SPE;

	// activation DMA sur SPI5 et de l'IT d'erreur
	spi_reg->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;

	// taille mémoire d'une donnée : 8 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 8 bits
	// incrément automatique mémoire périph : 0
	// transfert (écriture) : mem => mem périph
	// transfert (lecture) : mem périph => mem
	rxDma->CR = (rxChan * DMA_SxCR_CHSEL_0) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	rxDma->PAR = (uint32_t) &spi_reg->DR;
	txDma->CR = (txChan * DMA_SxCR_CHSEL_0) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
	txDma->PAR = (uint32_t) &spi_reg->DR;
}

static void spi_register_device(const enum spi_device id, GPIO_TypeDef* gpio_cs, uint32_t pin_cs)
{
	int driverId = id >> 16;
	int deviceId = id & 0xffff;

	if( driverId >= SPI_DRIVER_MAX || deviceId >= SPI_DEVICE_MAX)
	{
		return;
	}

	spi_driver[driverId].devices[deviceId].gpio_cs = gpio_cs;
	spi_driver[driverId].devices[deviceId].pin_cs = pin_cs;
}

void isr_dma2_stream3()
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( DMA2->LISR | DMA_LISR_TCIF3)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;
		xSemaphoreGiveFromISR(spi_driver[SPI_DRIVER_5].sem, &xHigherPriorityTaskWoken);

	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_dma2_stream4()
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( DMA2->HISR | DMA_HISR_TCIF4)
	{
		DMA2->HIFCR |= DMA_HIFCR_CTCIF4;
		DMA2_Stream4->CR &= ~DMA_SxCR_EN;

		if( SPI5->CR1 & SPI_CR1_BIDIOE )
		{
			xSemaphoreGiveFromISR(spi_driver[SPI_DRIVER_5].sem, &xHigherPriorityTaskWoken);
		}
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_dma2_stream5()
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( DMA2->HISR | DMA_HISR_TCIF5)
	{
		DMA2->HIFCR |= DMA_HIFCR_CTCIF5;
		DMA2_Stream4->CR &= ~DMA_SxCR_EN;

		if( SPI5->CR1 & SPI_CR1_BIDIOE )
		{
			xSemaphoreGiveFromISR(spi_driver[SPI_DRIVER_6].sem, &xHigherPriorityTaskWoken);
		}
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_dma2_stream6()
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( DMA2->HISR | DMA_HISR_TCIF6)
	{
		DMA2->HIFCR |= DMA_HIFCR_CTCIF6;
		DMA2_Stream4->CR &= ~DMA_SxCR_EN;

		if( SPI5->CR1 & SPI_CR1_BIDIOE )
		{
			xSemaphoreGiveFromISR(spi_driver[SPI_DRIVER_6].sem, &xHigherPriorityTaskWoken);
		}
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_spi5(void)
{
	int status = SPI5->SR;
	if( status & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR) )
	{
		// TODO : remonter erreur + log erreur

		// erreur MODF qui ne doit pas arriver (NSS soft et SSI = 1)
		// erreur CRC qui ne doit pas arriver (pas active)

		// on desactive le DMA de reception
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;

		// clear overrun : lecture DR puis lecture SR
		SPI5->DR;
		SPI5->SR;
	}
}

void isr_spi6(void)
{
	int status = SPI6->SR;
	if( status & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR) )
	{
		// TODO : remonter erreur + log erreur

		// erreur MODF qui ne doit pas arriver (NSS soft et SSI = 1)
		// erreur CRC qui ne doit pas arriver (pas active)

		// on desactive le DMA de reception
		DMA2_Stream5->CR &= ~DMA_SxCR_EN;

		// clear overrun : lecture DR puis lecture SR
		SPI6->DR;
		SPI6->SR;
	}
}

//! @return -1 si timeout ou erreur
//! @return 0 sinon
int spi_transaction(enum spi_device id, const void* tx_buffer, void* rx_buffer, uint16_t size)
{
	int driverId = id >> 16;
	int deviceId = id & 0xffff;
	int res = 0;

	if( driverId >= SPI_DRIVER_MAX || deviceId >= SPI_DEVICE_MAX)
	{
		return -1;
	}

	SpiDriver* drv = &spi_driver[driverId];
	SpiDevice* dev = &drv->devices[deviceId];
	xSemaphoreTake(drv->mutex, portMAX_DELAY);

	gpio_reset_pin(dev->gpio_cs, dev->pin_cs);
	xSemaphoreTake(drv->sem, 0);
	if( rx_buffer )
	{
		drv->reg->CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);
		drv->rxDma->M0AR = (uint32_t) rx_buffer;
		drv->rxDma->NDTR = size;
		drv->rxDma->CR |= DMA_SxCR_EN;
	}
	else
	{
		drv->reg->CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE;
	}
	drv->txDma->M0AR = (uint32_t) tx_buffer;
	drv->txDma->NDTR = size;
	drv->txDma->CR |= DMA_SxCR_EN;
	if( xSemaphoreTake(drv->sem, 2) == pdFALSE )
	{
		res = -1;
	}
	gpio_set_pin(dev->gpio_cs, dev->pin_cs);

	xSemaphoreGive(drv->mutex);

	return res;
}

//! @return -1 si timeout
//! @return 0 sinon
int spi_write(enum spi_device device, const void* tx_buffer, uint16_t size)
{
	return spi_transaction(device, tx_buffer, 0, size);
}
