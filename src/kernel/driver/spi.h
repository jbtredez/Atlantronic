#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if defined(__discovery__)
enum spi_driver
{
	SPI_DRIVER_1 = 0,
	SPI_DRIVER_MAX,
};

enum spi_device
{
	SPI_DEVICE_ACCELERO = (SPI_DRIVER_1 << 16),
	SPI_DEVICE_GYRO,
	SPI_DEVICE_UNUSED_SPI1,
};
#elif defined(__disco__)
enum spi_driver
{
	SPI_DRIVER_5 = 0,
	SPI_DRIVER_6,
	SPI_DRIVER_MAX,
};

enum spi_device
{
	SPI_DEVICE_GYRO = (SPI_DRIVER_5 << 16),
	SPI_DEVICE_LCD,
	SPI_DEVICE_UNUSED_SPI5,
	SPI_DEVICE_UNUSED1_SPI6 = (SPI_DRIVER_6 << 16),
	SPI_DEVICE_UNUSED2_SPI6,
};
#else
#error unknown card
#endif

int spi_transaction(enum spi_device device, const void* tx_buffer, void* rx_buffer, uint16_t size);

int spi_write(enum spi_device device, const void* tx_buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
