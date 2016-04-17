#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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
	SPI_DEVICE_ESP8266 = (SPI_DRIVER_6 << 16),
	SPI_DEVICE_UNUSED2_SPI6,
};

int spi_transaction(enum spi_device device, const void* tx_buffer, void* rx_buffer, uint16_t size);

int spi_write(enum spi_device device, const void* tx_buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
