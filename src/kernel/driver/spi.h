#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum spi_device
{
	SPI_DEVICE_ACCELERO = 0,
	SPI_DEVICE_GYRO,
	SPI_DEVICE_UNUSED,
	SPI_DEVICE_MAX,
};

int spi_transaction(enum spi_device device, uint8_t* tx_buffer, uint8_t* rx_buffer, uint8_t size);

int spi_register_callback(enum spi_device device, void(*callback)(void));

#ifdef __cplusplus
}
#endif

#endif
