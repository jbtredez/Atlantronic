#ifndef SPI_H
#define SPI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum spi_calibration_cmd
{
	GYRO_CALIBRATION_START,
	GYRO_CALIBRATION_STOP,
};

struct spi_gyro_cmd_set_position_arg
{
	float theta;
} __attribute__((packed));

int32_t spi_gyro_get_raw_data();
float spi_gyro_get_omega();
float spi_gyro_get_theta_euler();
float spi_gyro_get_theta_simpson();

void spi_gyro_set_theta(float theta);

void spi_gyro_calib(int cmd);

#ifdef __cplusplus
}
#endif

#endif
