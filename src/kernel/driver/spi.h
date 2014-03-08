#ifndef SPI_H
#define SPI_H

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

float spi_gyro_get_theta();

void spi_gyro_set_theta(float theta);

void spi_gyro_calib(int cmd);

#ifdef __cplusplus
}
#endif

#endif
