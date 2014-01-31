#ifndef SPI_H
#define SPI_H

#ifdef __cplusplus
extern "C" {
#endif

enum spi_calibration_cmd
{
	GYRO_CALIBRATION_RESET,
	GYRO_CALIBRATION_START,
	GYRO_CALIBRATION_STOP,
};

float spi_gyro_get_theta();

void spi_gyro_calib(int cmd);

#ifdef __cplusplus
}
#endif

#endif
