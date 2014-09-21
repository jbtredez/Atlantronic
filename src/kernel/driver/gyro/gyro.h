#ifndef GYRO_H
#define GYRO_H

#ifndef WEAK_GYRO
#define WEAK_GYRO __attribute__((weak, alias("nop_function") ))
#endif

#include <stdint.h>

enum GyroState
{
	GYRO_STATE_DISCONNECTED = 0,
	GYRO_STATE_RUNNING,
};

enum GyroCalibrationCmd
{
	GYRO_CALIBRATION_START,
	GYRO_CALIBRATION_STOP,
};

struct gyro_cmd_calibration_arg
{
	int32_t calib_cmd;
} __attribute__((packed));

struct gyro_cmd_set_position_arg
{
	float theta;
} __attribute__((packed));

struct gyro_cmd_set_calibration_values_arg
{
	float scale;
	float bias;
	float dead_zone;
} __attribute__((packed));


int16_t gyro_get_raw_data() WEAK_GYRO;
float gyro_get_omega() WEAK_GYRO;
float gyro_get_theta_euler() WEAK_GYRO;
float gyro_get_theta_simpson() WEAK_GYRO;

void gyro_set_theta(float theta);

void gyro_calib(enum GyroCalibrationCmd cmd);

#endif
