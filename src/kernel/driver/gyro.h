#ifndef GYRO_H
#define GYRO_H

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

struct gyro_cmd_set_position_arg
{
	float theta;
} __attribute__((packed));

float gyro_get_theta();

void gyro_set_theta(float theta);

void gyro_calib(enum GyroCalibrationCmd cmd);

#endif
