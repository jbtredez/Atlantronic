#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/systick.h"
#include "kernel/control/kinematics.h"
#include "kernel/math/VectPlan.h"
#include "kernel/math/matrix_homogeneous.h"
#include "middleware/motion/Motion.h"
#include "kernel/driver/encoder.h"
#include "kernel/driver/DynamixelManager.h"

//! période de la tache de controle en ms
#define CONTROL_PERIOD                           10
#define CONTROL_DT                            0.010f
#define CONTROL_HZ          (1000.0f/CONTROL_PERIOD)

struct control_usb_data_light
{
	struct systime current_time;
	int32_t motion_state;
	VectPlan cons;
	VectPlan pos;
	VectPlan wanted_pos;
	uint32_t gpio;
	uint16_t encoder[ENCODER_MAX];
	uint8_t pumpState;
	uint8_t color;
	float vBat;
	uint32_t power_state;
	float elevatorHeight;
	struct DynamixelUsbData ax12;
	//struct dynamixel_usb_data rx24;
} __attribute__((packed));

struct control_usb_data : control_usb_data_light
{
	float cons_motors_v[MOTION_MOTOR_MAX];
	Kinematics mes_motors[MOTION_MOTOR_MAX];
	float mes_motor_current[MOTION_MOTOR_MAX];
	float iPwm[4];
	//	int16_t raw_data_gyro;
	//	float omega_gyro;
//	float pos_theta_gyro_euler;
//	float pos_theta_gyro_simpson;
	//MatrixHomogeneous arm_matrix;
} __attribute__((packed));

#endif
