#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/systick.h"
#include "kernel/control/kinematics.h"
#include "kernel/math/vect_plan.h"
#include "kernel/math/matrix_homogeneous.h"
#include "motion.h"
#include "kernel/driver/encoder.h"
#include "kernel/driver/dynamixel.h"

//! période de la tache de controle en ms
#define CONTROL_PERIOD                            10
#define CONTROL_DT                            0.010f
#define CONTROL_HZ          (1000.0f/CONTROL_PERIOD)
#define EPSILON                                 1e-4

struct control_usb_data
{
	struct systime current_time;
	int32_t motion_state;
	VectPlan cons;
	VectPlan pos;
	VectPlan wanted_pos;
	int16_t raw_data_gyro;
	uint16_t encoder[ENCODER_MAX];
	uint32_t gpio;
	float omega_gyro;
	float pos_theta_gyro_euler;
	float pos_theta_gyro_simpson;
	float cons_v1;
	float cons_v2;
	float cons_v3;
	float cons_theta1;
	float cons_theta2;
	float cons_theta3;
	Kinematics mes_motors[CAN_MOTOR_MAX];
	float mes_motor_current[CAN_MOTOR_MAX];
	float vBat;
	float iPwm[4];
	bool homingDone;
	uint8_t pumpState;
	uint32_t power_state;
	struct dynamixel_usb_data dynamixel;
	MatrixHomogeneous arm_matrix;
} __attribute__((packed));

#endif
