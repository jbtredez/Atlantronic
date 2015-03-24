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
#include "kernel/motion/motion.h"
#include "kernel/driver/encoder.h"
#include "kernel/driver/dynamixel.h"

//! période de la tache de controle en ms
#define CONTROL_PERIOD                             5
#define CONTROL_DT                            0.005f
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
	float cons_motors_v[CAN_MOTOR_MAX];
	Kinematics mes_motors[CAN_MOTOR_MAX];
	float mes_motor_current[CAN_MOTOR_MAX];
	float vBat;
	float iPwm[4];
	uint8_t pumpState;
	uint8_t color;
	uint32_t power_state;
	float elevatorHeight;
	struct dynamixel_usb_data dynamixel;
	//MatrixHomogeneous arm_matrix;
} __attribute__((packed));

#endif
