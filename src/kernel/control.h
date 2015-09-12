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
#include "kernel/motion/new_state/CMotionStateMachine.h"
#include "kernel/driver/encoder.h"
#include "kernel/driver/dynamixel.h"

//! période de la tache de controle en ms
#define CONTROL_PERIOD                            5
#define CONTROL_DT                            0.005f
#define CONTROL_HZ          (1000.0f/CONTROL_PERIOD)

struct control_usb_data_light
{
	struct systime current_time;
	int32_t motion_state;
	VectPlan cons;
	VectPlan pos;
	VectPlan wanted_pos;
	uint32_t gpio;
	uint8_t pumpState;
	uint8_t color;
	float vBat;
	uint32_t power_state;
	float elevatorHeight;
	struct dynamixel_usb_data dynamixel;
} __attribute__((packed));

struct control_usb_data
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
	struct dynamixel_usb_data dynamixel;

	float cons_motors_v[CAN_MOTOR_MAX];
	Kinematics mes_motors[CAN_MOTOR_MAX];
	float mes_motor_current[CAN_MOTOR_MAX];
	float iPwm[4];
	//	int16_t raw_data_gyro;
	//	float omega_gyro;
//	float pos_theta_gyro_euler;
//	float pos_theta_gyro_simpson;
	//MatrixHomogeneous arm_matrix;
} __attribute__((packed));

#endif
