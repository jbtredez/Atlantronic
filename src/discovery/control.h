#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/systick.h"
#include "kernel/control/kinematics.h"
#include "kernel/math/vect_plan.h"
#include "motion.h"
#include "kernel/driver/encoder.h"

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
	float omega_gyro;
	float pos_theta_gyro_euler;
	float pos_theta_gyro_simpson;
	float cons_v1;
	float cons_v2;
	float cons_v3;
	float cons_theta1;
	float cons_theta2;
	float cons_theta3;
	float mes_v1;
	float mes_v2;
	float mes_v3;
	float mes_theta1;
	float mes_theta2;
	float mes_theta3;
	float vBat;
	float iPwm[4];
} __attribute__((packed));

#endif
