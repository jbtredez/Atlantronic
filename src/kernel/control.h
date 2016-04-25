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
#include "kernel/driver/encoder/EncoderAB.h"
#include "kernel/driver/DynamixelManager.h"

//! période de la tache de controle en ms
#define CONTROL_PERIOD                           10
#define CONTROL_DT                            0.010f

struct control_usb_data_light
{
	struct systime current_time; //2*4
	int32_t motion_state; //4
	VectPlan cons;// 4*3
	VectPlan pos;;// 4*3
	VectPlan wanted_pos;;// 4*3
	uint16_t gpio;//2
	uint16_t encoder[ENCODER_MAX];  //Seulement 2 encoders utilisés//3 *2
	uint8_t pumpState; //1
	uint8_t color;  //1
	float vBat;  //4
	uint8_t power_state; //1
	float elevatorHeight;//4
	struct DynamixelUsbData ax12;//7*9 = 63
	//struct dynamixel_usb_data rx24;
} __attribute__((packed));

struct control_usb_data : control_usb_data_light
{
	float cons_motors_v[MOTION_MOTOR_MAX];  //2*4
	Kinematics mes_motors[MOTION_MOTOR_MAX];
	float mes_motor_current[MOTION_MOTOR_MAX];// 2*4
	float iPwm[4];//4*4
	//	int16_t raw_data_gyro;
	//	float omega_gyro;
//	float pos_theta_gyro_euler;
//	float pos_theta_gyro_simpson;
	//MatrixHomogeneous arm_matrix;
} __attribute__((packed));

#endif
