#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/systick.h"
#include "kernel/control/kinematics.h"
#include "kernel/math/vect_plan.h"

//! période de la tache de controle en ms
#define CONTROL_PERIOD                             5
#define CONTROL_DT                            0.005f
#define CONTROL_HZ          (1000.0f/CONTROL_PERIOD)
#define EPSILON                                 1e-4

enum control_state
{
	CONTROL_READY_FREE = 0,       //!< no trajectory ongoing, control off
	CONTROL_READY_ASSER,          //!< no trajectory ongoing, control on
	CONTROL_SPEED,                //!< robot pilote en vitesse (mode manuel)
	CONTROL_ACTUATOR_SPEED,       //!< pilotage des vitesses des moteurs (debug)
	CONTROL_TRAJECTORY,           //!< trajectoire en cours
	CONTROL_BACK_TO_WALL,         //!< pas d'asservissement, les deux roues en marche arrière, pwm à x %. Arrêt quand le robot ne bouge plus
	CONTROL_END,                  //!< end : halted forever
};

enum control_status
{
	CONTROL_TARGET_REACHED = 0,   //!< cible atteinte
	CONTROL_TARGET_NOT_REACHED,   //!< cible non atteinte
	CONTROL_COLSISION,            //!< collision
	CONTROL_TIMEOUT,              //!< timeout
	CONTROL_PREPARING_MOTION,     //!< mise en place des tourelles
	CONTROL_IN_MOTION,            //!< trajectorie en cours
};

enum control_speed
{
	CONTROL_SPEED_OK,
	CONTROL_OVER_SPEED,
	CONTROL_UNDER_SPEED,
	CONTROL_WRONG_WAY,
};

// TODO
enum trajectory_way
{
	TRAJECTORY_ANY_WAY,
	TRAJECTORY_FORWARD,
	TRAJECTORY_BACKWARD
};

// TODO
enum control_type
{
	CONTROL_LINE_A,     //!< rotation sur place
	CONTROL_LINE_XY,    //!< aller a la position x,y en ligne droite (=> rotation puis avance)
	CONTROL_LINE_XYA,   //!< aller a la position x,y, alpha en ligne droite (=> rotation puis avance puis rotation)
};

void control_stop(bool asser);

//!< demande de trajectoire
void control_goto(VectPlan dest, VectPlan cp, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam);

//!< demande de vitesse
void control_set_cp_speed(VectPlan cp, VectPlan u, float v);

//!< demande de vitesse actionneur (debug)
void control_set_actuator_speed(float v[6]);

struct control_usb_data
{
	struct systime current_time;
	int32_t control_state;
	VectPlan cons;
	VectPlan pos;
	VectPlan wanted_pos;
	int16_t raw_data_gyro;
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

struct control_cmd_param_arg
{
	int32_t kp_av;
	int32_t ki_av;
	int32_t kd_av;
	int32_t kp_rot;
	int32_t ki_rot;
	int32_t kd_rot;
	int32_t kx;
	int32_t ky;
	int32_t kalpha;
}  __attribute__((packed));

struct control_cmd_max_speed_arg
{
	uint32_t vmax_av;
	uint32_t vmax_rot;
}  __attribute__((packed));

struct control_cmd_goto_arg
{
	VectPlan dest;
	VectPlan cp;
	KinematicsParameters linearParam;
	KinematicsParameters angularParam;
}  __attribute__((packed));

struct control_cmd_speed_arg
{
	VectPlan dest;
	VectPlan cp;
	KinematicsParameters linearParam;
	KinematicsParameters angularParam;
}  __attribute__((packed));

struct control_cmd_set_speed_arg
{
	VectPlan cp;
	VectPlan u;
	float v;
}  __attribute__((packed));

struct control_cmd_set_actuator_speed_arg
{
	float v[6];
}  __attribute__((packed));

#endif
