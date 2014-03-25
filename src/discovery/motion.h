#ifndef MOTION_H
#define MOTION_H

//! @file motion.h
//! @brief Gestion du deplacement du robot (asservissement)
//! @author Atlantronic

#include <stdint.h>
#include "kernel/systick.h"
#include "kernel/asm/asm_base_func.h"

#ifndef WEAK_MOTION
#define WEAK_MOTION __attribute__((weak, alias("nop_function") ))
#endif

enum motion_state
{
	MOTION_READY_FREE = 0,       //!< no trajectory ongoing, control off
	MOTION_READY_ASSER,          //!< no trajectory ongoing, control on
	MOTION_SPEED,                //!< robot pilote en vitesse (mode manuel)
	MOTION_ACTUATOR_SPEED,       //!< pilotage des vitesses des moteurs (debug)
	MOTION_TRAJECTORY,           //!< trajectoire en cours
	MOTION_BACK_TO_WALL,         //!< pas d'asservissement, les deux roues en marche arrière, pwm à x %. Arrêt quand le robot ne bouge plus
	MOTION_END,                  //!< end : halted forever
};

enum motion_status
{
	MOTION_TARGET_REACHED = 0,   //!< cible atteinte
	MOTION_TARGET_NOT_REACHED,   //!< cible non atteinte
	MOTION_COLSISION,            //!< collision
	MOTION_TIMEOUT,              //!< timeout
	MOTION_PREPARING_MOTION,     //!< mise en place des tourelles
	MOTION_IN_MOTION,            //!< trajectorie en cours
};

enum motion_speed
{
	MOTION_SPEED_OK,
	MOTION_OVER_SPEED,
	MOTION_UNDER_SPEED,
	MOTION_WRONG_WAY,
};

// TODO
enum trajectory_way
{
	TRAJECTORY_ANY_WAY,
	TRAJECTORY_FORWARD,
	TRAJECTORY_BACKWARD
};

// TODO
enum motion_type
{
	MOTION_LINE_A,     //!< rotation sur place
	MOTION_LINE_XY,    //!< aller a la position x,y en ligne droite (=> rotation puis avance)
	MOTION_LINE_XYA,   //!< aller a la position x,y, alpha en ligne droite (=> rotation puis avance puis rotation)
};

void motion_stop(bool asser);

//!< demande de trajectoire
void motion_goto(VectPlan dest, VectPlan cp, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam);

//!< demande de vitesse
void motion_set_cp_speed(VectPlan cp, VectPlan u, float v);

//!< demande de vitesse actionneur (debug)
void motion_set_actuator_speed(float v[6]);

//!< mise a jour de l'asservissement
void motion_compute() WEAK_MOTION;

void motion_update_usb_data(struct control_usb_data* data) WEAK_MOTION;

struct motion_cmd_param_arg
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

struct motion_cmd_max_speed_arg
{
	uint32_t vmax_av;
	uint32_t vmax_rot;
}  __attribute__((packed));

struct motion_cmd_goto_arg
{
	VectPlan dest;
	VectPlan cp;
	KinematicsParameters linearParam;
	KinematicsParameters angularParam;
}  __attribute__((packed));

struct motion_cmd_speed_arg
{
	VectPlan dest;
	VectPlan cp;
	KinematicsParameters linearParam;
	KinematicsParameters angularParam;
}  __attribute__((packed));

struct motion_cmd_set_speed_arg
{
	VectPlan cp;
	VectPlan u;
	float v;
}  __attribute__((packed));

struct motion_cmd_set_actuator_speed_arg
{
	float v[6];
}  __attribute__((packed));

#endif
