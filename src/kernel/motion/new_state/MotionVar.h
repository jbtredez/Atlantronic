

#ifndef MOTIONVAR_H_
#define MOTIONVAR_H_
#include "kernel/asm/asm_base_func.h"
#include "kernel/can_motor_mip.h"

#include "kernel/math/vect_plan.h"
enum
{
	MOTION_ENABLE_WANTED_UNKNOWN = -1,
	MOTION_ENABLE_WANTED_OFF = 0,
	MOTION_ENABLE_WANTED_ON = 1
};

enum motion_state
{
	MOTION_STATE_DISABLED = 0,         //!< pas de puissance sur les moteurs
	MOTION_STATE_TRY_ENABLE,           //!< mise en puissance des moteurs
	MOTION_STATE_ENABLED,              //!< moteurs avec puissance
	MOTION_STATE_ACTUATOR_KINEMATICS,  //!< pilotage des vitesses ou position des moteurs (debug)
	MOTION_STATE_SPEED,                //!< robot pilote en vitesse (mode manuel)
	MOTION_STATE_TRAJECTORY,           //!< trajectoire en cours
	MOTION_STATE_INTERRUPTING,         //!< arret en cours
//	MOTION_BACK_TO_WALL,         //!< pas d'asservissement, les deux roues en marche arrière, pwm à x %. Arrêt quand le robot ne bouge plus
	MOTION_NONE_STATE,
	MOTION_MAX_STATE,
};

enum motion_status
{
	MOTION_UPDATING_TRAJECTORY = 0,  //!< mise a jour de la trajectoire en cours
	MOTION_TARGET_REACHED,           //!< cible atteinte
	MOTION_TARGET_NOT_REACHED,       //!< cible non atteinte
	MOTION_COLISION,                //!< collision
	MOTION_TIMEOUT,                  //!< timeout
	MOTION_IN_MOTION,                //!< trajectorie en cours
};

enum motion_wanted_state
{
	MOTION_WANTED_STATE_UNKNOWN = 0,
	MOTION_WANTED_STATE_ACTUATOR_KINEMATICS,
	MOTION_WANTED_STATE_SPEED,
	MOTION_WANTED_STATE_TRAJECTORY,
};

#define MOTION_TARGET_REACHED_LIN_THRESHOLD_SQUARE          (10*10)
#define MOTION_TARGET_REACHED_ANG_THRESHOLD                  0.02f
#define MOTION_TARGET_NOT_REACHED_TIMEOUT                      300


enum motion_trajectory_step
{
	MOTION_TRAJECTORY_PRE_ROTATE = 0,
	MOTION_TRAJECTORY_STRAIGHT,
	MOTION_TRAJECTORY_ROTATE,
};


enum motion_way
{
	WAY_BACKWARD = -1,    //!< marche arriere
	WAY_ANY  = 0,         //!< marche avant ou marche arriere (selon le plus rapide)
	WAY_FORWARD  = 1,     //!< marche avant
};

enum motion_trajectory_type
{
	MOTION_AXIS_XYA = 0,   //!< aller a la position x,y, alpha en ligne droite (=> rotation puis avance puis rotation)
	MOTION_AXIS_A,         //!< rotation sur place
	MOTION_AXIS_XY,        //!< aller a la position x,y en ligne droite (=> rotation puis avance)
};

struct motion_goto_parameter
{
		VectPlan motion_wanted_dest;
		enum motion_way motion_wanted_way;
		enum motion_trajectory_type motion_wanted_trajectory_type;
		KinematicsParameters motion_wanted_linearParam;
		KinematicsParameters motion_wanted_angularParam;
};


struct motion_cmd_param_arg
{
	float kp_av;
	float ki_av;
	float kd_av;
	float kp_rot;
	float ki_rot;
	float kd_rot;
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
	int8_t way;
	int8_t type;
	KinematicsParameters linearParam;
	KinematicsParameters angularParam;
}  __attribute__((packed));

struct motion_cmd_set_speed_arg
{
	VectPlan u;
	float v;
}  __attribute__((packed));

struct motion_cmd_set_actuator_kinematics_arg
{
	int mode[CAN_MOTOR_MAX];
	float val[CAN_MOTOR_MAX];
}  __attribute__((packed));

struct motion_cmd_enable_arg
{
	uint8_t enable;
}  __attribute__((packed));

struct motion_cmd_set_max_driving_current_arg
{
	float maxDrivingCurrent;
}  __attribute__((packed));

#endif
