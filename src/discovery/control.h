#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/systick.h"
#include "kernel/control/kinematics.h"

//! période de la tache de controle en ms
#define CONTROL_PERIOD                             5
#define CONTROL_HZ          (1000.0f/CONTROL_PERIOD)

enum control_state
{
	CONTROL_READY_FREE = 0,       //!< no trajectory ongoing, control off
	CONTROL_READY_ASSER,          //!< no trajectory ongoing, control on
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
	CONTROL_IN_MOTION,            //!< trajectorie en cours
};

enum control_speed
{
	CONTROL_SPEED_OK,
	CONTROL_OVER_SPEED,
	CONTROL_UNDER_SPEED,
	CONTROL_WRONG_WAY,
};

enum trajectory_way
{
	TRAJECTORY_ANY_WAY,
	TRAJECTORY_FORWARD,
	TRAJECTORY_BACKWARD
};

enum control_type
{
	CONTROL_LINE_A,     //!< rotation sur place
	CONTROL_LINE_XY,    //!< aller a la position x,y en ligne droite (=> rotation puis avance)
	CONTROL_LINE_XYA,   //!< aller a la position x,y, alpha en ligne droite (=> rotation puis avance puis rotation)
};

struct control_usb_data
{
	struct systime current_time;
	int32_t control_state;
	int32_t control_cons_x;
	int32_t control_cons_y;
	int32_t control_cons_alpha;
	float control_pos_x;
	float control_pos_y;
	float control_pos_alpha;
	int32_t control_v_dist_cons;
	int32_t control_v_rot_cons;
	int32_t control_v_dist_mes;
	int32_t control_v_rot_mes;
	uint16_t control_i_right;
	uint16_t control_i_left;
	int16_t control_u_right;
	int16_t control_u_left;
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

#endif
