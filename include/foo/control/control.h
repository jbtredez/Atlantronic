#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

enum control_state
{
	CONTROL_READY_ASSER,          //!< no trajectory ongoing, control on
	CONTROL_READY_FREE,           //!< no trajectory ongoing, control off
	CONTROL_TRAJECTORY,           //!< trajectoire en cours
	CONTROL_END,                  //!< end : halted forever
};

enum control_speed
{
	CONTROL_SPEED_OK,
	CONTROL_OVER_SPEED,
	CONTROL_UNDER_SPEED,
	CONTROL_WRONG_WAY,
};

struct control_usb_data
{
	int32_t control_state;
	float control_dest_x;
	float control_dest_y;
	float control_dest_alpha;
	float control_cons_x;
	float control_cons_y;
	float control_cons_alpha;
	float control_pos_x;
	float control_pos_y;
	float control_pos_alpha;
	float control_v_dist_cons;
	float control_v_rot_cons;
	float control_v_dist_mes;
	float control_v_rot_mes;
	uint16_t control_i_right;
	uint16_t control_i_left;
} __attribute__((packed));

struct control_cmd_param_arg
{
	float kp_av;
	float ki_av;
	float kd_av;
	float kp_rot;
	float ki_rot;
	float kd_rot;
	float kx;
	float ky;
	float kalpha;
};

enum trajectory_way
{
	TRAJECTORY_ANY_WAY,
	TRAJECTORY_FORWARD,
	TRAJECTORY_BACKWARD
};

void control_goto_near(float x, float y, float alpha, float dist, enum trajectory_way sens);

void control_free();

float control_find_rotate(float debut, float fin);

int32_t control_get_state();

#endif
