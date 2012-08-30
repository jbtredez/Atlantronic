#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

//! période de la tache de propulsion en tick ("fréquence" de l'asservissement)
#define CONTROL_TICK_PERIOD        ms_to_tick(5)
#define CONTROL_HZ                 200
#define CONTROL_TE                 0.005f


enum control_state
{
	CONTROL_READY_ASSER,          //!< no trajectory ongoing, control on
	CONTROL_READY_FREE,           //!< no trajectory ongoing, control off
	CONTROL_TRAJECTORY,           //!< trajectoire en cours
	CONTROL_BACK_TO_WALL,         //!< pas d'asservissement, les deux roues en marche arrière, pwm à x %. Arrêt quand le robot ne bouge plus
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
	int32_t control_cons_x;
	int32_t control_cons_y;
	int32_t control_cons_alpha;
	int32_t control_pos_x;
	int32_t control_pos_y;
	int32_t control_pos_alpha;
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

//!< deplacement vers la position demandée en fonction du type de trajectoire et du sens
void control_goto_near(int32_t x, int32_t y, int32_t alpha, int32_t dist, enum control_type type, enum trajectory_way way);

//!< pas d'asservissement, les deux roues en marche arrière, pwm à x %. Arrêt quand le robot ne bouge plus
void control_back_to_wall(void);

//!< arrêt de l'asservissement des moteurs
void control_free(void);

//!< donne la rotation a effecter pour aller de l'angle debut à l'angle fin
int32_t control_find_rotate(int32_t debut, int32_t fin);

//!< etat de control
int32_t control_get_state(void);

//!< permet d'indiquer un obstacle sur la trajectoire, en cours de mouvement, et la distance d'approche souhaitée
void control_set_front_object(struct fx_vect2* a, int32_t approx_dist);

//!< permet d'indiquer un obstacle sur la trajectoire, en cours de mouvement, et la distance d'approche souhaitée
void control_set_back_object(struct fx_vect2* a, int32_t approx_dist);

//!< limitation de la vitesse.
//!< vitesse en % de la vmax de configuration
void control_set_max_speed(uint32_t v_max_dist, uint32_t v_max_rot);

void control_disable_sick(void);

void control_enable_sick(void);

#endif