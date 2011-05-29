//! @file control.c
//! @brief Asservissement
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "priority.h"
#include "kernel/log.h"
#include "location/location.h"
#include "kernel/vect_pos.h"
#include "pwm.h"
#include "control/pid.h"
#include "kernel/trapeze.h"
#include "kernel/robot_parameters.h"
#include "kernel/event.h"
#include "adc.h"
#include "gpio.h"
#include <math.h>

//! @todo réglage au pif
#define CONTROL_STACK_SIZE       200

//! période de la tache de propulsion en tick ("fréquence" de l'asservissement)
#define CONTROL_TICK_PERIOD        5*72000

#define TE                         (float) ((float)CONTROL_TICK_PERIOD) / ((float)72000000)

static void control_task(void *);
static float sinc( float x );
static void control_compute();
static void control_compute_goto();
static void control_colision_detection();


struct control_param_ad
{
	float angle;
	float distance;
};

struct control_param_arc
{
	float r;
	float angle;
};

static int32_t control_state;
static struct vect_pos control_dest;
static struct vect_pos control_cons;
static float control_v_dist_cons;
static float control_v_rot_cons;
static struct trapeze control_trapeze;
static struct adc_an control_an;
static uint8_t control_contact;
static volatile struct vect_pos control_pos;
static portTickType control_timer;

// coupe 2011 - coefs samedi - Angers
//static float control_kx = 0.01f;
//static float control_ky = 0;
//static float control_kalpha = 0.015f;
static float control_kx = 0.01f;
static float control_ky = 0;
static float control_kalpha = 0.015f;

union
{
	struct control_param_ad ad;
	struct control_param_arc arc;
} control_param;

static struct pid control_pid_av;
static struct pid control_pid_rot;

static float sinc( float x )
{
	if( fabsf(x) < 0.01 )
	{
		return 1.0;
	}
	else
	{
		return (sin(x)/x);
	}
}

static int control_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	// coupe 2011 - coefs samedi - Angers
	// 	pid_init(&control_pid_av, 0.5f, 0.10f, 0, PWM_ARR);
	pid_init(&control_pid_av, 0.5f, 0.10f, 0, PWM_ARR);

	// coupe 2011 - coefs samedi - Angers
	// 	pid_init(&control_pid_rot, 250.0f, 40.0f, 0, PWM_ARR);
	pid_init(&control_pid_rot, 250.0f, 40.0f, 0, PWM_ARR);


	control_state = CONTROL_READY_FREE;
	control_timer = 0;

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* arg)
{
	(void) arg;

	portTickType wake_time = systick_get_time();

	while(1)
	{
		control_compute();

		wake_time += CONTROL_TICK_PERIOD;
		vTaskDelayUntil(wake_time);
	}
}

static void control_recalage()
{
	if( control_pos.y + PARAM_NP_X + 1050.0f < 200.0f && fabsf(control_pos.alpha - PI/2.0f) < PI/8.0f)
	{
		location_set_position(control_pos.x, -1050.0f - PARAM_NP_X, PI/2.0f);
	}
	else if( (fabsf(control_pos.alpha) < PI/8.0f) && (control_pos.x + PARAM_NP_X + 1500.0f < 200.0f) )
	{
		location_set_position(-1500.0f - PARAM_NP_X, control_pos.y, 0);
	}
}

static void control_compute()
{
	float u1 = 0;
	float u2 = 0;

	int sens1 = 1;
	int sens2 = 1;

	location_update();
	control_pos = location_get_position();

	adc_get(&control_an);

// TODO mutex pour laisser les IT
	portENTER_CRITICAL();

	if(vTaskGetEvent() & EVENT_END)
	{
		control_state = CONTROL_END;
		goto end_pwm_critical;
	}

	// detection de collisions
	control_colision_detection();

	// calcul du prochain point
	switch(control_state)
	{
		case CONTROL_READY_FREE:
			goto end_pwm_critical;
			break;
		case CONTROL_READY_ASSER:
			break;
		case CONTROL_STRAIGHT:
		case CONTROL_ROTATE:
		case CONTROL_GOTO:
			// on a eu une collision. Ce n'est pas prevu sur ce type de trajectoire
			// => on va tout couper.
			if( control_contact )
			{
				control_state = CONTROL_READY_FREE;
				vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION);
			}
			else
			{
				control_compute_goto();
			}
			break;
		case CONTROL_STRAIGHT_TO_WALL:
			// on a eu une collision
			if( control_contact == CONTACT_LEFT || control_contact == CONTACT_RIGHT)
			{
				// obstacle sur un des deux cotés
				// il faut laisser tourner le robot et desactiver l'asservissement en rotation
				control_dest.alpha = control_pos.alpha;
				control_dest.ca = control_pos.ca;
				control_dest.sa = control_pos.sa;
				control_cons.alpha = control_pos.alpha;
				control_cons.ca = control_pos.ca;
				control_cons.sa = control_pos.sa;
			}
			else if( control_contact == (CONTACT_RIGHT | CONTACT_LEFT) )
			{
				// obstacle des deux cotés
				control_recalage();

				control_state = CONTROL_READY_FREE;
				vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION);
			}
			control_compute_goto();
			break;
		case CONTROL_ARC:
			// TODO
			goto end_pwm_critical;
			break;
		case CONTROL_END:
			goto end_pwm_critical;
			break;
		default:
			// erreur de prog ou corruption mem
			goto end_pwm_critical;
			break;
	}

	// gestion du timeout
	// condition "on ne demande pas de bouger"
/*	if( fabsf(control_v_dist_cons) < 0.01f && fabsf(control_v_rot_cons) < 0.01f)
	{
		// on augmente le temps avec consigne nulle
		control_timer += CONTROL_TICK_PERIOD;
		if( control_timer > ms_to_tick(1000))
		{
			// ca fait une seconde qu'on devrait avoir termine la trajectoire
			// il y a un probleme
			control_state = CONTROL_READY_FREE;
			vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_TIMEOUT);
		}
		goto end_pwm_critical;
	}
*/
	// calcul de l'erreur de position dans le repère du robot
	float ex = control_pos.ca  * (control_cons.x - control_pos.x) + control_pos.sa * (control_cons.y - control_pos.y);
	float ey = -control_pos.sa * (control_cons.x - control_pos.x) + control_pos.ca * (control_cons.y - control_pos.y);
	float ealpha = control_cons.alpha - control_pos.alpha;

	float v_d_c = control_v_dist_cons * cos(ealpha) + control_kx * ex;
	float v_r_c = control_v_rot_cons + control_ky * control_v_dist_cons * sinc(ealpha) * ey + control_kalpha * ealpha;

	float v_d = location_get_speed_curv_abs();
	float v_r = location_get_speed_rot();

	// régulation en vitesse
	float u_av = pid_apply(&control_pid_av, v_d_c - v_d);
	float u_rot = pid_apply(&control_pid_rot, v_r_c - v_r);

	// on prefere l'angle à l'avance en cas de saturation
	if( u_rot > PWM_ARR)
	{
		u_rot = PWM_ARR;
		u_av = 0;
	}
	else if( u_rot < - PWM_ARR)
	{
		u_rot = - PWM_ARR;
		u_av = 0;
	}
	else
	{
		// la rotation ne prend pas toute la pwm
		float max = PWM_ARR - fabs(u_rot);
		if( u_av > max)
		{
			u_av = max;
		}
		else if( u_av < -max)
		{
			u_av = -max;
		}
	}

	u1 = u_av + u_rot;
	u2 = u_av - u_rot;

	if(u1 < 0)
	{
		sens1 = -1;
		u1 = -u1;
	}
	if(u2 < 0)
	{
		sens2 = -1;
		u2 = -u2;
	}

	if(control_contact & CONTACT_RIGHT)
	{
		u1 = 0;
	}

	if(control_contact & CONTACT_LEFT)
	{
		u1 = 0;
	}

end_pwm_critical:
	pwm_set(PWM_RIGHT, (uint32_t)u1, sens1);
	pwm_set(PWM_LEFT, (uint32_t)u2, sens2);
	portEXIT_CRITICAL();
}

static void control_compute_goto()
{
	if(control_param.ad.angle)
	{
		// TODO marge en dur
		if(fabs(control_dest.alpha - control_pos.alpha) < 0.02f)
		{
			control_param.ad.angle = 0;
			control_cons.alpha = control_dest.alpha;
			control_cons.ca = control_dest.ca;
			control_cons.sa = control_dest.sa;
			control_v_dist_cons = 0;
			control_v_rot_cons = 0;
			trapeze_reset(&control_trapeze, 0, 0);
		}
		else
		{
			trapeze_set(&control_trapeze, 1000.0f*TE/((float) PI*PARAM_VOIE_MOT), 800.0f*TE*TE/((float) PI*PARAM_VOIE_MOT));
			trapeze_apply(&control_trapeze, control_param.ad.angle);
			control_cons.alpha += control_trapeze.v;
			control_cons.ca = cos(control_cons.alpha);
			control_cons.sa = sin(control_cons.alpha);
			control_v_dist_cons = 0;
			control_v_rot_cons = control_trapeze.v;
		}
	}
	else if(control_param.ad.distance)
	{
		// TODO marges en dur
		float ex = control_pos.ca  * (control_dest.x - control_pos.x) + control_pos.sa * (control_dest.y - control_pos.y);
		//float ey = -control_pos.sa * (control_dest.x - control_pos.x) + control_pos.ca * (control_dest.y - control_pos.y);

		if( fabsf(ex) < 2.0f)
		{
			//if(fabsf(ey) < 10.0f)
			//{
				control_param.ad.distance = 0;
				control_cons = control_dest;
				control_v_dist_cons = 0;
				control_v_rot_cons = 0;
				trapeze_reset(&control_trapeze, 0, 0);
			//}
		}
		else
		{
			trapeze_set(&control_trapeze, 1000.0f*TE, 250.0f*TE*TE);
			trapeze_apply(&control_trapeze, control_param.ad.distance);
			control_cons.x += control_trapeze.v * control_cons.ca;
			control_cons.y += control_trapeze.v * control_cons.sa;
			control_v_dist_cons = control_trapeze.v;
			control_v_rot_cons = 0;
		}
	}
	else
	{
		control_v_dist_cons = 0;
		control_v_rot_cons = 0;
		control_state = CONTROL_READY_ASSER;
		vTaskSetEvent(EVENT_CONTROL_READY);
	}
}

//! le but est de mettre a jour control_contact
//! on detecte une colision sur le coté droit ou sur le coté gauche
static void control_colision_detection()
{
	if( control_param.ad.distance < 0)
	{
		control_contact = get_contact();
	}
	else
	{
		control_contact = 0;
	}

	// TODO régler seuil
	if( control_an.i_right > 2000 )
	{
		control_contact |= CONTACT_RIGHT;
	}

	// TODO régler seuil
	if( control_an.i_left > 2000 )
	{
		control_contact |= CONTACT_LEFT;
	}
}

void control_straight(float dist)
{
	portENTER_CRITICAL();
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_STRAIGHT;
	}
	vTaskClearEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);
	trapeze_reset(&control_trapeze, 0, 0);
	control_cons = location_get_position();
	control_dest = control_cons;
	control_dest.x += control_dest.ca * dist;
	control_dest.y += control_dest.sa * dist;
	control_param.ad.angle = 0;
	control_param.ad.distance = dist;
	control_timer = 0;
	portEXIT_CRITICAL();
}

void control_rotate(float angle)
{
	portENTER_CRITICAL();
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_ROTATE;
	}
	vTaskClearEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);
	trapeze_reset(&control_trapeze, 0, 0);
	control_cons = location_get_position();
	control_dest = control_cons;
	control_dest.alpha += angle;
	control_dest.ca = cos(control_dest.alpha);
	control_dest.sa = sin(control_dest.alpha);
	control_param.ad.angle = angle;
	control_param.ad.distance = 0;
	control_timer = 0;
	portEXIT_CRITICAL();
}

void control_goto(float x, float y)
{
	portENTER_CRITICAL();
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_GOTO;
	}
	vTaskClearEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);
	trapeze_reset(&control_trapeze, 0, 0);
	control_cons = location_get_position();

	control_dest.x = x;
	control_dest.y = y;
	float dx = x - control_cons.x;
	float dy = y - control_cons.y;
	control_dest.alpha = atan2(dy, dx);
	control_dest.ca = cos(control_dest.alpha);
	control_dest.sa = sin(control_dest.alpha);

	control_param.ad.angle = control_dest.alpha - control_cons.alpha;
	control_param.ad.distance = sqrt(dx*dx+dy*dy);
	control_timer = 0;
	portEXIT_CRITICAL();
}

void control_straight_to_wall(float dist)
{
	portENTER_CRITICAL();
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_STRAIGHT_TO_WALL;
	}
	vTaskClearEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);
	trapeze_reset(&control_trapeze, 0, 0);
	control_cons = location_get_position();
	control_dest = control_cons;
	control_dest.x += control_dest.ca * dist;
	control_dest.y += control_dest.sa * dist;
	control_param.ad.angle = 0;
	control_param.ad.distance = dist;
	control_timer = 0;
	portEXIT_CRITICAL();
}

int32_t control_get_state()
{
	portENTER_CRITICAL();
	int32_t tmp = control_state;
	portEXIT_CRITICAL();
	return tmp;
}

void control_free()
{
	portENTER_CRITICAL();
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_READY_FREE;
		vTaskSetEvent(EVENT_CONTROL_READY);
	}
	portEXIT_CRITICAL();
}