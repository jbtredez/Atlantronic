//! @file control.c
//! @brief Asservissement
//! @author Atlantronic

#include <math.h>
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

//! @todo réglage au pif
#define CONTROL_STACK_SIZE       150

//! période de la tache de propulsion en tick ("fréquence" de l'asservissement)
#define CONTROL_TICK_PERIOD        5*72000

#define TE                         (float) ((float)CONTROL_TICK_PERIOD) / ((float)72000000)

static void control_task(void *);
static float sinc( float x );

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

// TODO
// tests vite fait Tresgor :
// kx = 1
// ky = 0
// kalpha = 200
// Simulation :
// kx = 0.02
// ky = 0.0002
// kalpha = 0.002
static float control_kx = 0.02;
static float control_ky = 0.0002;
static float control_kalpha = 0.002;

union
{
	struct control_param_ad ad;
	struct control_param_arc arc;
} control_param;

static struct pid control_pid_av;
static struct pid control_pid_rot;

static float sinc( float x )
{
	if( fabs(x) < 0.01 )
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

	// TODO
	// tests vite fait Tresgor :
	// kp = 40
	// ki = 0
	// kd = 0
	// Simulation :
	// kp = 40
	// ki = 120
	// kd = 0
	pid_init(&control_pid_av, 40, 120, 0, PWM_ARR);

	// tests vite fait Tresgor :
	// kp = 40
	// ki = 0
	// kd = 0
	// Simulation :
	// kp = 1000000
	// ki = 50000
	// kd = 0
	pid_init(&control_pid_rot, 1000000, 50000, 0, PWM_ARR);
	/////

	control_state = CONTROL_READY_FREE;

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* arg)
{
	(void) arg;

	portTickType wake_time = systick_get_time();
	struct vect_pos pos;

	while(1)
	{
		location_update();
		pos = location_get_position();
// TODO mutex pour laisser les IT
		portENTER_CRITICAL();

		if(vTaskGetEvent() & EVENT_END)
		{
			control_state = CONTROL_END;
		}

		// calcul du prochain point
		switch(control_state)
		{
			case CONTROL_READY_FREE:
				break;
			case CONTROL_READY_ASSER:
				break;
			case CONTROL_STRAIGHT:
			case CONTROL_ROTATE:
			case CONTROL_GOTO:
				if(control_param.ad.angle)
				{
					// TODO marge en dur
					if(fabs(control_dest.alpha - pos.alpha) < 0.05f)
					{
						control_param.ad.angle = 0;
						control_cons.alpha = control_dest.alpha;
						control_cons.ca = control_dest.ca;
						control_cons.sa = control_dest.sa;
						control_v_dist_cons = 0;
						control_v_rot_cons = 0;
						trapeze_reset(&control_trapeze);
					}
					else
					{
						trapeze_set(&control_trapeze, 1000.0f*TE*TE/((float) M_PI*PARAM_VOIE_MOT), 1000.0f*TE/((float) M_PI*PARAM_VOIE_MOT));
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
					float ex = pos.ca  * (control_dest.x - pos.x) + pos.sa * (control_dest.y - pos.y);
					//float ey = -pos.sa * (control_dest.x - pos.x) + pos.ca * (control_dest.y - pos.y);

					if( fabsf(ex) < 2.0f)
					{
						//if(fabsf(ey) < 10.0f)
						//{
							control_param.ad.distance = 0;
							control_cons = control_dest;
							control_v_dist_cons = 0;
							control_v_rot_cons = 0;
							trapeze_reset(&control_trapeze);
						//}
					}
					else
					{
						trapeze_set(&control_trapeze, 1000.0f*TE*TE, 1000.0f*TE);
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
				break;
			case CONTROL_ARC:
				// TODO
				break;
			case CONTROL_END:

				break;
			default:
				// TODO cas d'erreur de prog
				break;
		}

		if(control_state != CONTROL_READY_FREE && control_state != CONTROL_END)
		{
			// calcul de l'erreur de position dans le repère du robot
			float ex = pos.ca  * (control_cons.x - pos.x) + pos.sa * (control_cons.y - pos.y);
			float ey = -pos.sa * (control_cons.x - pos.x) + pos.ca * (control_cons.y - pos.y);
			float ealpha = control_cons.alpha - pos.alpha;

			float v_d_c = control_v_dist_cons * cos(ealpha) + control_kx * ex;
			float v_r_c = control_v_rot_cons + control_ky * control_v_dist_cons * sinc(ealpha) * ey + control_kalpha * ealpha;

			float v_d = location_get_speed_curv_abs();
			float v_r = location_get_speed_rot();

			// régulation en vitesse
			float u_av = pid_apply(&control_pid_av, v_d_c - v_d);
			float u_rot = pid_apply(&control_pid_rot, v_r_c - v_r);

			// TODO : pb de saturation
			float u1 = u_av + u_rot;
			float u2 = u_av - u_rot;

			int sens1 = 1;
			int sens2 = 1;

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

			// TODO saturer autrement
			if(u1 > PWM_ARR)
			{
				u1 = PWM_ARR;
			}

			if(u2 > PWM_ARR)
			{
				u2 = PWM_ARR;
			}
			pwm_set(PWM_RIGHT, (uint32_t)u1, sens1);
			pwm_set(PWM_LEFT, (uint32_t)u2, sens2);
		}
		else
		{
			pwm_set(PWM_RIGHT, 0, 1);
			pwm_set(PWM_LEFT, 0, 1);
		}

		portEXIT_CRITICAL();

		wake_time += CONTROL_TICK_PERIOD;
		vTaskDelayUntil(wake_time);
	}
}

void control_straight(float dist)
{
	portENTER_CRITICAL();
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_STRAIGHT;
	}
	vTaskClearEvent(EVENT_CONTROL_READY);
	trapeze_reset(&control_trapeze);
	control_cons = location_get_position();
	control_dest = control_cons;
	control_dest.x += control_dest.ca * dist;
	control_dest.y += control_dest.sa * dist;
	control_param.ad.angle = 0;
	control_param.ad.distance = dist;
	portEXIT_CRITICAL();
}

void control_rotate(float angle)
{
	portENTER_CRITICAL();
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_ROTATE;
	}
	vTaskClearEvent(EVENT_CONTROL_READY);
	trapeze_reset(&control_trapeze);
	control_cons = location_get_position();
	control_dest = control_cons;
	control_dest.alpha += angle;
	control_dest.ca = cos(control_dest.alpha);
	control_dest.sa = sin(control_dest.alpha);
	control_param.ad.angle = angle;
	control_param.ad.distance = 0;
	portEXIT_CRITICAL();
}

void control_goto(float x, float y)
{
	portENTER_CRITICAL();
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_GOTO;
	}
	vTaskClearEvent(EVENT_CONTROL_READY);
	trapeze_reset(&control_trapeze);
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
