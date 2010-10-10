//! @file control.c
//! @brief Asservissement
//! @author Jean-Baptiste Trédez

#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "control/control.h"
#include "priority.h"
#include "log.h"
#include "location/location.h"
#include "vect_pos.h"
#include "io/pwm.h"
#include "control/pid.h"
#include "trapeze.h"
#include "robot_parameters.h"
#include "event.h"

//! @todo réglage au pif
#define CONTROL_STACK_SIZE       50

//! période de la tache de propulsion en tick ("fréquence" de l'asservissement)
#define CONTROL_TICK_PERIOD        5

#define TE                         ((float)CONTROL_TICK_PERIOD) / ((float)configTICK_RATE_HZ)

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

static int32_t state;
static struct vect_pos dest;
static struct vect_pos cons;
static float v_dist_cons;
static float v_rot_cons;
static struct trapeze trapeze;

union
{
	struct control_param_ad ad;
	struct control_param_arc arc;
} control_param;

static struct pid pid_av;
static struct pid pid_rot;

static float sinc( float x )
{
	if( fabs(x) < 0.001 )
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
	portBASE_TYPE err = xTaskCreate(control_task, (const signed char *) "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	trapeze_reset(&trapeze);

	pid_init(&pid_av);
	pid_init(&pid_rot);

	// TODO
	pid_av.kp = 40;
	pid_av.ki = 1500;
	pid_av.kd = 200;
	pid_av.max_out = 65535;

	pid_rot.kp = 400000;
	pid_rot.ki = 1500000;
	pid_rot.kd = 200000;
	pid_rot.max_out = 65535;
	/////

	state = READY_FREE;

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* arg)
{
	(void) arg;

	portTickType xLastWakeTime = xTaskGetTickCount();
	struct vect_pos pos;

	///// TODO
	float kx = 0.02;
	float ky = 0.0002;
	float kalpha = 0.002;
	////

	while(1)
	{
		location_update();
		pos = location_get_position();

		portENTER_CRITICAL();

		// calcul du prochain point
		switch(state)
		{
			case READY_FREE:
				break;
			case READY_ASSERT:
				break;
			case STRAIGHT:
			case ROTATE:
			case GOTO:
				if(control_param.ad.angle)
				{
					// TODO marge en dur
					if(fabs(dest.alpha - pos.alpha) < 0.01f)
					{
						control_param.ad.angle = 0;
						cons.alpha = dest.alpha;
						cons.ca = dest.ca;
						cons.sa = dest.sa;
						v_dist_cons = 0;
						v_rot_cons = 0;
						trapeze_reset(&trapeze);
					}
					else
					{
						trapeze_set(&trapeze, 1000*TE*TE/(M_PI*PARAM_VOIE_MOT), 1000*TE/(M_PI*PARAM_VOIE_MOT));
						trapeze_apply(&trapeze, control_param.ad.angle);
						cons.alpha += trapeze.v;
						cons.ca = cos(cons.alpha);
						cons.sa = sin(cons.alpha);
						v_dist_cons = 0;
						v_rot_cons = trapeze.v;
					}
				}
				else if(control_param.ad.distance)
				{
					// TODO marges en dur
					float ex = pos.ca  * (dest.x - pos.x) + pos.sa * (dest.y - pos.y);
					//float ey = -pos.sa * (dest.x - pos.x) + pos.ca * (dest.y - pos.y);

					if( fabsf(ex) < 1.0f)
					{
						//if(fabsf(ey) < 10.0f)
						//{
							control_param.ad.distance = 0;
							cons = dest;
							v_dist_cons = 0;
							v_rot_cons = 0;
							trapeze_reset(&trapeze);
						//}
					}
					else
					{
						trapeze_set(&trapeze, 1000*TE*TE, 1000*TE);
						trapeze_apply(&trapeze, control_param.ad.distance);
						cons.x += trapeze.v * cons.ca;
						cons.y += trapeze.v * cons.sa;
						v_dist_cons = trapeze.v;
						v_rot_cons = 0;
					}
				}
				else
				{
					v_dist_cons = 0;
					v_rot_cons = 0;
					state = READY_ASSERT;
					vTaskSetEvent(EVENT_CONTROL_READY);
				}
				break;
			case ARC:
				// TODO
				break;
			default:
				// TODO cas d'erreur de prog
				break;
		}

		if(state != READY_FREE)
		{
			// calcul de l'erreur de position dans le repère du robot
			float ex = pos.ca  * (cons.x - pos.x) + pos.sa * (cons.y - pos.y);
			float ey = -pos.sa * (cons.x - pos.x) + pos.ca * (cons.y - pos.y);
			float ealpha = cons.alpha - pos.alpha;

			float v_d_c = v_dist_cons * cos(ealpha) + kx * ex;
			float v_r_c = v_rot_cons + ky * v_dist_cons * sinc(ealpha) * ey + kalpha * ealpha;

			float v_d = location_get_speed_curv_abs();
			float v_r = location_get_speed_rot();

//TODO à virer, test
	if(state != READY_ASSERT)
	{
		meslog(_info_, 1, "%f   %f   %f : %f   %f   %f", cons.x, cons.y, cons.alpha, pos.x, pos.y, pos.alpha);
	}
			// régulation en vitesse
			float u_av = pid_apply(&pid_av, v_d_c - v_d);
			float u_rot = pid_apply(&pid_rot, v_r_c - v_r);

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

			pwm_set(PWM_RIGHT, (uint32_t)u1, sens1);
			pwm_set(PWM_LEFT, (uint32_t)u2, sens2);
		}
		else
		{
			pwm_set(PWM_RIGHT, 0, 1);
			pwm_set(PWM_LEFT, 0, 1);
		}

		portEXIT_CRITICAL();

		vTaskDelayUntil(&xLastWakeTime, CONTROL_TICK_PERIOD);
	}
}

void control_straight(float dist)
{
	portENTER_CRITICAL();
	state = STRAIGHT;
	vTaskClearEvent(EVENT_CONTROL_READY);
	trapeze_reset(&trapeze);
	cons = location_get_position();
	dest = cons;
	dest.x += dest.ca * dist;
	dest.y += dest.sa * dist;
	control_param.ad.angle = 0;
	control_param.ad.distance = dist;
	portEXIT_CRITICAL();
}

void control_rotate(float angle)
{
	portENTER_CRITICAL();
	state = ROTATE;
	vTaskClearEvent(EVENT_CONTROL_READY);
	trapeze_reset(&trapeze);
	cons = location_get_position();
	dest = cons;
	dest.alpha += angle;
	dest.ca = cos(dest.alpha);
	dest.sa = sin(dest.alpha);
	control_param.ad.angle = angle;
	control_param.ad.distance = 0;
	portEXIT_CRITICAL();
}

void control_goto(float x, float y)
{
	portENTER_CRITICAL();
	state = GOTO;
	vTaskClearEvent(EVENT_CONTROL_READY);
	trapeze_reset(&trapeze);
	cons = location_get_position();

	dest.x = x;
	dest.y = y;
	float dx = x - cons.x;
	float dy = y - cons.y;
	dest.alpha = atan2(dy, dx);
	dest.ca = cos(dest.alpha);
	dest.sa = sin(dest.alpha);

	control_param.ad.angle = dest.alpha - cons.alpha;
	control_param.ad.distance = sqrt(dx*dx+dy*dy);
	portEXIT_CRITICAL();
}

int32_t control_get_state()
{
	portENTER_CRITICAL();
	int32_t tmp = state;
	portEXIT_CRITICAL();
	return tmp;
}

void control_free()
{
	portENTER_CRITICAL();
	state = READY_FREE;
	vTaskSetEvent(EVENT_CONTROL_READY);
	portEXIT_CRITICAL();
}
