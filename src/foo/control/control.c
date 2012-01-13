//! @file control.c
//! @brief Asservissement
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
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
#include "kernel/driver/usb.h"
#include "adc.h"
#include "gpio.h"
#include <math.h>
#include <stdlib.h>
#include "us.h"

//! @todo réglage au pif
#define CONTROL_STACK_SIZE       300

//! période de la tache de propulsion en tick ("fréquence" de l'asservissement)
#define CONTROL_TICK_PERIOD        ms_to_tick(5)
#define CONTROL_HZ                 200
#define TE                         0.005f

#define CONTROL_POS_REACHED_TOLERANCE_X        2.0f

static void control_task(void *);
static float sinc( float x );
static void control_compute();
static void control_compute_goto();
static void control_colision_detection();

// interface usb
static void control_cmd_goto_near(void* arg);
static void control_cmd_straight(void* arg);
static void control_cmd_straight_to_wall(void* arg);
static void control_cmd_rotate(void* arg);
static void control_cmd_rotate_to(void* arg);
static void control_cmd_free(void* arg);
static void control_cmd_param(void* arg);
static void control_cmd_print_param(void* arg);

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

static struct control_usb_data control_usb_data;
static xSemaphoreHandle control_mutex;
static int32_t control_state;
static struct vect_pos control_dest;
static struct vect_pos control_cons;
static float control_v_dist_cons;
static float control_v_rot_cons;
static struct trapeze control_trapeze;
static struct adc_an control_an;
static uint8_t control_contact;
static uint8_t control_us;
static volatile struct vect_pos control_pos;
static portTickType control_timer;
static float control_kx;
static float control_ky;
static float control_kalpha;
static float control_aMax_av;
static float control_vMax_av;
static float control_aMax_rot;
static float control_vMax_rot;
static int control_use_us;

union
{
	struct control_param_ad ad;
	struct control_param_arc arc;
} control_param;

static struct pid control_pid_av;
static struct pid control_pid_rot;

static int control_module_init()
{
	control_use_us = US_FRONT_MASK | US_BACK_MASK;
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	control_mutex = xSemaphoreCreateMutex();

	if(control_mutex == NULL)
	{
		return ERR_INIT_CONTROL;
	}

	// coupe 2011 - coefs samedi - Angers
	// pid_init(&control_pid_av, 0.5f, 0.10f, 0, PWM_ARR);
	// pid_init(&control_pid_rot, 250.0f, 40.0f, 0, PWM_ARR);

	// test
	//pid_init(&control_pid_av, 0.7f, 0.01f, 0.0f, PWM_ARR);
	//pid_init(&control_pid_rot, 312.0f, 16.0f, 0.0f, PWM_ARR);

	pid_init(&control_pid_av, 0.5f, 0.10f, 0, PWM_ARR);
	pid_init(&control_pid_rot, 250.0f, 40.0f, 0, PWM_ARR);

	// coupe 2011 - coefs samedi - Angers
	//static float control_kx = 0.01f;
	//static float control_ky = 0;
	//static float control_kalpha = 0.015f;
	control_kx = 0.01f;
	control_ky = 0.0f;
	control_kalpha = 0.015f;

	control_aMax_av = 0;
	control_vMax_av = 0;
	control_aMax_rot = 0;
	control_vMax_rot = 0;

	control_state = CONTROL_READY_FREE;
	control_timer = 0;

	usb_add_cmd(USB_CMD_GOTO_NEAR, &control_cmd_goto_near);
	usb_add_cmd(USB_CMD_STRAIGHT, &control_cmd_straight);
	usb_add_cmd(USB_CMD_STRAIGHT_TO_WALL, &control_cmd_straight_to_wall);
	usb_add_cmd(USB_CMD_ROTATE, &control_cmd_rotate);
	usb_add_cmd(USB_CMD_ROTATE_TO, &control_cmd_rotate_to);
	usb_add_cmd(USB_CMD_FREE, &control_cmd_free);
	usb_add_cmd(USB_CMD_CONTROL_PARAM, &control_cmd_param);
	usb_add_cmd(USB_CMD_CONTROL_PRINT_PARAM, &control_cmd_print_param);

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* arg)
{
	(void) arg;

	portTickType wake_time = systick_get_time();

	while(1)
	{
		// mise à jour de la position
		location_update();

		// recuperation des entrées AN
		adc_get(&control_an);

		control_compute();

		wake_time += CONTROL_TICK_PERIOD;
		vTaskDelayUntil(wake_time);
	}
}

void control_cmd_param(void* arg)
{
	struct control_cmd_param_arg* cmd_arg = (struct control_cmd_param_arg*) arg;

	pid_init(&control_pid_av, cmd_arg->kp_av, cmd_arg->ki_av, cmd_arg->kd_av, PWM_ARR);
	pid_init(&control_pid_rot, cmd_arg->kp_rot, cmd_arg->ki_rot, cmd_arg->kd_rot, PWM_ARR);

	control_kx = cmd_arg->kx;
	control_ky = cmd_arg->ky;
	control_kalpha = cmd_arg->kalpha;
}

void control_cmd_print_param(void* arg)
{
	(void) arg;

	log_format(LOG_INFO, "av: %f %f %f", control_pid_av.kp/PWM_ARR, control_pid_av.ki/PWM_ARR, control_pid_av.kd/PWM_ARR);
	log_format(LOG_INFO, "rot: %f %f %f", control_pid_rot.kp/PWM_ARR, control_pid_rot.ki/PWM_ARR, control_pid_rot.kd/PWM_ARR);
	log_format(LOG_INFO, "rot: %f %f %f", control_kx, control_ky, control_kalpha);
}

static float sinc( float x )
{
	if( fabsf(x) < 0.01f )
	{
		return 1.0f;
	}
	else
	{
		return (sinf(x)/x);
	}
}

static void control_recalage()
{
	// on met alpha dans [0 ; 2*PI]
	float alpha = fmodf(control_pos.alpha, 2*PI);
	if(alpha < 0)
	{
		alpha += 2*PI;
	}

	if( control_pos.y + PARAM_NP_X + 1050.0f < 200.0f && fabsf(alpha - PI/2.0f) < PI/8.0f)
	{
		location_set_position(control_pos.x, -1050.0f - PARAM_NP_X, PI/2.0f);
	}
	else if( (alpha < PI/8.0f) && (control_pos.x + PARAM_NP_X + 1500.0f < 200.0f) )
	{
		location_set_position(-1500.0f - PARAM_NP_X, control_pos.y, 0);
	}
	else if( (fabsf(alpha - PI) < PI/8.0f) && (control_pos.x - PARAM_NP_X - 1500.0f > -200.0f) )
	{
		location_set_position(1500.0f + PARAM_NP_X, control_pos.y, PI);
	}
}

static int control_check_speed(int mes, int cons, int delta)
{
	int res = CONTROL_SPEED_OK;

	if( abs(mes) > abs(cons) + delta)
	{
		res = CONTROL_OVER_SPEED;
	}
	else if( abs(cons) > abs(mes) + delta )
	{
		res = CONTROL_UNDER_SPEED;
	}
	else if( abs(cons - mes) > delta && cons * mes < 0)
	{
		res =  CONTROL_WRONG_WAY;
	}

	return res;
}


static void control_compute()
{
	float u1 = 0;
	float u2 = 0;

	int sens1 = 1;
	int sens2 = 1;

	control_pos = location_get_position();
	float v_d = location_get_speed_curv_abs();
	float v_r = location_get_speed_rot();

	xSemaphoreTake(control_mutex, portMAX_DELAY);

	if(vTaskGetEvent() & EVENT_END)
	{
		control_state = CONTROL_END;
		goto end_pwm_critical;
	}

	// detection de collisions
	control_colision_detection();

	// verification du suivit de vitesse
	int vd_mm = v_d * CONTROL_HZ;
	int vd_cons_mm = control_v_dist_cons * CONTROL_HZ;
	int speed_check = control_check_speed(vd_mm, vd_cons_mm, 100);

	// calcul du prochain point :
	// control_v_dist_cons
	// control_v_rot_cons
	// control_cons
	switch(control_state)
	{
		case CONTROL_READY_FREE:
			control_v_dist_cons = 0;
			control_v_rot_cons = 0;
			goto end_pwm_critical;
			break;
		case CONTROL_READY_ASSER:
			control_v_dist_cons = 0;
			control_v_rot_cons = 0;
			break;
		case CONTROL_STRAIGHT:
		case CONTROL_ROTATE:
		case CONTROL_GOTO:
			// on a eu une collision. Ce n'est pas prevu sur ce type de trajectoire
			// => on va tout couper.
			if( control_contact)
			{
				log(LOG_INFO, "collision - contact : goto => free");
				control_state = CONTROL_READY_FREE;
				vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION);
			}
			else if(speed_check)
			{
				log_format(LOG_ERROR, "erreur de suivit cons %d mes %d check %d", vd_cons_mm, vd_mm, speed_check);
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
				u1 = -PWM_ARR;
				u2 = -PWM_ARR;
				if(control_contact & CONTACT_RIGHT)
				{
					u1 = 0;
				}

				if(control_contact & CONTACT_LEFT)
				{
					u2 = 0;
				}
				goto end_pwm_critical;
			}
			else if( control_contact == (CONTACT_RIGHT | CONTACT_LEFT) )
			{
				log(LOG_INFO, "double contact");
				// obstacle des deux cotés
				control_recalage();

				control_state = CONTROL_READY_FREE;
				vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION);
				goto end_pwm_critical;
			}
			control_compute_goto();
			break;
		case CONTROL_ARC:
			log_format(LOG_ERROR, "trajectoire non programmee : %ld", control_state);
			control_state = CONTROL_READY_FREE;
			vTaskSetEvent(EVENT_CONTROL_READY);
			goto end_pwm_critical;
			break;
		case CONTROL_END:
			goto end_pwm_critical;
			break;
		default:
			// erreur de prog ou corruption mem
			log_format(LOG_ERROR, "etat inconnu : %ld", control_state);
			control_state = CONTROL_READY_FREE;
			vTaskSetEvent(EVENT_CONTROL_READY);
			goto end_pwm_critical;
			break;
	}

	// gestion du timeout (on demande de ne pas bouger pendant 2 sec avec asservissement actif)
	// condition "on ne demande pas de bouger" (donc zero pur)
	if( control_v_rot_cons == 0 && control_v_dist_cons == 0 )
	{
		// on augmente le temps avec consigne nulle
		control_timer += CONTROL_TICK_PERIOD;
		if( control_timer > ms_to_tick(2000))
		{
			// ca fait 2 seconde qu'on devrait avoir terminé la trajectoire
			// il y a un probleme
			log_format(LOG_INFO, "2 sec sans bouger => CONTROL_READY_FREE");
			control_state = CONTROL_READY_FREE;
			vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_TIMEOUT);
		}
		goto end_pwm_critical;
	}
	else
	{
		control_timer = 0;
	}

	// calcul de l'erreur de position dans le repère du robot
	float ex = control_pos.ca  * (control_cons.x - control_pos.x) + control_pos.sa * (control_cons.y - control_pos.y);
	float ey = -control_pos.sa * (control_cons.x - control_pos.x) + control_pos.ca * (control_cons.y - control_pos.y);
	float ealpha = control_cons.alpha - control_pos.alpha;

	float v_d_c = control_v_dist_cons * cosf(ealpha) + control_kx * ex;
	float v_r_c = control_v_rot_cons + control_ky * control_v_dist_cons * sinc(ealpha) * ey + control_kalpha * ealpha;

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
		float max = PWM_ARR - fabsf(u_rot);
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

end_pwm_critical:
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
	control_usb_data.control_state = control_state;
	control_usb_data.control_dest_x = control_dest.x;
	control_usb_data.control_dest_y = control_dest.y;
	control_usb_data.control_dest_alpha = control_dest.alpha;
	control_usb_data.control_cons_x = control_cons.x;
	control_usb_data.control_cons_y = control_cons.y;
	control_usb_data.control_cons_alpha = control_cons.alpha;
	control_usb_data.control_pos_x = control_pos.x;
	control_usb_data.control_pos_y = control_pos.y;
	control_usb_data.control_pos_alpha = control_pos.alpha;
	control_usb_data.control_v_dist_cons = control_v_dist_cons;
	control_usb_data.control_v_rot_cons = control_v_rot_cons;
	control_usb_data.control_v_dist_mes = v_d;
	control_usb_data.control_v_rot_mes = v_r;
	control_usb_data.control_i_right = control_an.i_right;
	control_usb_data.control_i_left = control_an.i_left;
	xSemaphoreGive(control_mutex);

	usb_add(USB_CONTROL, &control_usb_data, sizeof(control_usb_data));
}

static void control_compute_goto()
{
	if(control_param.ad.angle)
	{
		// TODO marge en dur
		if(fabsf(control_dest.alpha - control_pos.alpha) < 0.02f)
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
			trapeze_set(&control_trapeze, control_vMax_rot, control_aMax_rot);
			trapeze_apply(&control_trapeze, control_param.ad.angle);
			control_cons.alpha += control_trapeze.v;
			control_cons.ca = cosf(control_cons.alpha);
			control_cons.sa = sinf(control_cons.alpha);
			control_v_dist_cons = 0;
			control_v_rot_cons = control_trapeze.v;
		}
	}
	else if(control_param.ad.distance)
	{
		if( control_param.ad.distance > 0)
		{
			if( control_us & US_FRONT_MASK)
			{
				control_v_dist_cons = 0;
				control_v_rot_cons = 0;
				control_state = CONTROL_READY_FREE;
				vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION);
			}
		}
		else
		{
			if( control_us & US_BACK_MASK)
			{
				control_v_dist_cons = 0;
				control_v_rot_cons = 0;
				control_state = CONTROL_READY_FREE;
				vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION);
			}
		}

		// TODO marges en dur
		float ex = control_pos.ca  * (control_dest.x - control_pos.x) + control_pos.sa * (control_dest.y - control_pos.y);
		//float ey = -control_pos.sa * (control_dest.x - control_pos.x) + control_pos.ca * (control_dest.y - control_pos.y);

		if( fabsf(ex) < CONTROL_POS_REACHED_TOLERANCE_X)
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
			control_v_rot_cons = 0;
			trapeze_set(&control_trapeze, control_vMax_av, control_aMax_av);
			trapeze_apply(&control_trapeze, control_param.ad.distance);
			control_v_dist_cons = control_trapeze.v;
			control_cons.x += control_v_dist_cons * control_cons.ca;
			control_cons.y += control_v_dist_cons * control_cons.sa;
		}
	}
	else
	{
		log(LOG_INFO, "target reached");
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

/*	// TODO régler seuil
	if( control_an.i_right > 2000 )
	{
		control_contact |= CONTACT_RIGHT;
	}

	// TODO régler seuil
	if( control_an.i_left > 2000 )
	{
		control_contact |= CONTACT_LEFT;
	}
*/
	// arrêt sur us si demande des us
	if( control_use_us )
	{
		control_us = us_check_collision() & control_use_us;
	}
	else
	{
		control_us = 0;
	}
}

void control_cmd_straight(void* arg)
{
	struct control_cmd_straight_arg* cmd_arg = (struct control_cmd_straight_arg*) arg;
	control_straight(cmd_arg->dist);
}

void control_straight(float dist)
{
	log_format(LOG_INFO, "param %.2f", dist);
	xSemaphoreTake(control_mutex, portMAX_DELAY);
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
	control_aMax_av = 500.0f*TE*TE;
	control_vMax_av = 1000.0f*TE;
	control_aMax_rot = 0;
	control_vMax_rot = 0;
	pid_reset(&control_pid_av);
	pid_reset(&control_pid_rot);
	xSemaphoreGive(control_mutex);
}

void control_cmd_rotate(void* arg)
{
	struct control_cmd_rotate_arg* cmd_arg = (struct control_cmd_rotate_arg*) arg;
	control_rotate(cmd_arg->angle);
}

void control_rotate(float angle)
{
	log_format(LOG_INFO, "param %f", angle);
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_ROTATE;
	}
	vTaskClearEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);
	trapeze_reset(&control_trapeze, 0, 0);
	control_cons = location_get_position();
	control_dest = control_cons;
	control_dest.alpha += angle;
	control_dest.ca = cosf(control_dest.alpha);
	control_dest.sa = sinf(control_dest.alpha);
	control_param.ad.angle = angle;
	control_param.ad.distance = 0;
	control_timer = 0;
	control_aMax_av = 0;
	control_vMax_av = 0;
	control_aMax_rot = 1200.0f*TE*TE/((float) PI*PARAM_VOIE_MOT);
	control_vMax_rot = 1500.0f*TE/((float) PI*PARAM_VOIE_MOT);
	pid_reset(&control_pid_av);
	pid_reset(&control_pid_rot);
	xSemaphoreGive(control_mutex);
}

void control_cmd_rotate_to(void* arg)
{
	struct control_cmd_rotate_to_arg* cmd_arg = (struct control_cmd_rotate_to_arg*) arg;
	control_rotate_to(cmd_arg->angle);
}

void control_rotate_to(float alpha)
{
	log_format(LOG_INFO, "param %f", alpha);
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_ROTATE;
	}
	vTaskClearEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);
	trapeze_reset(&control_trapeze, 0, 0);
	control_cons = location_get_position();
	control_dest = control_cons;
	float da = fmodf(alpha - control_cons.alpha, 2*PI);
	control_dest.alpha += da;
	control_dest.ca = cosf(control_dest.alpha);
	control_dest.sa = sinf(control_dest.alpha);
	control_param.ad.angle = da;
	control_param.ad.distance = 0;
	control_timer = 0;
	control_aMax_av = 0;
	control_vMax_av = 0;
	control_aMax_rot = 1200.0f*TE*TE/((float) PI*PARAM_VOIE_MOT);
	control_vMax_rot = 1500.0f*TE/((float) PI*PARAM_VOIE_MOT);
	pid_reset(&control_pid_av);
	pid_reset(&control_pid_rot);
	xSemaphoreGive(control_mutex);
}

float trouverRotation(float debut, float fin)
{
	float alpha = fin - debut;
	alpha = fmodf(alpha, 2*PI); // Retour dans [-2*PI;2*PI]
	
	// Retour dans [-PI;PI] si neccessaire
	if (alpha > PI)
	{
		alpha -= 2*PI;
	}
	else if (alpha < -PI)
	{
		alpha += 2*PI;
	}
	
	return alpha;
}

void control_cmd_goto_near(void* arg)
{
	struct control_cmd_goto_near_arg* cmd_arg = (struct control_cmd_goto_near_arg*) arg;
	control_goto_near(cmd_arg->x, cmd_arg->y, cmd_arg->dist, cmd_arg->way);
}

void control_goto_near(float x, float y, float dist, enum control_way sens)
{
	log_format(LOG_INFO, "param %.2f %.2f %.2f %d", x, y, dist, sens);
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_GOTO;
	}
	vTaskClearEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);
	trapeze_reset(&control_trapeze, 0, 0);
	control_cons = location_get_position();


	float dx = x - control_cons.x;
	float dy = y - control_cons.y;
	control_param.ad.distance = sqrtf(dx*dx+dy*dy) - dist;
	float a = atan2f(dy, dx);

	if(sens == CONTROL_FORWARD)
	{
		control_param.ad.angle = trouverRotation(control_cons.alpha, a);
	}
	else if(sens == CONTROL_BACKWARD)
	{
		control_param.ad.angle = trouverRotation(control_cons.alpha, a + PI);
		control_param.ad.distance *= -1;
	}
	else
	{
		float theta1av = trouverRotation(control_cons.alpha, a);
		float theta1re = trouverRotation(control_cons.alpha, a + PI);

		if ( fabsf(theta1av) > fabsf(theta1re))
		{
			control_param.ad.angle = theta1re;
			control_param.ad.distance *= -1;
		}
		else
		{
			control_param.ad.angle = theta1av;
		}
	}

	control_dest.alpha = control_cons.alpha + control_param.ad.angle;
	control_dest.ca = cosf(control_dest.alpha);
	control_dest.sa = sinf(control_dest.alpha);
	control_dest.x = control_cons.x + control_param.ad.distance * control_dest.ca;
	control_dest.y = control_cons.y + control_param.ad.distance * control_dest.sa;

	control_timer = 0;
	control_aMax_av = 350.0f*TE*TE;
	control_vMax_av = 1300.0f*TE;
	control_aMax_rot = 1200.0f*TE*TE/((float) PI*PARAM_VOIE_MOT);
	control_vMax_rot = 1500.0f*TE/((float) PI*PARAM_VOIE_MOT);
	pid_reset(&control_pid_av);
	pid_reset(&control_pid_rot);
	xSemaphoreGive(control_mutex);
}

void control_cmd_straight_to_wall(void* arg)
{
	struct control_cmd_straight_to_wall_arg* cmd_arg = (struct control_cmd_straight_to_wall_arg*) arg;
	control_straight_to_wall(cmd_arg->dist);
}

void control_straight_to_wall(float dist)
{
	log_format(LOG_INFO, "param %.2f", dist);
	xSemaphoreTake(control_mutex, portMAX_DELAY);
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
	control_aMax_av = 100.0f*TE*TE;
	control_vMax_av = 250.0f*TE;
	control_aMax_rot = 0;
	control_vMax_rot = 0;
	pid_reset(&control_pid_av);
	pid_reset(&control_pid_rot);
	xSemaphoreGive(control_mutex);
}

int32_t control_get_state()
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	int32_t tmp = control_state;
	xSemaphoreGive(control_mutex);
	return tmp;
}

void control_cmd_free(void* arg)
{
	(void) arg;
	control_free();
}

void control_free()
{
	log(LOG_INFO, "free wheel");
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_READY_FREE;
		vTaskSetEvent(EVENT_CONTROL_READY);
	}
	xSemaphoreGive(control_mutex);
}

void control_set_use_us(uint8_t use_us_mask)
{
	control_use_us = use_us_mask;
}
