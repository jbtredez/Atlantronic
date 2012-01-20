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
#include "kernel/robot_parameters.h"
#include "kernel/event.h"
#include "kernel/driver/usb.h"
#include "adc.h"
#include "gpio.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include "detection.h"
#include "kernel/trapeze.h"

//! @todo réglage au pif
#define CONTROL_STACK_SIZE       350
#define TRAJECTORY_POS_REACHED_TOLERANCE_X        2.0f
#define TRAJECTORY_POS_REACHED_TOLERANCE_ALPHA   0.02f

//! période de la tache de propulsion en tick ("fréquence" de l'asservissement)
#define CONTROL_TICK_PERIOD        ms_to_tick(5)
#define CONTROL_HZ                 200
#define TE                         0.005f

static void control_task(void *);
static float sinc( float x );
static void control_compute();
static void control_compute_trajectory();

// interface usb
void control_cmd_free(void* arg);
void control_cmd_param(void* arg);
void control_cmd_print_param(void* arg);

static struct control_usb_data control_usb_data;
static xSemaphoreHandle control_mutex;
static int32_t control_state;
static struct vect_pos control_cons;
static struct adc_an control_an;
static struct vect_pos control_pos;
static portTickType control_timer;
static float control_kx;
static float control_ky;
static float control_kalpha;
static struct vect_pos control_dest;
static struct trapeze control_trapeze_rot;
static struct trapeze control_trapeze_av;
static float control_angle;
static float control_dist;

static struct pid control_pid_av;
static struct pid control_pid_rot;

static int control_module_init()
{
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
	control_kx = 0;//0.01f;
	control_ky = 0.0f;
	control_kalpha = 0;//0.015f;

	trapeze_reset(&control_trapeze_av, 0, 0);
	trapeze_reset(&control_trapeze_rot, 0, 0);

	control_trapeze_av.v_max = 400 * TE;
	control_trapeze_av.a_max = 500 * TE * TE;
	control_trapeze_av.d_max = 1000 * TE * TE;
	control_trapeze_rot.v_max = 400 * TE / ((float) PI*PARAM_VOIE_MOT);
	control_trapeze_rot.a_max = 500 * TE * TE / ((float) PI*PARAM_VOIE_MOT);
	control_trapeze_rot.d_max = 1000 * TE * TE / ((float) PI*PARAM_VOIE_MOT);

	control_state = CONTROL_READY_FREE;
	control_timer = 0;

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
	log_format(LOG_INFO, "pos: %f %f %f", control_kx, control_ky, control_kalpha);
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
#if 0
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
#endif
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

	// verification du suivit de vitesse
	int vd_mm = v_d * CONTROL_HZ;
	int vd_cons_mm = control_trapeze_av.v * CONTROL_HZ;
	int speed_check = control_check_speed(vd_mm, vd_cons_mm, 200);

	if( control_state == CONTROL_TRAJECTORY)
	{
		if(speed_check)
		{
			log_format(LOG_ERROR, "erreur de suivit cons %d mes %d check %d", vd_cons_mm, vd_mm, speed_check);
			control_state = CONTROL_READY_FREE;
			vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION);
			goto end_pwm_critical;
		}
		else
		{
			control_compute_trajectory();
		}
	}
	else if(control_state == CONTROL_READY_ASSER)
	{
		trapeze_reset(&control_trapeze_rot, 0, 0);
		trapeze_reset(&control_trapeze_av, 0, 0);
	}
	else
	{
		trapeze_reset(&control_trapeze_rot, 0, 0);
		trapeze_reset(&control_trapeze_av, 0, 0);
		goto end_pwm_critical;
	}

	// gestion du timeout (on demande de ne pas bouger pendant 2 sec avec asservissement actif)
	// condition "on ne demande pas de bouger" (donc zero pur)
	if( control_trapeze_rot.v == 0 && control_trapeze_av.v == 0 )
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

	float v_d_c = control_trapeze_av.v * cosf(ealpha) + control_kx * ex;
	float v_r_c = control_trapeze_rot.v + control_ky * control_trapeze_av.v * sinc(ealpha) * ey + control_kalpha * ealpha;

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
	control_usb_data.control_v_dist_cons = control_trapeze_av.v;
	control_usb_data.control_v_rot_cons = control_trapeze_rot.v;
	control_usb_data.control_v_dist_mes = v_d;
	control_usb_data.control_v_rot_mes = v_r;
	control_usb_data.control_i_right = control_an.i_right;
	control_usb_data.control_i_left = control_an.i_left;
	xSemaphoreGive(control_mutex);

	usb_add(USB_CONTROL, &control_usb_data, sizeof(control_usb_data));
}

//! generation de trajectoire pour aller vers le point voulu
static void control_compute_trajectory()
{
	int collision = 0;

	if(control_angle)
	{
		if(	fabsf(control_dest.alpha - control_pos.alpha) < TRAJECTORY_POS_REACHED_TOLERANCE_ALPHA)
		{
			control_angle = 0;
			control_cons.alpha = control_dest.alpha;
			control_cons.ca = control_dest.ca;
			control_cons.sa = control_dest.sa;
			trapeze_reset(&control_trapeze_rot, 0, 0);
		}
		else
		{
			trapeze_apply(&control_trapeze_rot, control_angle);
			control_cons.alpha += control_trapeze_rot.v;
			control_cons.ca = cosf(control_cons.alpha);
			control_cons.sa = sinf(control_cons.alpha);
		}
	}
	else if(control_dist)
	{
		float ex = control_pos.ca  * (control_dest.x - control_pos.x) + control_pos.sa * (control_dest.y - control_pos.y);

		if( fabsf(ex) < TRAJECTORY_POS_REACHED_TOLERANCE_X)
		{
			control_dist = 0;
			control_cons = control_dest;
			trapeze_reset(&control_trapeze_av, 0, 0);
		}
		else
		{
			float distance = control_dist;
			if(distance > 0)
			{
				struct vect_pos front_obj;
				struct vect_pos front_obj_robot;
				detection_get_front_object(&front_obj);
				pos_table_to_robot(&control_cons, &front_obj, &front_obj_robot);
				float dist = front_obj_robot.x - PARAM_LEFT_CORNER_X - 50;
				if(dist < 0)
				{
					dist = 0;
					collision = 1;
				}

				if(dist < ex)
				{
					distance = control_trapeze_av.s + dist;
				}
			}

			trapeze_apply(&control_trapeze_av, distance);

			control_cons.x += control_trapeze_av.v * control_cons.ca;
			control_cons.y += control_trapeze_av.v * control_cons.sa;
		}
	}
	else
	{
		log(LOG_INFO, "target reached");
		trapeze_reset(&control_trapeze_rot, 0, 0);
		trapeze_reset(&control_trapeze_av, 0, 0);
		control_state = CONTROL_READY_ASSER;
		vTaskSetEvent(EVENT_CONTROL_READY);
	}

	if( collision && fabsf(control_trapeze_av.v) < 0.01f && fabsf(control_trapeze_rot.v) < 0.001f)
	{
		log(LOG_INFO, "collision");
		trapeze_reset(&control_trapeze_rot, 0, 0);
		trapeze_reset(&control_trapeze_av, 0, 0);
		control_state = CONTROL_READY_ASSER;
		vTaskSetEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION);
	}
}

float control_find_rotate(float debut, float fin)
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

void control_goto_near(float x, float y, float alpha, float dist, enum trajectory_way sens)
{
	log_format(LOG_INFO, "param %.2f %.2f %.2f %d", x, y, dist, sens);

	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_TRAJECTORY;
	}
	vTaskClearEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);

	control_trapeze_av.s = 0;
	control_trapeze_rot.s = 0;

	float dx = x - control_pos.x;
	float dy = y - control_pos.y;
	control_dist = sqrtf(dx*dx+dy*dy) - dist;
	control_cons = control_pos;

	if(control_dist > 0.1f)
	{
		control_trapeze_rot.v = 0;
		float a = atan2f(dy, dx);

		if(sens == TRAJECTORY_FORWARD)
		{
			control_angle = control_find_rotate(control_pos.alpha, a);
		}
		else if(sens == TRAJECTORY_BACKWARD)
		{
			control_angle = control_find_rotate(control_pos.alpha, a + PI);
			control_dist *= -1;
		}
		else
		{
			float angle_forward = control_find_rotate(control_pos.alpha, a);
			float angle_backward = control_find_rotate(control_pos.alpha, a + PI);

			if ( fabsf(angle_forward) > fabsf(angle_backward))
			{
				control_angle = angle_backward;
				control_dist *= -1;
			}
			else
			{
				control_angle = angle_forward;
			}
		}
	}
	else
	{
		control_trapeze_av.v = 0;
		control_angle = alpha - control_pos.alpha;
	}

	control_dest.alpha = control_pos.alpha + control_angle;
	control_dest.ca = cosf(control_dest.alpha);
	control_dest.sa = sinf(control_dest.alpha);
	control_dest.x = control_pos.x + control_dist * control_dest.ca;
	control_dest.y = control_pos.y + control_dist * control_dest.sa;

	control_timer = 0;
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
