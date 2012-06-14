//! @file control.c
//! @brief Asservissement
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/math/trigo.h"
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
#include "kernel/trapeze.h"

//! @todo réglage au pif
#define CONTROL_STACK_SIZE       350
#define TRAJECTORY_POS_REACHED_TOLERANCE_X       (20 << 16)
#define TRAJECTORY_POS_REACHED_TOLERANCE_ALPHA   (0.006f * (1<<26) )

#define CONTROL_SPEED_CHECK_TOLERANCE            ((100 << 16) / CONTROL_HZ)

const int32_t CONTROL_VMAX_AV = ((800 << 16) / CONTROL_HZ);
const int32_t CONTROL_AMAX_AV = ((300 << 16) / (CONTROL_HZ * CONTROL_HZ));
const int32_t CONTROL_DMAX_AV = ((300 << 16) / (CONTROL_HZ * CONTROL_HZ));

const int32_t CONTROL_VMAX_ROT = 0.7f*(1<<26) / CONTROL_HZ; // 0.8 tr/s
const int32_t CONTROL_AMAX_ROT = 0.5f*(1<<26) / (CONTROL_HZ*CONTROL_HZ); // 1.5 tr/s²
const int32_t CONTROL_DMAX_ROT = 0.5f*(1<<26) / (CONTROL_HZ*CONTROL_HZ); // 1.5 tr/s²

static void control_task(void *);
//static int32_t sinc( int32_t x );
static void control_compute();
static void control_compute_trajectory();

// interface usb
void control_cmd_free(void* arg);
void control_cmd_param(void* arg);
void control_cmd_print_param(void* arg);
void control_cmd_set_max_speed(void* arg);

static struct control_usb_data control_usb_data;
static xSemaphoreHandle control_mutex;
static enum control_state control_state;

static struct kinematics control_kinematics;
static struct kinematics control_kinematics_cons;
static struct fx_vect_pos control_dest;

static struct fx_vect2 control_front_object;    //!< objet devant le robot
static int32_t control_front_object_approx;     //!< distance d'approche de l'objet
static struct fx_vect2 control_back_object;    //!< objet derrière le robot
static int32_t control_back_object_approx;     //!< distance d'approche de l'objet

static int disable_sick;

static struct adc_an control_an;

static portTickType control_timer;
static portTickType control_timer_stop_mes;
static int control_speed_check_error_count;

static int32_t control_kx;
static int32_t control_ky;
static int32_t control_kalpha;
static int32_t control_alpha_align;
static int32_t control_dist;

// limitation de vitesses, accélérations, décélérations
static int32_t control_vmax_av;
static int32_t control_amax_av;
static int32_t control_dmax_av;
static int32_t control_vmax_rot;
static int32_t control_amax_rot;
static int32_t control_dmax_rot;

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

#if( PWM_ARR != 2879)
#error "revoir les gains d'asservissement"
#endif

	pid_init(&control_pid_av, 80000000, 8000000, 0, PWM_ARR, 32);
	pid_init(&control_pid_rot, 1800000, 280000, 0, PWM_ARR, 26);

	control_kx = 1000;
	control_ky = 100;
	control_kalpha = 2000;

	control_vmax_av = CONTROL_VMAX_AV;
	control_amax_av = CONTROL_AMAX_AV;
	control_dmax_av = CONTROL_DMAX_AV;
	control_vmax_rot = CONTROL_VMAX_ROT;
	control_amax_rot = CONTROL_AMAX_ROT;
	control_dmax_rot = CONTROL_DMAX_ROT;

	control_state = CONTROL_READY_FREE;
	control_timer = 0;
	control_timer_stop_mes = 0;
	control_front_object_approx = 0;
	control_front_object.x = 1 << 30;
	control_front_object.y = 1 << 30;
	control_back_object.x = 1 << 30;
	control_back_object.y = 1 << 30;

	control_speed_check_error_count = 0;

	usb_add_cmd(USB_CMD_CONTROL_PARAM, &control_cmd_param);
	usb_add_cmd(USB_CMD_CONTROL_PRINT_PARAM, &control_cmd_print_param);
	usb_add_cmd(USB_CMD_CONTROL_MAX_SPEED, &control_cmd_set_max_speed);

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

	control_pid_av.kp = cmd_arg->kp_av;
	control_pid_av.ki = cmd_arg->ki_av;
	control_pid_av.kd = cmd_arg->kd_av;

	control_pid_rot.kp = cmd_arg->kp_rot;
	control_pid_rot.ki = cmd_arg->ki_rot;
	control_pid_rot.kd = cmd_arg->kd_rot;

	control_kx = cmd_arg->kx;
	control_ky = cmd_arg->ky;
	control_kalpha = cmd_arg->kalpha;
}

void control_cmd_print_param(void* arg)
{
	(void) arg;

	log_format(LOG_INFO, "av: %d %d %d (en 2^-%d)", (int)control_pid_av.kp, (int)control_pid_av.ki, (int)control_pid_av.kd, control_pid_av.fx_unit);
	log_format(LOG_INFO, "rot: %d %d %d (en 2^-%d)", (int)control_pid_rot.kp, (int)control_pid_rot.ki, (int)control_pid_rot.kd, control_pid_rot.fx_unit);
	log_format(LOG_INFO, "pos: %d %d %d", (int)control_kx, (int)control_ky, (int)control_kalpha);
}

void control_cmd_set_max_speed(void* arg)
{
	struct control_cmd_max_speed_arg* cmd_arg = (struct control_cmd_max_speed_arg*) arg;
	control_set_max_speed(cmd_arg->vmax_av, cmd_arg->vmax_rot);
}

void control_set_max_speed(uint32_t v_max_dist, uint32_t v_max_rot)
{
	v_max_dist = ((int64_t)v_max_dist * (int64_t)CONTROL_VMAX_AV) >> 16;
	v_max_rot = ((int64_t)v_max_rot * (int64_t)CONTROL_VMAX_ROT) >> 16;

	xSemaphoreTake(control_mutex, portMAX_DELAY);
	control_vmax_av = v_max_dist;
	control_vmax_rot = v_max_rot;
	xSemaphoreGive(control_mutex);
}

static int32_t control_check_speed(int32_t mes, int32_t cons, int32_t delta)
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
	int32_t u1 = 0;
	int32_t u2 = 0;

	control_kinematics = location_get_kinematics();

	xSemaphoreTake(control_mutex, portMAX_DELAY);

	// gestion de la fin de la partie : arrêt complet des moteurs quoi qu'il arrive
	if(vTaskGetEvent() & EVENT_END)
	{
		control_state = CONTROL_END;
		goto end_pwm_critical;
	}

	// mise a jour du timer "on ne bouge plus" (sur la mesure)
	if( abs(control_kinematics.v) < 6553 && abs(control_kinematics.w) < 2500)
	{
		control_timer_stop_mes += CONTROL_TICK_PERIOD;
	}
	else
	{
		control_timer_stop_mes = 0;
	}

	// recalage en marche arrière
	if( control_state == CONTROL_BACK_TO_WALL)
	{
		// on ne bouge plus depuis 300ms
		if( control_timer_stop_mes > ms_to_tick(300) )
		{
			control_state = CONTROL_READY_FREE;
			vTaskSetEvent( EVENT_CONTROL_COLSISION );
		}
		else
		{
			u1 = - PWM_ARR / 4;
			u2 = - PWM_ARR / 4;
		}

		goto end_pwm_critical;
	}

	// verification du suivit de vitesse
	int speed_check = control_check_speed(control_kinematics.v, control_kinematics_cons.v, CONTROL_SPEED_CHECK_TOLERANCE);

	if( control_state == CONTROL_TRAJECTORY)
	{
		if(speed_check)
		{
			control_speed_check_error_count++;
		}
		else
		{
			control_speed_check_error_count = 0;
		}

		if(control_speed_check_error_count > 50 )
		{
			log_format(LOG_ERROR, "erreur de suivit mes %d cons %d check %d", (int)(control_kinematics.v * CONTROL_HZ) >> 16, (int) (control_kinematics_cons.v * CONTROL_HZ) >> 16, (speed_check * CONTROL_HZ) >> 16);
			control_state = CONTROL_READY_FREE;
			vTaskSetEvent( EVENT_CONTROL_COLSISION );
			goto end_pwm_critical;
		}
		else
		{
			control_compute_trajectory();
		}
	}
	else if(control_state == CONTROL_READY_ASSER)
	{
		control_kinematics_cons.v = 0;
		control_kinematics_cons.w = 0;
	}
	else
	{
		control_kinematics_cons.v = 0;
		control_kinematics_cons.w = 0;
		goto end_pwm_critical;
	}

	// gestion du timeout (on demande de ne pas bouger pendant 2 sec avec asservissement actif)
	// condition "on ne demande pas de bouger" (donc zero pur)
	if( control_kinematics_cons.w == 0 && control_kinematics_cons.v == 0 )
	{
		// on augmente le temps avec consigne nulle
		control_timer += CONTROL_TICK_PERIOD;
		if( control_timer > ms_to_tick(2000))
		{
			// ca fait 2 seconde qu'on devrait avoir terminé la trajectoire
			// il y a un probleme
			log_format(LOG_INFO, "2 sec sans bouger => CONTROL_READY_FREE");
			control_state = CONTROL_READY_FREE;
			vTaskSetEvent( EVENT_CONTROL_TIMEOUT );
		}
		goto end_pwm_critical;
	}
	else
	{
		control_timer = 0;
	}

	// calcul de l'erreur de position dans le repère du robot
	int32_t ex = (  (int64_t)control_kinematics.ca * (int64_t)(control_kinematics_cons.x - control_kinematics.x) + (int64_t)control_kinematics.sa * (int64_t)(control_kinematics_cons.y - control_kinematics.y)) >> 30;
	int32_t ey = (- (int64_t)control_kinematics.sa * (int64_t)(control_kinematics_cons.x - control_kinematics.x) + (int64_t)control_kinematics.ca * (int64_t)(control_kinematics_cons.y - control_kinematics.y)) >> 30;
	int32_t ealpha = control_kinematics_cons.alpha - control_kinematics.alpha;

	int32_t v_c = (((int64_t)control_kinematics_cons.v * (int64_t)fx_cos(ealpha)) >> 30) + (((int64_t)control_kx * (int64_t)ex) >> 16);
	int32_t w_c = (int32_t)control_kinematics_cons.w + (((int64_t)control_kalpha * (int64_t)ealpha) >> 16) + (((((int64_t)control_ky * (int64_t)control_kinematics_cons.v) >> 16) * (int64_t)ey) >> 16);

	// régulation en vitesse
	int32_t u_rot = pid_apply(&control_pid_rot, w_c - control_kinematics.w);
	int32_t u_av = 0;

	// on prefere l'angle à l'avance en cas de saturation
	if( abs(u_rot) != PWM_ARR)
	{
		// la rotation ne prend pas toute la pwm
		u_av = pid_apply(&control_pid_av, v_c - control_kinematics.v);
		int32_t max = PWM_ARR - abs(u_rot);
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
	pwm_set(PWM_RIGHT, u1);
	pwm_set(PWM_LEFT, u2);
	control_usb_data.control_state = control_state;
	control_usb_data.control_cons_x = control_kinematics_cons.x;
	control_usb_data.control_cons_y = control_kinematics_cons.y;
	control_usb_data.control_cons_alpha = control_kinematics_cons.alpha;
	control_usb_data.control_v_dist_cons = control_kinematics_cons.v;
	control_usb_data.control_v_rot_cons = control_kinematics_cons.w;
	control_usb_data.control_pos_x = control_kinematics.x;
	control_usb_data.control_pos_y = control_kinematics.y;
	control_usb_data.control_pos_alpha = control_kinematics.alpha;
	control_usb_data.control_v_dist_mes = control_kinematics.v;
	control_usb_data.control_v_rot_mes = control_kinematics.w;
	control_usb_data.control_i_right = control_an.i_right;
	control_usb_data.control_i_left = control_an.i_left;
	control_usb_data.control_u_right = u1;
	control_usb_data.control_u_left = u2;
	xSemaphoreGive(control_mutex);

	usb_add(USB_CONTROL, &control_usb_data, sizeof(control_usb_data));
}

//! generation de trajectoire pour aller vers le point voulu
static void control_compute_trajectory()
{
	int collision = 0;

	// on s'oriente correctement
	if(control_alpha_align != control_kinematics_cons.alpha || control_kinematics_cons.w != 0)
	{
		control_speed_check_error_count = 0; // TODO pas de check pour la rotation
		control_kinematics_cons.v = 0;
		control_kinematics_cons.w = trapeze_speed_filter(control_kinematics_cons.w, control_alpha_align - control_kinematics_cons.alpha, control_amax_rot, control_dmax_rot, control_vmax_rot);
		control_kinematics_cons.alpha += control_kinematics_cons.w;
		control_kinematics_cons.ca = fx_cos(control_kinematics_cons.alpha);
		control_kinematics_cons.sa = fx_sin(control_kinematics_cons.alpha);

		if(control_kinematics_cons.alpha == control_alpha_align && control_kinematics_cons.w == 0)
		{
			int da = control_alpha_align - control_kinematics.alpha;
			if(	abs( da ) > TRAJECTORY_POS_REACHED_TOLERANCE_ALPHA)
			{
				control_state = CONTROL_READY_FREE;
				log_format(LOG_ERROR, "rotation - not reached %d", da);
				vTaskSetEvent(EVENT_CONTROL_TARGET_NOT_REACHED);
			}
		}
	}
	else if(control_dist)
	{
		int32_t ex_cons = ((int64_t)control_kinematics_cons.ca  * (int64_t)(control_dest.x - control_kinematics_cons.x) + (int64_t)control_kinematics_cons.sa * (int64_t)(control_dest.y - control_kinematics_cons.y)) >> 30;
		int32_t distance = control_dist;

		if(distance > 0)
		{
			struct fx_vect2 obj_robot;
			vect2_abs_to_loc((struct fx_vect_pos*)&control_kinematics_cons, &control_front_object, &obj_robot);
			int32_t dmin = obj_robot.x - PARAM_LEFT_CORNER_X;
			int32_t dist = dmin - control_front_object_approx;
			// on ignore les objets du robot ou derrière
			if( dmin >= 0)
			{
				if(dist < (10<<16))
				{
					dist = 0;
					collision = 1;
				}

				if(dist < ex_cons)
				{
					ex_cons = dist;
				}
			}
		}
		else
		{
			// TODO : pour l'homologation, a corriger
			int sick_state = get_sick(SICK_RIGHT | SICK_LEFT);
			if(sick_state && ! disable_sick)
			{
				collision = 1;
				ex_cons = 0;
			}
			/*struct fx_vect2 obj_robot;
			vect2_abs_to_loc((struct fx_vect_pos*)&control_kinematics_cons, &control_back_object, &obj_robot);
			int32_t dmin = obj_robot.x - PARAM_NP_X;
			int32_t dist = dmin + control_back_object_approx;
			// on ignore les objets du robot ou devant
			if( dmin <= 0)
			{
				if(dist > (10<<16))
				{
					dist = 0;
					collision = 1;
				}

				if(dist < ex_cons)
				{
					ex_cons = dist;
				}
			}*/
		}

		control_kinematics_cons.v = trapeze_speed_filter(control_kinematics_cons.v, ex_cons, control_amax_av, control_dmax_av, control_vmax_av);
		control_kinematics_cons.w = 0;
		if(control_kinematics_cons.v == ex_cons)
		{
			control_kinematics_cons.x = control_dest.x;
			control_kinematics_cons.y = control_dest.y;
		}
		else
		{
			control_kinematics_cons.x += ( (int64_t)control_kinematics_cons.v * (int64_t)control_kinematics_cons.ca) >> 30;
			control_kinematics_cons.y += ( (int64_t)control_kinematics_cons.v * (int64_t)control_kinematics_cons.sa) >> 30;
		}

		if(ex_cons == 0 && control_kinematics_cons.v == 0 && !collision)
		{
			control_dist = 0;
			control_alpha_align = control_dest.alpha;

			int32_t ex = ((int64_t)control_kinematics.ca  * (int64_t)(control_dest.x - control_kinematics.x) + (int64_t)control_kinematics.sa * (int64_t)(control_dest.y - control_kinematics.y)) >> 30;
			if( abs(ex) > TRAJECTORY_POS_REACHED_TOLERANCE_X)
			{
				control_state = CONTROL_READY_FREE;
				log_format(LOG_ERROR, "avance - not reached %d", (int)ex);
				vTaskSetEvent(EVENT_CONTROL_TARGET_NOT_REACHED);
				// TODO voir si on replanifie une traj direct (3 tentatives)
				//control_dist = ex;
			}
		}
	}
	else
	{
		control_kinematics_cons.v = 0;
		control_kinematics_cons.w = 0;
		control_state = CONTROL_READY_ASSER;
		vTaskSetEvent(EVENT_CONTROL_TARGET_REACHED);
	}

	if( collision && abs(control_kinematics_cons.v) < 655 && abs(control_kinematics_cons.w) < 10680)
	{
		control_kinematics_cons.v = 0;
		control_kinematics_cons.w = 0;
		control_state = CONTROL_READY_FREE;
		vTaskSetEvent( EVENT_CONTROL_COLSISION );
	}
}

int32_t control_find_rotate(int32_t debut, int32_t fin)
{
	int32_t alpha = fin - debut;

	// modulo 1 tour => retour dans [ 0 ; 1 tour = 2^26 [
	if(alpha < 0)
	{
		alpha = 0x4000000 - ((-alpha) & 0x3ffffff);
	}

	alpha &= 0x3ffffff;

	// retour dans [ -0.5 ; 0.5 ] tour
	if( alpha & 0x2000000 )
	{
		alpha -= 0x4000000;
	}

	return alpha;
}

void control_goto_near(int32_t x, int32_t y, int32_t alpha, int32_t dist, enum control_type type, enum trajectory_way way)
{
	log_format(LOG_INFO, "param %d %d %d %d %d %d", (int)x>>16, (int)y>>16, (int)alpha, (int)dist>>16, type, way);

	xSemaphoreTake(control_mutex, portMAX_DELAY);
//	if( control_state == CONTROL_READY_FREE)
	{
		pid_reset(&control_pid_av);
		pid_reset(&control_pid_rot);
	}

	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_TRAJECTORY;
	}
	vTaskClearEvent(EVENT_CONTROL_TARGET_REACHED | EVENT_CONTROL_TARGET_NOT_REACHED | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);

	int32_t da = 0;
	int64_t dx = x - control_kinematics.x;
	int64_t dy = y - control_kinematics.y;
	control_dist = sqrtf(dx*dx+dy*dy) - dist;
	control_kinematics_cons = control_kinematics;

	// TODO deplacer le seuil après le calcul de l'angle : si c'est en x : ok, si c'est en y => ko jusqu'à 5mm
	if(control_dist >> 16 && type != CONTROL_LINE_A)
	{
		control_kinematics_cons.w = 0;
		int32_t a = fx_atan2(dy, dx);

		if(way == TRAJECTORY_FORWARD)
		{
			da = control_find_rotate(control_kinematics.alpha, a);
		}
		else if(way == TRAJECTORY_BACKWARD)
		{
			da = control_find_rotate(control_kinematics.alpha, a + 0x2000000);
			control_dist *= -1;
		}
		else
		{
			int32_t angle_forward = control_find_rotate(control_kinematics.alpha, a);
			int32_t angle_backward = control_find_rotate(control_kinematics.alpha, a + 0x2000000);

			if ( abs(angle_forward) > abs(angle_backward))
			{
				da = angle_backward;
				control_dist *= -1;
			}
			else
			{
				da = angle_forward;
			}
		}
	}
	else
	{
		control_dist = 0;
		control_kinematics_cons.v = 0;
	}

	control_alpha_align = control_kinematics.alpha + da;
	if( type == CONTROL_LINE_XY)
	{
		// pas de rotation finale
		control_dest.alpha = control_alpha_align;
	}
	else
	{
		if( control_dist == 0 )
		{
			control_alpha_align = alpha;
		}
		control_dest.alpha = control_kinematics.alpha + control_find_rotate(control_kinematics.alpha, alpha);
	}
	control_dest.ca = fx_cos(control_alpha_align);
	control_dest.sa = fx_sin(control_alpha_align);
	control_dest.x = control_kinematics.x + (int32_t)(((int64_t)control_dist * (int64_t)control_dest.ca) >> 30);
	control_dest.y = control_kinematics.y + (int32_t)(((int64_t)control_dist * (int64_t)control_dest.sa) >> 30);

	control_timer = 0;
	control_timer_stop_mes = 0;
	xSemaphoreGive(control_mutex);
}

void control_back_to_wall()
{
	log(LOG_INFO, "back to wall");
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_BACK_TO_WALL;
	}

	control_timer_stop_mes = 0;
	xSemaphoreGive(control_mutex);
}

int32_t control_get_state()
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	int32_t tmp = control_state;
	xSemaphoreGive(control_mutex);
	return tmp;
}

void control_free()
{
	log(LOG_INFO, "free wheel");
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_READY_FREE;
	}
	xSemaphoreGive(control_mutex);
}

void control_set_front_object(struct fx_vect2* a, int32_t approx_dist)
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	control_front_object = *a;
	control_front_object_approx = approx_dist;
	xSemaphoreGive(control_mutex);
}

void control_set_back_object(struct fx_vect2* a, int32_t approx_dist)
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	control_back_object = *a;
	control_back_object_approx = approx_dist;
	xSemaphoreGive(control_mutex);
}

void control_disable_sick()
{
	disable_sick = 1;
}

void control_enable_sick()
{
	disable_sick = 0;
}