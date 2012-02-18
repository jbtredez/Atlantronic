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
#define TRAJECTORY_POS_REACHED_TOLERANCE_X       (2 << 16)
#define TRAJECTORY_POS_REACHED_TOLERANCE_ALPHA   (0.02f * (1<<26) / ( 2 * 3.141592654f ))

#define CONTROL_SPEED_CHECK_TOLERANCE            ((100 << 16) / CONTROL_HZ)

const int32_t CONTROL_VMAX_AV = ((800 << 16) / CONTROL_HZ);
const int32_t CONTROL_AMAX_AV = ((600 << 16) / (CONTROL_HZ * CONTROL_HZ));
const int32_t CONTROL_DMAX_AV = ((1000 << 16) / (CONTROL_HZ * CONTROL_HZ));

const int32_t CONTROL_VMAX_ROT = ((((int64_t)100 << 26) / CONTROL_HZ)  / ( PI*PARAM_VOIE_MOT ));
const int32_t CONTROL_AMAX_ROT = ((((int64_t)125 << 26) / (CONTROL_HZ * CONTROL_HZ))  / ( PI*PARAM_VOIE_MOT ));
const int32_t CONTROL_DMAX_ROT = ((((int64_t)250 << 26) / (CONTROL_HZ * CONTROL_HZ))  / ( PI*PARAM_VOIE_MOT ));

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
static struct fx_vect_pos control_cons;
static struct fx_vect_pos control_dest;

static struct fx_vect2 control_front_object;    //!< objet devant le robot
static int32_t control_front_object_approx;     //!< distance d'approche de l'objet

static struct adc_an control_an;

static portTickType control_timer;
static int control_speed_check_error_count;

static int32_t control_kx;
static int32_t control_ky;
static int32_t control_kalpha;
static struct trapeze control_trapeze_rot;
static struct trapeze control_trapeze_av;
static int32_t control_angle;
static int32_t control_dist;

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

	pid_init(&control_pid_av, 60000000, 15000000, 0, PWM_ARR, 32);
	pid_init(&control_pid_rot, 3000000, 500000, 0, PWM_ARR, 26);

	control_kx = 0;
	control_ky = 0;
	control_kalpha = 0;

	trapeze_reset(&control_trapeze_av, 0, 0);
	trapeze_reset(&control_trapeze_rot, 0, 0);

	control_trapeze_av.v_max = CONTROL_VMAX_AV;
	control_trapeze_av.a_max = CONTROL_AMAX_AV;
	control_trapeze_av.d_max = CONTROL_DMAX_AV;
	control_trapeze_rot.v_max = CONTROL_VMAX_ROT;
	control_trapeze_rot.a_max = CONTROL_AMAX_ROT;
	control_trapeze_rot.d_max = CONTROL_DMAX_ROT;

	control_state = CONTROL_READY_FREE;
	control_timer = 0;
	control_front_object_approx = 0;
	control_front_object.x = 1 << 30;
	control_front_object.y = 1 << 30;

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

#if 0
static int32_t sinc( int32_t x )
{
	if( abs(x) < 1068070 ) // < 0.1 rd
	{
		return 1.0f;
	}
	else
	{
		int64_t sx = fx_sin(x);
		sx <<= 26;
		return sx / x;
	}
}
#endif

#if 0
static void control_recalage()
{
	int32_t alpha = control_pos.alpha;

	// modulo 1 tour => retour dans [ 0 ; 1 tour = 2^26 [
	if(alpha < 0)
	{
		alpha = 0x4000000 - ((-alpha) & 0x3ffffff);
	}
	else
	{
		alpha &= 0x3ffffff;
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

void control_set_max_speed(uint32_t v_max_dist, uint32_t v_max_rot)
{
	v_max_dist = ((int64_t)v_max_dist * (int64_t)CONTROL_VMAX_AV) >> 16;
	v_max_rot = ((int64_t)v_max_rot * (int64_t)CONTROL_VMAX_ROT) >> 16;

	xSemaphoreTake(control_mutex, portMAX_DELAY);
	control_trapeze_av.v_max = v_max_dist;
	control_trapeze_rot.v_max = v_max_rot;
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

	if(vTaskGetEvent() & EVENT_END)
	{
		control_state = CONTROL_END;
		goto end_pwm_critical;
	}

	// verification du suivit de vitesse
	// TODO : ajouter un timer : remise à zero si ok, incrementer si ko et si depassement => retourner ko
	int speed_check = control_check_speed(control_kinematics.v, control_trapeze_av.v, CONTROL_SPEED_CHECK_TOLERANCE);

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

		if(control_speed_check_error_count > 10 )
		{
			log_format(LOG_ERROR, "erreur de suivit mes %d cons %d check %d", (int)(control_kinematics.v * CONTROL_HZ) >> 16, (int) (control_trapeze_av.v * CONTROL_HZ) >> 16, (speed_check * CONTROL_HZ) >> 16);
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
			vTaskSetEvent( EVENT_CONTROL_TIMEOUT );
		}
		goto end_pwm_critical;
	}
	else
	{
		control_timer = 0;
	}

	// calcul de l'erreur de position dans le repère du robot
	int32_t ex = (  (int64_t)control_kinematics.ca * (int64_t)(control_cons.x - control_kinematics.x) + (int64_t)control_kinematics.sa * (int64_t)(control_cons.y - control_kinematics.y)) >> 30;
//	int32_t ey = (- (int64_t)control_kinematics.sa * (int64_t)(control_cons.x - control_kinematics.x) + (int64_t)control_kinematics.ca * (int64_t)(control_cons.y - control_kinematics.y)) >> 30;
	int32_t ealpha = control_cons.alpha - control_kinematics.alpha;

	int32_t v_c = (((int64_t)control_trapeze_av.v * (int64_t)fx_cos(ealpha)) >> 30) + (((int64_t)control_kx * (int64_t)ex) >> 16);
	int32_t w_c = (int32_t)control_trapeze_rot.v + (((int64_t)control_kalpha * (int64_t)ealpha) >> 16);// + control_ky * control_trapeze_av.v * sinc(ealpha) * ey;

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
	control_usb_data.control_cons_x = control_cons.x;
	control_usb_data.control_cons_y = control_cons.y;
	control_usb_data.control_cons_alpha = control_cons.alpha;
	control_usb_data.control_pos_x = control_kinematics.x;
	control_usb_data.control_pos_y = control_kinematics.y;
	control_usb_data.control_pos_alpha = control_kinematics.alpha;
	control_usb_data.control_v_dist_cons = control_trapeze_av.v;
	control_usb_data.control_v_rot_cons = control_trapeze_rot.v;
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

	if(control_angle)
	{
		trapeze_apply(&control_trapeze_rot, control_angle);
		control_cons.alpha += control_trapeze_rot.v;
		control_cons.ca = fx_cos(control_cons.alpha);
		control_cons.sa = fx_sin(control_cons.alpha);

		if(control_trapeze_rot.s == control_angle && control_trapeze_rot.v == 0)
		{
			control_angle = 0;
			control_cons.alpha = control_dest.alpha;
			control_cons.ca = control_dest.ca;
			control_cons.sa = control_dest.sa;
			if(	abs( control_dest.alpha - control_kinematics.alpha ) > TRAJECTORY_POS_REACHED_TOLERANCE_ALPHA)
			{
				trapeze_reset(&control_trapeze_rot, 0, 0);
				trapeze_reset(&control_trapeze_av, 0, 0);
				control_state = CONTROL_READY_FREE;
				vTaskSetEvent(EVENT_CONTROL_TARGET_NOT_REACHED);
			}
		}
	}
	else if(control_dist)
	{
		int32_t ex = ((int64_t)control_kinematics.ca  * (int64_t)(control_dest.x - control_kinematics.x) + (int64_t)control_kinematics.sa * (int64_t)(control_dest.y - control_kinematics.y)) >> 30;
		int32_t distance = control_dist;

		if(distance > 0)
		{
			struct fx_vect2 obj_robot;
			fx_vect2_table_to_robot(&control_cons, &control_front_object, &obj_robot);
			int32_t dist = obj_robot.x - PARAM_LEFT_CORNER_X - control_front_object_approx;
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

		control_cons.x += ( (int64_t)control_trapeze_av.v * (int64_t)control_cons.ca) >> 30;
		control_cons.y += ( (int64_t)control_trapeze_av.v * (int64_t)control_cons.sa) >> 30;

		if(control_trapeze_av.s == control_dist && control_trapeze_av.v == 0)
		{
			control_dist = 0;
			control_cons = control_dest;
			trapeze_reset(&control_trapeze_av, 0, 0);

			if( abs(ex) > TRAJECTORY_POS_REACHED_TOLERANCE_X)
			{
				trapeze_reset(&control_trapeze_rot, 0, 0);
				trapeze_reset(&control_trapeze_av, 0, 0);
				control_state = CONTROL_READY_FREE;
				vTaskSetEvent(EVENT_CONTROL_TARGET_NOT_REACHED);
			}
		}
	}
	else
	{
		trapeze_reset(&control_trapeze_rot, 0, 0);
		trapeze_reset(&control_trapeze_av, 0, 0);
		control_state = CONTROL_READY_ASSER;
		vTaskSetEvent(EVENT_CONTROL_TARGET_REACHED);
	}

	if( collision && abs(control_trapeze_av.v) < 655 && abs(control_trapeze_rot.v) < 10680)
	{
		trapeze_reset(&control_trapeze_rot, 0, 0);
		trapeze_reset(&control_trapeze_av, 0, 0);
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
	else
	{
		alpha &= 0x3ffffff;
	}

	// retour dans [ -0.5 ; 0.5 ] tour
	if( alpha & 0x2000000 )
	{
		alpha -= 0x4000000;
	}

	return alpha;
}

void control_goto_near(int32_t x, int32_t y, int32_t alpha, int32_t dist, enum trajectory_way sens)
{
	log_format(LOG_INFO, "param %d %d %d %d %d", (int)x>>16, (int)y>>16, (int)alpha, (int)dist>>16, sens);

	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if( control_state == CONTROL_READY_FREE)
	{
		pid_reset(&control_pid_av);
		pid_reset(&control_pid_rot);
	}

	if(control_state != CONTROL_END)
	{
		control_state = CONTROL_TRAJECTORY;
	}
	vTaskClearEvent(EVENT_CONTROL_TARGET_REACHED | EVENT_CONTROL_TARGET_NOT_REACHED | EVENT_CONTROL_COLSISION | EVENT_CONTROL_TIMEOUT);

	control_trapeze_av.s = 0;
	control_trapeze_rot.s = 0;

	int64_t dx = x - control_kinematics.x;
	int64_t dy = y - control_kinematics.y;
	control_dist = sqrtf(dx*dx+dy*dy) - dist;
	control_cons.x = control_kinematics.x;
	control_cons.y = control_kinematics.y;
	control_cons.alpha = control_kinematics.alpha;
	control_cons.ca = control_kinematics.ca;
	control_cons.sa = control_kinematics.sa;

	if(control_dist >> 16)
	{
		control_trapeze_rot.v = 0;
		int32_t a = fx_atan2(dy, dx);

		if(sens == TRAJECTORY_FORWARD)
		{
			control_angle = control_find_rotate(control_kinematics.alpha, a);
		}
		else if(sens == TRAJECTORY_BACKWARD)
		{
			control_angle = control_find_rotate(control_kinematics.alpha, a + 0x2000000);
			control_dist *= -1;
		}
		else
		{
			int32_t angle_forward = control_find_rotate(control_kinematics.alpha, a);
			int32_t angle_backward = control_find_rotate(control_kinematics.alpha, a + 0x2000000);

			if ( abs(angle_forward) > abs(angle_backward))
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
		control_angle = alpha - control_kinematics.alpha;
	}

	control_dest.alpha = control_kinematics.alpha + control_angle;
	control_dest.ca = fx_cos(control_dest.alpha);
	control_dest.sa = fx_sin(control_dest.alpha);
	control_dest.x = control_kinematics.x + (int32_t)(((int64_t)control_dist * (int64_t)control_dest.ca) >> 30);
	control_dest.y = control_kinematics.y + (int32_t)(((int64_t)control_dist * (int64_t)control_dest.sa) >> 30);

	control_timer = 0;
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