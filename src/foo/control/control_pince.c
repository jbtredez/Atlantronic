//! @file control_pince.c
//! @brief Asservissement des pinces (en hauteur)
//! @author Atlantronic

#include <math.h>
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control_pince.h"
#include "priority.h"
#include "pwm.h"
#include "control/pid.h"
#include "kernel/trapeze.h"
#include "kernel/robot_parameters.h"
#include "kernel/event.h"
#include "adc.h"

//! @todo réglage au pif
#define CONTROL_PINCE_STACK_SIZE       150

//! période de la tache de propulsion en tick ("fréquence" de l'asservissement)
#define CONTROL_PINCE_TICK_PERIOD        ms_to_tick(5)

#define TE                         (float) ((float)CONTROL_PINCE_TICK_PERIOD) / ((float)RCC_SYSCLK)

static void control_pince_task(void *);
static void control_pince_compute();
static int32_t control_pince_state;

struct {
	struct pid pid_ind_height1;
	struct pid pid_ind_height2;
	struct trapeze ind_trapeze_right;
	struct trapeze ind_trapeze_left;
	float ind_cons1;
	float ind_cons2;
	struct pid pid_dual_height;
	struct pid pid_dual_alpha;
	struct trapeze dual_trapeze_height;
	struct trapeze dual_trapeze_angle;
	float dual_cons_h;
	float dual_cons_a;
}control_pince_param;

struct adc_an control_pince_an;

static int control_pince_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(control_pince_task, "ctrl_p", CONTROL_PINCE_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL_PINCE, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL_PINCE;
	}

	pid_init(&control_pince_param.pid_ind_height1, 5, 0, 0, PWM_ARR);
	pid_init(&control_pince_param.pid_ind_height2, 5, 0, 0, PWM_ARR);
	pid_init(&control_pince_param.pid_dual_height, 4, 0, 0, PWM_ARR);
	pid_init(&control_pince_param.pid_dual_alpha, 4, 0, 0, PWM_ARR);

	trapeze_set(&control_pince_param.ind_trapeze_right, 4000.0f*TE, 16000.0f*TE*TE);
	trapeze_set(&control_pince_param.ind_trapeze_left, 4000.0f*TE, 16000.0f*TE*TE);
	trapeze_set(&control_pince_param.dual_trapeze_height, 4000.0f*TE, 16000.0f*TE*TE);
	trapeze_set(&control_pince_param.dual_trapeze_angle, 4000.0f*TE, 16000.0f*TE*TE);

	control_pince_state = CONTROL_PINCE_READY_FREE;

	return 0;
}

module_init(control_pince_module_init, INIT_CONTROL_PINCE);

static void control_pince_task(void* arg)
{
	(void) arg;

	portTickType wake_time = systick_get_time();

	while(1)
	{
		control_pince_compute();

		wake_time += CONTROL_PINCE_TICK_PERIOD;
		vTaskDelayUntil(wake_time);
	}
}

static void control_pince_compute()
{
	float u1 = 0;
	float u2 = 0;

	int sens1 = 1;
	int sens2 = 1;

// TODO mutex pour laisser les IT
	portENTER_CRITICAL();
	adc_get(&control_pince_an);

	if(vTaskGetEvent() & EVENT_END)
	{
		control_pince_state = CONTROL_PINCE_END;
		goto end_pwm_critical;
	}

	if( control_pince_an.i3 > 500 || control_pince_an.i4 > 500)
	{
		control_pince_state = CONTROL_PINCE_READY_FREE;
		vTaskSetEvent(EVENT_CONTROL_PINCE_COLISION | EVENT_CONTROL_PINCE_READY);
		goto end_pwm_critical;
	}

	// calcul du prochain point
	switch(control_pince_state)
	{
		case CONTROL_PINCE_READY_FREE:
			goto end_pwm_critical;
			break;
		case CONTROL_PINCE_READY_ASSER:
			break;
		case CONTROL_PINCE_DUAL:
			{
				// régulation en position
				trapeze_apply(&control_pince_param.dual_trapeze_height, control_pince_param.dual_cons_h);
				trapeze_apply(&control_pince_param.dual_trapeze_angle, control_pince_param.dual_cons_a);

				float eh = control_pince_param.dual_trapeze_height.s - 0.5f * ((int16_t)control_pince_an.potard_right + (int16_t)control_pince_an.potard_left);
				float ea = control_pince_param.dual_trapeze_angle.s - 0.5f * ((int16_t)control_pince_an.potard_right - (int16_t)control_pince_an.potard_left);

				float uh = pid_apply(&control_pince_param.pid_dual_height, eh);
				float ua = pid_apply(&control_pince_param.pid_dual_alpha, ea);

				// on prefere l'angle à la hauteur en cas de saturation
				if( ua > PWM_ARR)
				{
					ua = PWM_ARR;
					uh = 0;
				}
				else if( ua < - PWM_ARR)
				{
					ua = - PWM_ARR;
					uh = 0;
				}
				else
				{
					// la rotation ne prend pas toute la pwm
					float max = PWM_ARR - fabs(ua);
					if( uh > max)
					{
						uh = max;
					}
					else if( uh < -max)
					{
						uh = -max;
					}
				}
				u1 = uh + ua;
				u2 = uh - ua;
			}
			break;
		case CONTROL_PINCE_INDEPENDANT:
			{
				// régulation en position
				trapeze_apply(&control_pince_param.ind_trapeze_right, control_pince_param.ind_cons1);
				trapeze_apply(&control_pince_param.ind_trapeze_left, control_pince_param.ind_cons2);

				float e1 = control_pince_param.ind_trapeze_right.s - control_pince_an.potard_right;
				float e2 = control_pince_param.ind_trapeze_left.s - control_pince_an.potard_left;

				u1 = pid_apply(&control_pince_param.pid_ind_height1, e1);
				u2 = pid_apply(&control_pince_param.pid_ind_height2, e2);
			}
			break;
		case CONTROL_PINCE_END:
			goto end_pwm_critical;
			break;
		default:
			// erreur de prog ou corruption mem
			goto end_pwm_critical;
			break;
	}

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

	if(u1 > PWM_ARR)
	{
		u1 = PWM_ARR;
	}

	if(u2 > PWM_ARR)
	{
		u2 = PWM_ARR;
	}

end_pwm_critical:
	pwm_set(PWM_UP_RIGHT, (uint32_t)u1, sens1);
	pwm_set(PWM_UP_LEFT, (uint32_t)u2, sens2);
	portEXIT_CRITICAL();
}

void control_pince_independant(float h1, float h2)
{
	portENTER_CRITICAL();
	if(control_pince_state != CONTROL_PINCE_END)
	{
		control_pince_state = CONTROL_PINCE_INDEPENDANT;
	}
	vTaskClearEvent(EVENT_CONTROL_PINCE_COLISION | EVENT_CONTROL_PINCE_READY);
	trapeze_reset(&control_pince_param.ind_trapeze_right, control_pince_an.potard_right, 0);
	trapeze_reset(&control_pince_param.ind_trapeze_left, control_pince_an.potard_left, 0);
	control_pince_param.ind_cons1 = h1;
	control_pince_param.ind_cons2 = h2;
	portEXIT_CRITICAL();
}

void control_pince_dual(float h, float alpha)
{
	portENTER_CRITICAL();
	if(control_pince_state != CONTROL_PINCE_END)
	{
		control_pince_state = CONTROL_PINCE_DUAL;
	}
	vTaskClearEvent(EVENT_CONTROL_PINCE_COLISION | EVENT_CONTROL_PINCE_READY);
	trapeze_reset(&control_pince_param.dual_trapeze_height, 0.5f * ((int16_t)control_pince_an.potard_right + (int16_t)control_pince_an.potard_left), 0);
	trapeze_reset(&control_pince_param.dual_trapeze_angle, 0.5f * ((int16_t)control_pince_an.potard_right - (int16_t)control_pince_an.potard_left), 0);
	control_pince_param.dual_cons_h = h;
	control_pince_param.dual_cons_a = alpha;
	portEXIT_CRITICAL();
}

int32_t control_pince_get_state()
{
	portENTER_CRITICAL();
	int32_t tmp = control_pince_state;
	portEXIT_CRITICAL();
	return tmp;
}

void control_pince_free()
{
	portENTER_CRITICAL();
	if(control_pince_state != CONTROL_PINCE_END)
	{
		control_pince_state = CONTROL_PINCE_READY_FREE;
		vTaskSetEvent(EVENT_CONTROL_PINCE_READY);
	}
	portEXIT_CRITICAL();
}
