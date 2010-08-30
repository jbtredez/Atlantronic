//! @file model.c
//! @brief Model of the robot
//! @author Jean-Baptiste Trédez

#include <assert.h>
#include <math.h>
#include <memory.h>
#include "simu/model.h"
#include "simu/model_motor.h"
#include "io/encoders.h"
#include "io/pwm.h"
#include "module.h"
#include "log.h"
#include "time2.h"
#include "vect_pos.h"
#include "robot_parameters.h"
#include "rtos/FreeRTOSConfig.h"

static struct model_motor model_motors[PWM_NB];
static unsigned long model_time;

//! state
static double X[9];
enum
{
	MODEL_MOT_RIGHT_I,
	MODEL_MOT_RIGHT_THETA,
	MODEL_MOT_RIGHT_W,
	MODEL_MOT_LEFT_I,
	MODEL_MOT_LEFT_THETA,
	MODEL_MOT_LEFT_W,
	MODEL_POS_X,
	MODEL_POS_Y,
	MODEL_POS_ALPHA
};

#define MODEL_FREQ_MULT     10

static int modele_module_init()
{
	memset(X, 0x00, sizeof(X));
	model_motors[0].pwm  = 0;
	model_motors[0].dir = 1;
	model_motors[0].gain_pwm = 24.0f/65536;
	model_motors[0].f  = 0;
	model_motors[0].r  = 1.1;
	model_motors[0].j  = 0.0001;
	model_motors[0].k  = 0.0363;
	model_motors[0].l  = 201e-6;
	model_motors[0].cp = 0;

	model_motors[1] = model_motors[0];

	model_time = 0;

	return 0;
}

module_init(modele_module_init, INIT_MODEL);

void model_dx(double *x, double* dx)
{
	model_motor_dx(&model_motors[PWM_RIGHT], x, dx);
	model_motor_dx(&model_motors[PWM_RIGHT], x+3, dx+3);

	double v_right = x[MODEL_MOT_RIGHT_W] * PARAM_RIGHT_MOT_WHEEL_RADIUS * PARAM_RIGHT_MOT_WHEEL_WAY;
	double v_left = x[MODEL_MOT_LEFT_W] * PARAM_LEFT_MOT_WHEEL_RADIUS  * PARAM_LEFT_MOT_WHEEL_WAY;

	double v_d = PARAM_DIST_MOD_GAIN * (v_right + v_left);
	double v_r = PARAM_ROT_MOD_GAIN * (v_right - v_left);

	dx[MODEL_POS_X] = v_d * cos(x[MODEL_POS_ALPHA]);
	dx[MODEL_POS_Y] = v_d * sin(x[MODEL_POS_ALPHA]);
	dx[MODEL_POS_ALPHA] = v_r;
}

void model_update()
{
	const double te = 1.0f/(configTICK_RATE_HZ * MODEL_FREQ_MULT);
	unsigned long t = time_sys();
	int i,j;

	double x[9];
	double k1[9];
	double k2[9];
	double k3[9];
	double k4[9];

	portENTER_CRITICAL();

	while(model_time < t)
	{
		for(i=0; i<MODEL_FREQ_MULT; i++)
		{
			// intégration runge-kutta 4
			model_dx(X, k1);

			for(j=0; j<9; j++)
			{
				x[j] = X[j] + k1[j] * te / 2;
			}
			model_dx(x, k2);

			for(j=0; j<9; j++)
			{
				x[j] = X[j] + k2[j] * te / 2;
			}
			model_dx(x, k3);

			for(j=0; j<9; j++)
			{
				x[j] = X[j] + k3[j] * te;
			}
			model_dx(x, k4);

			for(j=0; j<9; j++)
			{
				X[j] += (k1[j] + 2* k2[j] + 2*k3[j] + k4[j]) * te / 6;
			}
		}

		model_time++;
	}

	portEXIT_CRITICAL();
}

void model_pwm_set(unsigned int num, uint32_t val, int dir)
{
	assert(num < PWM_NB);
	portENTER_CRITICAL();
	model_update();
	model_motors[num].pwm = val;
	model_motors[num].dir = dir;
	portEXIT_CRITICAL();
}

uint16_t model_encoders_get(unsigned int num)
{
	uint16_t rep;

	portENTER_CRITICAL();
	model_update();
	switch(num)
	{
		case PWM_RIGHT:
			rep = (uint16_t) ((X[1] * PARAM_ODO_RED * PARAM_ENCODERS_RES * PARAM_RIGHT_MOT_WHEEL_RADIUS * PARAM_RIGHT_MOT_WHEEL_WAY * PARAM_RIGHT_ODO_WHEEL_WAY) / (2 * M_PI * PARAM_MOT_RED * PARAM_RIGHT_ODO_WHEEL_RADIUS));
			break;
		case PWM_LEFT:
			rep = (uint16_t) ((X[4] * PARAM_ODO_RED * PARAM_ENCODERS_RES * PARAM_LEFT_MOT_WHEEL_RADIUS * PARAM_LEFT_MOT_WHEEL_WAY * PARAM_LEFT_ODO_WHEEL_WAY) / (2 * M_PI * PARAM_MOT_RED * PARAM_LEFT_ODO_WHEEL_RADIUS));
			break;
		default:
			rep = 0;
			meslog(_erreur_, 0, "num = %d", num);
			break;
	}
	portEXIT_CRITICAL();

	return rep;
}

