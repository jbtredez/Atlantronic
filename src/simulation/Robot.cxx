#include "Robot.h"
#include "robot_parameters.h"
#include <memory.h>

//! Multiplicateur de la fréquence du noyau
//! La fréquence du modèle 1000 * MODEL_FREQ_MULT Hz
#define MODEL_FREQ_MULT     10

Robot::Robot(EnvironnementInterface* e) :
	env(e),
	cpu(this)
{
	memset(X, 0x00, sizeof(X));
	model_time = 0;
}

Robot::~Robot()
{

}

void Robot::start()
{
	cpu.start();
}

void Robot::update(	uint64_t vm_clk )
{
	const double te = 1.0f/(1000 * MODEL_FREQ_MULT);

	int i,j;

	double x[MODEL_SIZE];
	double k1[MODEL_SIZE];
	double k2[MODEL_SIZE];
	double k3[MODEL_SIZE];
	double k4[MODEL_SIZE];

	motor[0].pwm = cpu.TIM1.getPwm(0);
	motor[1].pwm = cpu.TIM1.getPwm(1);

	while(model_time < vm_clk)
	{
		for(i=0; i<MODEL_FREQ_MULT; i++)
		{
			// intégration runge-kutta 4
			compute_dx(X, k1);

			for(j=0; j<MODEL_SIZE; j++)
			{
				x[j] = X[j] + k1[j] * te / 2;
			}
			compute_dx(x, k2);

			for(j=0; j<MODEL_SIZE; j++)
			{
				x[j] = X[j] + k2[j] * te / 2;
			}
			compute_dx(x, k3);

			for(j=0; j<MODEL_SIZE; j++)
			{
				x[j] = X[j] + k3[j] * te;
			}
			compute_dx(x, k4);

			for(j=0; j<MODEL_SIZE; j++)
			{
				X[j] += (k1[j] + 2* k2[j] + 2*k3[j] + k4[j]) * te / 6;
			}
		}
		model_time++;

//		fprintf(model_log_file, "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", model_time, X[MODEL_MOT_RIGHT_I], X[MODEL_MOT_RIGHT_THETA], X[MODEL_MOT_RIGHT_W], X[MODEL_MOT_LEFT_I], X[MODEL_MOT_LEFT_THETA], X[MODEL_MOT_LEFT_W], X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA], X[MODEL_ODO_RIGHT_THETA], X[MODEL_ODO_LEFT_THETA]);
//		printf( "%lu\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", model_time, X[MODEL_MOT_RIGHT_I], X[MODEL_MOT_RIGHT_THETA], X[MODEL_MOT_RIGHT_W], X[MODEL_MOT_LEFT_I], X[MODEL_MOT_LEFT_THETA], X[MODEL_MOT_LEFT_W], X[MODEL_POS_X], X[MODEL_POS_Y], X[MODEL_POS_ALPHA], X[MODEL_ODO_RIGHT_THETA], X[MODEL_ODO_LEFT_THETA]);
	}

	env->update();

	cpu.TIM3.setEncoder((uint16_t) X[MODEL_ODO_RIGHT_THETA]);
	cpu.TIM4.setEncoder((uint16_t) X[MODEL_ODO_LEFT_THETA]);
//	printf("codeurs : %i     %i\n", (uint16_t) X[MODEL_ODO_RIGHT_THETA], (uint16_t) X[MODEL_ODO_LEFT_THETA]);
}

void Robot::compute_dx(double *x, double* dx)
{
	motor[0].compute_dx(x, dx); // moteur droit
	motor[1].compute_dx(x+3, dx+3); // moteur gauche

	double v_right = x[MODEL_MOT_RIGHT_W] * PARAM_RIGHT_MOT_WHEEL_RADIUS * PARAM_RIGHT_MOT_WHEEL_WAY;
	double v_left = x[MODEL_MOT_LEFT_W] * PARAM_LEFT_MOT_WHEEL_RADIUS  * PARAM_LEFT_MOT_WHEEL_WAY;

	double v_d = PARAM_DIST_MOD_GAIN * (v_right + v_left);
	double v_r = PARAM_ROT_MOD_GAIN * (v_right - v_left);

	dx[MODEL_POS_X] = v_d * cos(x[MODEL_POS_ALPHA]);
	dx[MODEL_POS_Y] = v_d * sin(x[MODEL_POS_ALPHA]);
	dx[MODEL_POS_ALPHA] = v_r;

	dx[MODEL_ODO_RIGHT_THETA] = (v_d / PARAM_DIST_ODO_GAIN + v_r / PARAM_ROT_ODO_GAIN )/(2 * PARAM_RIGHT_ODO_WHEEL_RADIUS * PARAM_RIGHT_ODO_WHEEL_WAY);
	dx[MODEL_ODO_LEFT_THETA] = (v_d / PARAM_DIST_ODO_GAIN - v_r / PARAM_ROT_ODO_GAIN )/(2 * PARAM_LEFT_ODO_WHEEL_RADIUS  * PARAM_LEFT_ODO_WHEEL_WAY);
}
