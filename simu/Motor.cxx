#include "Motor.h"

Motor::Motor()
{

}

Motor::~Motor()
{

}

void Motor::update()
{
/*
	// di/dt = ( u - ri - kw ) / L
	dx[0] = ( m->gain_pwm * m->pwm * m->dir - m->r * x[0] - m->k * x[2])/m->l;

	// dtheta/dt = w
	dx[1] = x[2];

	// dw/dt = (ki - cp - fw) / J
	dx[2] = (m->k * x[0] - m->cp - m->f * x[2]) / m->j;
*/
}
