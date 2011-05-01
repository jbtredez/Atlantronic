#include "Motor.h"

Motor::Motor()
{
	pwm  = 0;
	gain_pwm = 24.0f;
	f  = 0;
	r  = 1.1;
	j  = 0.0001;
	k  = 0.0363;
	l  = 201e-6;
	cp = 0;
}

Motor::~Motor()
{

}

//! Calcule la dérivée de l'état du moteur à partir de l'état et des paramètres du moteur
//! La dérivée est calculée par unité de temps (dt = 1)
//!
//! @param x etat du moteur (i, theta, w)
//! @param dx réponse : dérivée de l'état par unité de temps
void Motor::compute_dx(double *x, double* dx)
{
	// di/dt = ( u - ri - kw ) / L
	dx[0] = ( gain_pwm * pwm - r * x[0] - k * x[2]) / l;

	// dtheta/dt = w
	dx[1] = x[2];

	// dw/dt = (ki - cp - fw) / J
	dx[2] = (k * x[0] - cp - f * x[2]) / j;
}
