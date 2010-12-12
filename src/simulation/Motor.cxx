#include "Motor.h"

Motor::Motor()
{

}

Motor::~Motor()
{

}

//! Calcule la dérivée de l'état du moteur à partir de l'état et des paramètres du moteur
//! La dérivée est calculée par unité de temps (dt = 1)
//!
//! @param x etat du moteur (i, theta, w)
//! @param dx réponse : dérivée de l'état par unité de temps
void Motor::update_dx(double *x, double* dx)
{
	// di/dt = ( u - ri - kw ) / L
	dx[0] = ( gain_pwm * pwm * dir - r * x[0] - k * x[2]) / l;

	// dtheta/dt = w
	dx[1] = x[2];

	// dw/dt = (ki - cp - fw) / J
	dx[2] = (k * x[0] - cp - f * x[2]) / j;
}
