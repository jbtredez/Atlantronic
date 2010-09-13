//! @file model_motor.h
//! @brief Model
//! @author Jean-Baptiste Trédez

#include <stdint.h>

struct model_motor
{
	uint32_t pwm;   //!< pwm
	int dir;        //!< direction
	float gain_pwm; //!< gain entre la pwm et u
	float f;        //!< coefficient de frottement dynamique du moteur
	float r;        //!< résistance du moteur
	float j;        //!< moment d'inertie du robot par rapport à l'axe de rotation du moteur
	float k;        //!< constante du moteur
	float l;        //!< inductance du moteur
	float cp;       //!< couple de pertes (frottements de la roue sur le sol + frottements / transmission)
};

//! Calcule la dérivée de l'état du moteur à partir de l'état et des paramètres du moteur
//! La dérivée est calculée par unité de temps (dt = 1)
//!
//! @param m moteur
//! @param x etat du moteur (i, theta, w)
//! @param dx réponse : dérivée de l'état par unité de temps
void model_motor_dx(struct model_motor *m, double *x, double *dx);
