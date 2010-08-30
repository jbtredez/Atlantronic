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

//! @param x state (i, theta, w) 
void model_motor_dx(struct model_motor *m, double *x, double *dx);
