#ifndef MOTOR_H
#define MOTOR_H

//! @file Motor.h
//! @brief Model
//! @author Jean-Baptiste Trédez

#include <stdint.h>

class Motor
{
public:
	Motor();
	~Motor();

	void compute_dx(double *x, double* dx);

	float pwm;      //!< pwm
	float gain_pwm; //!< gain entre la pwm et u
	float f;        //!< coefficient de frottement dynamique du moteur
	float r;        //!< résistance du moteur
	float j;        //!< moment d'inertie du robot par rapport à l'axe de rotation du moteur
	float k;        //!< constante du moteur
	float l;        //!< inductance du moteur
	float cp;       //!< couple de pertes (frottements de la roue sur le sol + frottements / transmission)
};

#endif
