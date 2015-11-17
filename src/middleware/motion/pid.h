#ifndef PID_H
#define PID_H

//! @file pid.h
//! @brief PID control
//! @author Atlantronic

#include <stdint.h>

class Pid
{
	public:
		void init(float kp, float ki, float kd, float max);

		float compute(float error, float dt);
		void reset();

		float kp;            //!< gain proportionnel
		float ki;            //!< gain integral
		float kd;            //!< gai derive

		float max_integral;  //!< saturation de l'integrale
		float max_out;       //!< saturation de la sortie

	protected:
		float integral;      //!< valeur courante de l'integrale
		float lastError;     //!< derniere valeur pour calculer la derivee
};

#endif
