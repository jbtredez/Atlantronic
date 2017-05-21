#ifndef SRC_DISCO_STAR_SERVOS_H_
#define SRC_DISCO_STAR_SERVOS_H_

#include "star.h"


enum Net_Trap_state
{
	NET_TRAP_CLOSE,		// Fermé dans le robot
	NET_TRAP_OPEN,		// Ouvert

};


enum ServosWaitPolicy
{
	SERVO_POLICY_NON_BLOCKING,
	SERVO_POLICY_WAIT_END,
};


class Servos
{
	public:
	static void setTorque(bool enable);
	static void closeAll(void);
	static int setAngle(Dynamixel *servo, float angle, enum ServosWaitPolicy);
	static void setNetState(enum Net_Trap_state NetState);
	};

#endif /* SRC_DISCO_STAR_SERVOS_H_ */
