#ifndef SRC_DISCO_STAR_SERVOS_H_
#define SRC_DISCO_STAR_SERVOS_H_

#include "disco/gate/gate.h"

enum ServosWaitPolicy
{
	SERVO_POLICY_NON_BLOCKING = 0,
	SERVO_POLICY_WAIT_END,
};


class Servos
{
	public:
	static void setTorque(bool enable);
	static void closeAll(void);
	static int setAngle(Dynamixel *servo, float angle, enum ServosWaitPolicy);
};

#endif /* SRC_DISCO_STAR_SERVOS_H_ */
