#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include "kernel/module.h"
#include "bar/pwm.h"

static void can_servo(struct can_msg *msg);

int servo_module_init()
{
	can_register(CAN_SERVO, CAN_STANDARD_FORMAT, can_servo);

	return 0;
}

module_init(servo_module_init, INIT_SERVO);

void can_servo(struct can_msg *msg)
{
	if(msg->size == 2)
	{
		pwm_set(PWM_SERVO1, PWM_SERVO1_MIN + msg->data[0] * (PWM_SERVO1_MAX - PWM_SERVO1_MIN) / 255);
		pwm_set(PWM_SERVO_BALISE, PWM_SERVO1_MIN + msg->data[0] * (PWM_SERVO1_MAX - PWM_SERVO1_MIN) / 255);
	}
}