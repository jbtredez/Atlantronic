#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include "kernel/module.h"
#include "kernel/rcc.h"

static struct can_msg can_servo_msg;

int servo_module_init()
{
	// pre remplissage du message can
	can_servo_msg.id = CAN_SERVO;
	can_servo_msg.format = CAN_STANDARD_FORMAT;
	can_servo_msg.type = CAN_DATA_FRAME;
	can_servo_msg.size = 2;

	return 0;
}

module_init(servo_module_init, INIT_SERVO);

void servo_set(uint8_t servo_id, uint8_t val)
{
	if(servo_id < 2)
	{
		can_servo_msg.data[servo_id] = val;
		can_write(&can_servo_msg, ms_to_tick(1));
	}
}