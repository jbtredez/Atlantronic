#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include <stdint.h>
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"

enum
{
	CAN_MOTOR_DRIVING1 = 0,
	CAN_MOTOR_STEERING1,
	CAN_MOTOR_DRIVING2,
	CAN_MOTOR_STEERING2,
	CAN_MOTOR_DRIVING3,
	CAN_MOTOR_STEERING3,
};

class CanMotor : public CanopenNode
{
	public:
		CanMotor();
		uint16_t status_word;
		uint32_t position;
		uint16_t current;
		int speed;
		systime date;

		virtual void rx_pdo(struct can_msg *msg, int type);
		void set_speed(int32_t speed);
		int wait_update(portTickType timeout);

	protected:
		xSemaphoreHandle sem;
};

extern CanMotor can_motor[6];

#endif
