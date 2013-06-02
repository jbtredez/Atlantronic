#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include <stdint.h>
#include "kernel/can/can_id.h"

void can_motor_set_speed(uint8_t nodeid, int32_t speed);

#endif
