#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include <stdint.h>
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/control/kinematics.h"

enum
{
	CAN_MOTOR_DRIVING1 = 0,
	CAN_MOTOR_STEERING1,
	CAN_MOTOR_DRIVING2,
	CAN_MOTOR_STEERING2,
	CAN_MOTOR_DRIVING3,
	CAN_MOTOR_STEERING3,
	CAN_MOTOR_MAX,
};

enum homing_status
{
	CAN_MOTOR_HOMING_NONE,
	CAN_MOTOR_HOMING_RUNNING,
	CAN_MOTOR_HOMING_DONE,
};

class CanMotor : public CanopenNode
{
	public:
		CanMotor();

		const char* name;

		// donnees brutes
		uint16_t status_word;
		uint32_t raw_position; //!< position brute (en increments encodeurs)
		uint16_t current;
		Kinematics kinematics;
		enum homing_status homingStatus;

		//systime date;
		float inputGain;    //!< gain pour convertir la vitesse en unites moteurs (rpm)
		float outputGain;   //!< gain pour convertir la position en unites robot
		float positionOffset; //!< offset sur la position

		void start_homing(float v);
		void set_speed(float v);
		int wait_update(portTickType timeout);
		int wait_update_until(portTickType t);

		inline bool is_op_enable()
		{
			return (status_word & 0x6f) == 0x27;
		}
	protected:
		virtual void rx_pdo(struct can_msg *msg, int type);
		xSemaphoreHandle sem;
};

extern CanMotor can_motor[CAN_MOTOR_MAX];

#endif
