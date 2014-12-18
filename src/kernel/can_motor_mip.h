#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#include <stdint.h>
#include "kernel/can/can_id.h"
#include "kernel/can_mip.h"
#include "kernel/control/kinematics.h"
#include "kernel/fault.h"

enum
{
	CAN_MOTOR_RIGHT = 0,
	CAN_MOTOR_LEFT,
	CAN_MOTOR_MAX,
};

class CanMipMotor : public CanMipNode
{
	public:
		CanMipMotor();

		const char* name;
#if 0
		// donnees brutes
		uint16_t last_status_word;
		uint16_t status_word;
		uint32_t raw_position; //!< position brute (en increments encodeurs)
		uint16_t current;
		Kinematics kinematics;
#endif
		float inputGain;    //!< gain pour convertir la vitesse en unites moteurs (rpm)
		float outputGain;   //!< gain pour convertir la position en unites robot
		bool connected;
		enum fault fault_disconnected_id;
#if 0
		void update(portTickType absTimeout);
		void update_state();
		void set_speed(float v);
		void set_position(float pos);
		void set_max_current(float val);
		void enable(bool enable);
#endif
};

extern CanMipMotor can_motor[CAN_MOTOR_MAX];

#endif
