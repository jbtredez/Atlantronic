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

enum MotorState
{
	CAN_MOTOR_MIP_DISCONNECTED,
	CAN_MOTOR_MIP_INIT,
	CAN_MOTOR_MIP_READY,
};

enum MotorWriteConfIndex
{
	MOTOR_CONF_IDX_ZERO_CODER = 0,
	MOTOR_CONF_IDX_MAX_VOLTAGE,
	MOTOR_CONF_IDX_MIN_VOLTAGE,
	MOTOR_CONF_IDX_TRAJECTORY_VOLTAGE,
	MOTOR_CONF_IDX_STOP_VOLTAGE,
	MOTOR_CONF_IDX_INIT_VOLTAGE,
	MOTOR_CONF_IDX_TARGET_ACCURACY,
	MOTOR_CONF_IDX_RESERVED_7,
	MOTOR_CONF_IDX_SCREW_STEP,
	MOTOR_CONF_IDX_MANUAL_VOLTAGE,
	MOTOR_CONF_IDX_POSITION_CONTROL,
	MOTOR_CONF_IDX_RESERVED_B,
	MOTOR_CONF_IDX_INIT_WAY,
	MOTOR_CONF_IDX_MAX_POSITION,          //!< position max (32 bits)
	MOTOR_CONF_IDX_MIN_POSITION,          //!< position min (32 bits)
	MOTOR_CONF_IDX_INIT_POSITION,
	MOTOR_CONF_IDX_KP_VOLTAGE,
	MOTOR_CONF_IDX_TRAJECTORY_TIMEOUT,
	MOTOR_CONF_IDX_SEND_MSG_ON_TRAJ_END,
	MOTOR_CONF_IDX_UNCOUPLING_THRESHOLD,
	MOTOR_CONF_IDX_STOP_UNCOUPLING_THRESHOLD,
};

#define CAN_MIP_CMD_SPEED                  0x40
#define CAN_MIP_CMD_RAZ                    0x60
#define CAN_MIP_CMD_DISABLE                0x70
#define CAN_MIP_CMD_ENABLE                 0x80

#define CAN_MIP_MOTOR_MAX_SPEED            2800

#define CAN_MIP_MOTOR_STATE_TRAJ_PTS_FULL         0x01
#define CAN_MIP_MOTOR_STATE_IN_MOTION             0x02
#define CAN_MIP_MOTOR_STATE_POWERED               0x04
#define CAN_MIP_MOTOR_STATE_POSITION_UNKNOWN      0x08
#define CAN_MIP_MOTOR_STATE_ERR_RANGE             0x10
#define CAN_MIP_MOTOR_STATE_ERR_TRAJ              0x20
#define CAN_MIP_MOTOR_STATE_ERR_TEMPERATURE       0x40
#define CAN_MIP_MOTOR_STATE_ERR_CMD_VS_MES        0x80
#define CAN_MIP_MOTOR_STATE_ERROR                 0xf0

#define CAN_MIP_MOTOR_HISTORY_SIZE                  28

class CanMipMotor : public CanMipNode
{
	public:
		CanMipMotor();

		const char* name;
		// donnees brutes
		uint8_t mipState;
		uint32_t old_raw_position;
		uint32_t raw_position; //!< position brute (en increments encodeurs)
		uint16_t current; // TODO pas utilise
		Kinematics kinematics;

		float inputGain;    //!< gain pour convertir la vitesse en unites moteurs (rpm)
		float outputGain;   //!< gain pour convertir la position en unites robot
		enum fault fault_disconnected_id;
		MotorState state;

		void update(portTickType absTimeout);
		void rxMsg(struct can_msg *msg);
		void set_speed(float v);
		void enable(bool enable);

		inline bool is_op_enable()
		{
			return (mipState & CAN_MIP_MOTOR_STATE_POWERED) && !(mipState & CAN_MIP_MOTOR_STATE_POSITION_UNKNOWN);
		}

		inline void set_position(float pos)
		{
			(void) pos;
		}

		inline void set_max_current(float val)
		{
			(void) val;
		}

	protected:
		uint32_t configure(MotorWriteConfIndex idx, uint32_t val);
};

extern CanMipMotor can_motor[CAN_MOTOR_MAX];

#endif
