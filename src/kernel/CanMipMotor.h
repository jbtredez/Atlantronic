#ifndef CAN_MIP_MOTOR_H
#define CAN_MIP_MOTOR_H

#include <stdint.h>
#include "kernel/can/can_id.h"
#include "kernel/CanMipNode.h"
#include "kernel/control/kinematics.h"
#include "kernel/fault.h"
#include "MotorInterface.h"
#include "kernel/driver/encoder/EncoderInterface.h"

enum MotorState
{
	CAN_MOTOR_MIP_DISCONNECTED,
	CAN_MOTOR_MIP_INIT,
	CAN_MOTOR_MIP_INIT_WAIT,
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
#define CAN_MIP_CMD_CONFIGURE              0xb0


#define CAN_MIP_MOTOR_STATE_TRAJ_PTS_FULL         0x01
#define CAN_MIP_MOTOR_STATE_IN_MOTION             0x02
#define CAN_MIP_MOTOR_STATE_POWERED               0x04
#define CAN_MIP_MOTOR_STATE_POSITION_UNKNOWN      0x08
#define CAN_MIP_MOTOR_STATE_ERR_RANGE             0x10
#define CAN_MIP_MOTOR_STATE_ERR_TRAJ              0x20
#define CAN_MIP_MOTOR_STATE_ERR_TEMPERATURE       0x40
#define CAN_MIP_MOTOR_STATE_ERR_CMD_VS_MES        0x80
#define CAN_MIP_MOTOR_STATE_ERROR                 0xf0

class CanMipMotor : public CanMipNode, public MotorInterface, public EncoderInterface
{
	public:
		CanMipMotor();

		// donnees brutes
		uint8_t mipState;
		uint32_t old_raw_position;
		uint32_t raw_position; //!< position brute (en increments encodeurs)
		Kinematics kinematics;
		uint16_t lastSpeedCmd;

		float outputGain;   //!< gain pour convertir la position en unites robot
		enum fault fault_disconnected_id;
		MotorState state;
		int nullSpeedCount;
		systime lastPowerOffTime;

		// realisation de rampes pour le moteur vu que cela ne se passe pas bien si on met v = 0 directement
		KinematicsParameters kinematicsParam; //!< parametres cinematique en rpm
		Kinematics kinematicsCmd;             //!< cinematique commandee en rpm
		systime t_motor_online;
		systime lastRaz;
		systime lastMotionEnable;

		void update(float /*dt*/){};
		float getPosition(){return kinematics.pos;};
		float getSpeed(){return kinematics.v;};

		void update(portTickType absTimeout);
		void set_speed(float v);
		void enable(bool enable);

		inline bool is_in_motion()
		{
			return mipState & CAN_MIP_MOTOR_STATE_IN_MOTION;
		}

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
		void rxMsg(struct can_msg *msg);
};

#endif
