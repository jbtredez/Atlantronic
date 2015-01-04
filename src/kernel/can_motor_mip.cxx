#include "can_motor_mip.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/robot_parameters.h"

#include <math.h>

#define ARRAY_SIZE(a)        (sizeof(a)/sizeof(a[0]))

CanMipMotor can_motor[CAN_MOTOR_MAX];

int can_motor_module_init()
{
	can_motor[CAN_MOTOR_RIGHT].nodeId = CAN_MOTOR_RIGHT_NODEID;
	can_motor[CAN_MOTOR_RIGHT].inputGain = 60 * MOTOR_DRIVING1_RED / (float)(2 * M_PI * DRIVING1_WHEEL_RADIUS);
	can_motor[CAN_MOTOR_RIGHT].outputGain = 2 * M_PI * DRIVING1_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING1_RED);
	can_motor[CAN_MOTOR_RIGHT].name = "moteur droit";
	can_motor[CAN_MOTOR_RIGHT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_0;

	can_motor[CAN_MOTOR_LEFT].nodeId = CAN_MOTOR_LEFT_NODEID;
	can_motor[CAN_MOTOR_LEFT].inputGain = 60 * MOTOR_DRIVING2_RED / (float)(2 * M_PI * DRIVING2_WHEEL_RADIUS);
	can_motor[CAN_MOTOR_LEFT].outputGain = 2 * M_PI * DRIVING2_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING2_RED);
	can_motor[CAN_MOTOR_LEFT].name = "moteur gauche";
	can_motor[CAN_MOTOR_LEFT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_1;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		can_mip_register_node(&can_motor[i]);
	}

	return 0;
}

module_init(can_motor_module_init, INIT_CAN_MOTOR);

CanMipMotor::CanMipMotor()
{
	inputGain = 1;
	outputGain = 1;
	mipState = 0;
	state = CAN_MOTOR_MIP_DISCONNECTED;
	old_raw_position = 0;
	raw_position = 0;

	kinematics.pos = 0;
	kinematics.v = 0;
	kinematics.a = 0;
	posHistorySize = 0;
	posHistoryEnd = 0;
}

uint32_t CanMipMotor::configure16(MotorWriteConfIndex idx, uint16_t val)
{
	struct can_msg msg;
	msg.id = 0x01 + (nodeId << 3);
	msg.size = 4;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 0xb0;
	msg.data[1] = idx;
	msg.data[2] = val & 0xff;
	msg.data[3] = (val >> 8) & 0xff;

	return can_write(&msg, 0);
}

uint32_t CanMipMotor::configure32(MotorWriteConfIndex idx, uint32_t val)
{
	struct can_msg msg;
	msg.id = 0x01 + (nodeId << 3);
	msg.size = 6;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 0xb0;
	msg.data[1] = idx;
	msg.data[2] = val & 0xff;
	msg.data[3] = (val >> 8) & 0xff;
	msg.data[4] = (val >> 16) & 0xff;
	msg.data[5] = (val >> 24) & 0xff;

	return can_write(&msg, 0);
}

void CanMipMotor::update(portTickType absTimeout)
{
	CanMipNode::update(absTimeout);

	int res = wait_update_until(absTimeout);
	if( res )
	{
		// defaut, moteur ne repond pas
		fault(fault_disconnected_id, FAULT_ACTIVE);
		/*systime t = systick_get_time();
		if( (t - last_communication_time).ms > 1000 && (t - last_reset_node_time).ms > 1000) // TODO define pour le 1000 ms
		{
			resetNode();
		}*/
		state = CAN_MOTOR_MIP_DISCONNECTED;
	}
	else
	{
		fault(fault_disconnected_id, FAULT_CLEAR);
		float dt = 0.005f; // TODO
		float v = ((int)(old_raw_position - raw_position)) * outputGain / dt;
		old_raw_position = raw_position;
		//if( nodeId == 1 && v != 0) log_format(LOG_INFO, "v %d state %x", (int)(v*inputGain), state);

		kinematics.a = (v - kinematics.v) / dt;
		kinematics.v = v;
		kinematics.pos = ((int32_t)raw_position) * outputGain;
	}

	switch( state )
	{
		case CAN_MOTOR_MIP_INIT:
			fault(fault_disconnected_id, FAULT_CLEAR);
			//configure16(MOTOR_CONF_IDX_SEND_MSG_ON_TRAJ_END, 0);
			state = CAN_MOTOR_MIP_READY;
			break;
		case CAN_MOTOR_MIP_DISCONNECTED:
		case CAN_MOTOR_MIP_READY:
		default:
			break;
	}

	if( (mipState & CAN_MIP_MOTOR_STATE_ERROR) && (mipState & CAN_MIP_MOTOR_STATE_POWERED) )
	{
		// TODO affichage log erreur selon erreur
		log_format(LOG_ERROR, "motor %x error state = %x => disable motor", nodeId, mipState);
		enable(false);
	}
}

void CanMipMotor::enable(bool enable)
{
	struct can_msg msg;

	msg.id = 0x01 + (nodeId << 3);
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;

	if( enable && ! (mipState & CAN_MIP_MOTOR_STATE_POWERED) )
	{
		if( mipState & CAN_MIP_MOTOR_STATE_POSITION_UNKNOWN )
		{
			// init position + enable
			msg.size = 2;
			msg.data[0] = 0x60;
			msg.data[1] = 0x01;
			can_write(&msg, 0);
		}
		else
		{
			msg.size = 1;
			msg.data[0] = 0x80;
			can_write(&msg, 0);
		}
	}
	else if( ! enable && (mipState & CAN_MIP_MOTOR_STATE_POWERED) )
	{
		msg.size = 1;
		msg.data[0] = 0x70;
		can_write(&msg, 0);
	}
}

void CanMipMotor::set_speed(float v)
{
	struct can_msg msg;
	v *= inputGain;
	uint16_t speed = fabsf(v);

	msg.id = 0x01 + (nodeId << 3);
	msg.size = 4;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = 0x40;
	msg.data[1] = speed & 0xff;
	msg.data[2] = (speed >> 8) & 0xff;
	msg.data[3] = v > 0 ? 1 : 0;
	if( nodeId == 1) log_format(LOG_INFO, "set speed %x : %d", nodeId, (int)v);
	can_write(&msg, 0);
}

void CanMipMotor::set_position(float pos)
{
	struct can_msg msg;
	pos /= -outputGain;
	int32_t p = (int32_t)pos;
	uint16_t p16 = (uint16_t)p;

	// code special de fin de trajectorie a eviter
	if( p16 == 0x7FFF )
	{
		p16 = 0x7FFE;
	}

	int lastPos = posHistoryEnd - 1;
	if( lastPos < 0)
	{
		lastPos += CAN_MIP_MOTOR_HISTORY_SIZE;
	}
	/*if( p16 == posHistory[lastPos])
	{
		// TODO hack fin de trajectoire
		p16 = 0x7FFF;
	}*/
//	if( nodeId == 1) log_format(LOG_INFO, "set position %x : %x", nodeId, (int)p);

	posHistory[posHistoryEnd] = p16;
	posHistoryEnd++;
	int writeDataSize = 0;
	if( posHistoryEnd >= CAN_MIP_MOTOR_HISTORY_SIZE)
	{
		posHistoryEnd = 0;
	}

	if( posHistoryEnd % 4 == 0)
	{
		int begin = posHistoryEnd - 4;
		if( begin < 0)
		{
			begin += CAN_MIP_MOTOR_HISTORY_SIZE;
		}

		for(int i = 0; i < 4; i++)
		{
			posMotorBuffer[i] = posHistory[(begin + i)%CAN_MIP_MOTOR_HISTORY_SIZE];
		}
		writeDataSize = 1;
	}
	posHistorySize++;
	if( posHistorySize > CAN_MIP_MOTOR_HISTORY_SIZE)
	{
		posHistorySize = CAN_MIP_MOTOR_HISTORY_SIZE;
	}

//	bool trajPtsFull = mipState & CAN_MIP_MOTOR_STATE_TRAJ_PTS_FULL;
	bool inMotion = mipState & CAN_MIP_MOTOR_STATE_IN_MOTION;

	if( writeDataSize )
	{
		msg.size = 8;
		msg.format = CAN_STANDARD_FORMAT;
		msg.type = CAN_DATA_FRAME;

		for(int i = 0; i < writeDataSize; i++)
		{
			if( i == writeDataSize - 1 )
			{
				msg.id = 0x06 + (nodeId << 3);
			}
			else
			{
				msg.id = 0x05 + (nodeId << 3);
			}

			msg.data[0] = posMotorBuffer[4*i] & 0xff;
			msg.data[1] = (posMotorBuffer[4*i] >> 8) & 0xff;
			msg.data[2] = posMotorBuffer[4*i+1] & 0xff;
			msg.data[3] = (posMotorBuffer[4*i+1] >> 8) & 0xff;
			msg.data[4] = posMotorBuffer[4*i+2] & 0xff;
			msg.data[5] = (posMotorBuffer[4*i+2] >> 8) & 0xff;
			msg.data[6] = posMotorBuffer[4*i+3] & 0xff;
			msg.data[7] = (posMotorBuffer[4*i+3] >> 8) & 0xff;
			can_write(&msg, 0);
			if( nodeId == 1) log_format(LOG_INFO, "set positions %4x %4x %4x %4x", posMotorBuffer[4*i], posMotorBuffer[4*i+1], posMotorBuffer[4*i+2], posMotorBuffer[4*i+3]);
		}
	}

	if( posHistorySize>4 && !inMotion)
	{
		if( nodeId == 1) log_format(LOG_INFO, "start motion %x", nodeId);

		msg.id = 0x01 + (nodeId << 3);
		msg.size = 1;
		msg.data[0] = 0xa0;
		can_write(&msg, 0);
	}
}

void CanMipMotor::rxMsg(struct can_msg *msg)
{
	CanMipNode::rxMsg(msg);

	if( state == CAN_MOTOR_MIP_DISCONNECTED)
	{
		state = CAN_MOTOR_MIP_INIT;
	}

	// decodage msg
	if( mipState != msg->data[0])
	{
		log_format(LOG_INFO, "motor %x : new state %x => %x", nodeId, mipState, msg->data[0]);
		mipState = msg->data[0];
	}
	memcpy(&raw_position, &msg->data[2], 4);
	xSemaphoreGive(sem);
}
