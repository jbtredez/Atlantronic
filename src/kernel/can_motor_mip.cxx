#include "can_motor_mip.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/robot_parameters.h"
#include "kernel/control.h"
#include "kernel/driver/power.h"
#include "kernel/driver/adc.h"

#include <math.h>

#define CAN_MIP_MOTOR_MAX_SPEED            2800
#define CAN_MIP_MOTOR_MAX_ACCELERATION     6000
#define CAN_MIP_MOTOR_MAX_DECELERATION     3000

CanMipMotor can_motor[CAN_MOTOR_MAX];

int can_motor_module_init()
{
	can_motor[CAN_MOTOR_RIGHT].nodeId = CAN_MOTOR_RIGHT_NODEID;
	can_motor[CAN_MOTOR_RIGHT].inputGain = 60 * MOTOR_DRIVING2_RED / (float)(2 * M_PI * DRIVING2_WHEEL_RADIUS);
	can_motor[CAN_MOTOR_RIGHT].outputGain = 2 * M_PI * DRIVING2_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING2_RED);
	can_motor[CAN_MOTOR_RIGHT].name = "moteur droit";
	can_motor[CAN_MOTOR_RIGHT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_0;

	can_motor[CAN_MOTOR_LEFT].nodeId = CAN_MOTOR_LEFT_NODEID;
	can_motor[CAN_MOTOR_LEFT].inputGain = 60 * MOTOR_DRIVING1_RED / (float)(2 * M_PI * DRIVING1_WHEEL_RADIUS);
	can_motor[CAN_MOTOR_LEFT].outputGain = 2 * M_PI * DRIVING1_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING1_RED);
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
	lastSpeedCmd = 0;

	kinematics.pos = 0;
	kinematics.v = 0;
	kinematics.a = 0;

	kinematicsParam.vMax = CAN_MIP_MOTOR_MAX_SPEED;
	kinematicsParam.aMax = CAN_MIP_MOTOR_MAX_ACCELERATION;
	kinematicsParam.dMax = CAN_MIP_MOTOR_MAX_DECELERATION;

	kinematicsCmd.pos = 0;
	kinematicsCmd.v = 0;
	kinematicsCmd.a = 0;
}

uint32_t CanMipMotor::configure(MotorWriteConfIndex idx, uint32_t val)
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
		systime t = systick_get_time();
		if( lastSpeedCmd == 0 && (t - last_communication_time).ms < 10 )
		{
			// patch : le moteur rate un cycle apres une commande a 0
			return;
		}

		fault(fault_disconnected_id, FAULT_ACTIVE);
		kinematics.a = 0;
		kinematics.v = 0;
		if( (t - last_communication_time).ms > 200 )
		{
			if( (power_get() & (~POWER_OFF_MIP_MOTOR)) || adc_filtered_data.vBat < 10 )
			{
				lastPowerOffTime = systick_get_time();
				power_clear(POWER_OFF_MIP_MOTOR);
			}
			else if( power_get() == 0 && adc_filtered_data.vBat > 10 && (t - lastPowerOffTime).ms > 500 )
			{
				// le moteur s'est arrete a cause d'un pb de suivit..., on coupe l'alim du contoleur pour le relancer
				lastPowerOffTime = systick_get_time();
				power_set(POWER_OFF_MIP_MOTOR);
			}
			else if( (t - lastPowerOffTime).ms > 100 )
			{
				// on a coupe pendant 100 ms, on remet la puissance
				power_clear(POWER_OFF_MIP_MOTOR);
			}
		}
		state = CAN_MOTOR_MIP_DISCONNECTED;
	}
	else
	{
		fault(fault_disconnected_id, FAULT_CLEAR);
		float dt = CONTROL_DT;
		float v = ((int)(raw_position - old_raw_position)) * outputGain / dt;
		old_raw_position = raw_position;

		kinematics.a = (v - kinematics.v) / dt;
		kinematics.v = v;
		kinematics.pos = ((int32_t)raw_position) * outputGain;
	}

	switch( state )
	{
		case CAN_MOTOR_MIP_INIT:
			fault(fault_disconnected_id, FAULT_CLEAR);
			log_format(LOG_INFO, "configure mip %d max voltage", nodeId);
			configure(MOTOR_CONF_IDX_MANUAL_VOLTAGE, 28300);
			{
				struct can_msg msg;
				msg.id = 0x01 + (nodeId << 3);
				msg.format = CAN_STANDARD_FORMAT;
				msg.type = CAN_DATA_FRAME;
				// init position + enable
				lastRaz = systick_get_time();
				log_format(LOG_INFO, "mip init %d", nodeId);
				msg.size = 2;
				msg.data[0] = CAN_MIP_CMD_RAZ;
				msg.data[1] = 0x01;
				can_write(&msg, 0);

				// disable
				msg.size = 1;
				msg.data[0] = CAN_MIP_CMD_DISABLE;
				can_write(&msg, 0);
			}
			state = CAN_MOTOR_MIP_INIT_WAIT;
			break;
		case CAN_MOTOR_MIP_INIT_WAIT:
			{
				// disable
				struct can_msg msg;
				msg.id = 0x01 + (nodeId << 3);
				msg.format = CAN_STANDARD_FORMAT;
				msg.type = CAN_DATA_FRAME;
				msg.size = 1;
				msg.data[0] = CAN_MIP_CMD_DISABLE;
				can_write(&msg, 0);
				systime t = systick_get_time();
				if( (t - t_motor_online).ms > 200 )
				{
					state = CAN_MOTOR_MIP_READY;
				}
			}
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

	if( state == CAN_MOTOR_MIP_DISCONNECTED)
	{
		return;
	}

	// on evite de le faire trop vite enable (moteur pas vraiement pret)
	systime t = systick_get_time();
	if( (t - t_motor_online).ms < 200)
	{
		enable = false;
	}

	msg.id = 0x01 + (nodeId << 3);
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;

	if( enable )
	{
		if( ! (mipState & CAN_MIP_MOTOR_STATE_POWERED) )
		{
			if( mipState & CAN_MIP_MOTOR_STATE_POSITION_UNKNOWN )
			{
				if( (t - lastRaz).ms > 1000 )
				{
					// init position + enable
					lastRaz = t;
					log_format(LOG_INFO, "mip init %d", nodeId);
					msg.size = 2;
					msg.data[0] = CAN_MIP_CMD_RAZ;
					msg.data[1] = 0x01;
					can_write(&msg, 0);
				}
			}
			else if( (t - lastMotionEnable).ms > 1000 )
			{
				lastMotionEnable = t;
				log_format(LOG_INFO, "mip enable %d", nodeId);
				msg.size = 1;
				msg.data[0] = CAN_MIP_CMD_ENABLE;
				can_write(&msg, 0);
			}
		}
	}
	else if( mipState & CAN_MIP_MOTOR_STATE_POWERED )
	{
		log_format(LOG_INFO, "mip disable %d", nodeId);
		msg.size = 1;
		msg.data[0] = CAN_MIP_CMD_DISABLE;
		can_write(&msg, 0);
	}
}

void CanMipMotor::set_speed(float v)
{
	struct can_msg msg;
	v *= inputGain;

	kinematicsCmd.setSpeed(v, kinematicsParam, CONTROL_DT);
	v = kinematicsCmd.v;
	uint16_t speed = fabsf(v);

	if( state != CAN_MOTOR_MIP_READY )
	{
		// moteur pas pret, on ne fait rien
		return;
	}

	if( speed == 0 && lastSpeedCmd == 0 && ! (mipState & CAN_MIP_MOTOR_STATE_IN_MOTION) )
	{
		// 0 lance une fonction d'arret speciale cote moteur
		// si on n est pas IN_MOTION, pas besoin de faire un arret (cela pose un pb cote moteur sinon)
		return;
	}

	if( speed )
	{
		nullSpeedCount = 0;
	}
	else
	{
		nullSpeedCount++;
		// on retarde de quelques cycles v = 0 pour ne pas lancer la fonction d'arret du moteur pour rien
		if( nullSpeedCount < 4)
		{
			speed = 1;
		}
	}

	lastSpeedCmd = speed;

	// vitesse min pour le moteur...
	if( speed < 15 && speed > 0)
	{
		speed = 15;
	}

	msg.id = 0x01 + (nodeId << 3);
	msg.size = 6;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = CAN_MIP_CMD_SPEED;
	msg.data[1] = speed & 0xff;
	msg.data[2] = (speed >> 8) & 0xff;
	msg.data[3] = v >= 0 ? 1 : 0;
	msg.data[4] = 0;
	msg.data[5] = 100;

	can_write(&msg, 0);
}

void CanMipMotor::rxMsg(struct can_msg *msg)
{
	CanMipNode::rxMsg(msg);

	if( state == CAN_MOTOR_MIP_DISCONNECTED)
	{
		state = CAN_MOTOR_MIP_INIT;
		t_motor_online = systick_get_time();
	}

	// decodage msg
	if( mipState != msg->data[0])
	{
		log_format(LOG_INFO, "motor %x : new state %x => %x", nodeId, mipState, msg->data[0]);
		mipState = msg->data[0];
	}
	if( state != CAN_MOTOR_MIP_INIT )
	{
		memcpy(&raw_position, &msg->data[2], 4);
	}

	xSemaphoreGive(sem);
}
