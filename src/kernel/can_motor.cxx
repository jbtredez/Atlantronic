#include "can_motor.h"
#include "canopen.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/robot_parameters.h"

#include <math.h>

#define ARRAY_SIZE(a)        (sizeof(a)/sizeof(a[0]))

#define CAN_MOTOR_CMD_DI       0x08    //!< disable
#define CAN_MOTOR_CMD_EN       0x0f    //!< enable
#define CAN_MOTOR_CMD_GOHOSEQ  0x2f    //!< lancement homing
#define CAN_MOTOR_CMD_M        0x3c    //!< debut du mouvement
#define CAN_MOTOR_CMD_SETTTL   0x52    //!< passage de l'entree AN en 5V
#define CAN_MOTOR_CMD_HOSP     0x78    //!< vitesse du homing
#define CAN_MOTOR_CMD_HP       0x79    //!< hard polarity   // TODO : dans la conf, SDO sur 0x2310:5 ?
#define CAN_MOTOR_CMD_LCC      0x80    //!< limitation courant continu
#define CAN_MOTOR_CMD_LPC      0x81    //!< limitation courant max
#define CAN_MOTOR_CMD_SHA      0x8a    //!< mise a 0 de la position encodeur lors de la detection du switch (homing)
#define CAN_MOTOR_CMD_SHL      0x90    //!< arret du moteur lors de la detection du switch (homing)
#define CAN_MOTOR_CMD_V        0x93    //!< commande de vitesse
#define CAN_MOTOR_CMD_SHN      0x9a    //!< notification (via status word) lors de la detection du switch (homing) // TODO dans la conf sdo sur 0x2310:4 ?
#define CAN_MOTOR_CMD_LA       0xb4    //!< commande de position

const struct canopen_configuration can_motor_driving_configuration[] =
{
	{0x1802, 2, 1, 1},               // pdo 3 sur SYNC
	{0x2303, 1, 1, 0xc8},            // pdo 3 - position en parametre 1
	{0x2303, 2, 1, 4},               // pdo 3 - courant en parametre 2
	{0x6060, 0, 1, 0xffffffff},      // mode faulhaber
	{0x6083, 0, 4, 1500},            // acceleration
	{0x6084, 0, 4, 1500},            // deceleration
};

const struct canopen_configuration can_motor_steering_configuration[] =
{
	{0x1802, 2, 1, 1},               // pdo 3 sur SYNC
	{0x2303, 1, 1, 0xc8},            // pdo 3 - position en parametre 1
	{0x2303, 2, 1, 4},               // pdo 3 - courant en parametre 2
	{0x6060, 0, 1, 0xffffffff},      // mode faulhaber
	{0x6083, 0, 4, 500},             // acceleration
	{0x6084, 0, 4, 500},             // deceleration
};

CanMotor can_motor[CAN_MOTOR_MAX];

int can_motor_module_init()
{
	can_motor[0].nodeid = CAN_MOTOR_DRIVING1_NODEID;
	can_motor[0].static_conf = can_motor_driving_configuration;
	can_motor[0].conf_size = ARRAY_SIZE(can_motor_driving_configuration);
	can_motor[0].inputGain = 60 * MOTOR_DRIVING1_RED / (float)(2 * M_PI * DRIVING1_WHEEL_RADIUS);
	can_motor[0].outputGain = 2 * M_PI * DRIVING1_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING1_RED);
	can_motor[0].name = "driving 1 (gauche)";
	can_motor[0].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_0;

	can_motor[1].nodeid = CAN_MOTOR_STEERING1_NODEID;
	can_motor[1].static_conf = can_motor_steering_configuration;
	can_motor[1].conf_size = ARRAY_SIZE(can_motor_steering_configuration);
	can_motor[1].inputGain = 60 * MOTOR_STEERING1_RED / (float)(2 * M_PI);
	can_motor[1].outputGain = 2 * M_PI / (float)(MOTOR_STEERING1_RED * MOTOR_ENCODER_RESOLUTION);
	can_motor[1].positionOffset = MOTOR_STEERING1_OFFSET;
	can_motor[1].name = "steering 1 (gauche)";
	can_motor[1].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_1;

	can_motor[2].nodeid = CAN_MOTOR_DRIVING2_NODEID;
	can_motor[2].static_conf = can_motor_driving_configuration;
	can_motor[2].conf_size = ARRAY_SIZE(can_motor_driving_configuration);
	can_motor[2].inputGain = 60 * MOTOR_DRIVING2_RED / (float)(2 * M_PI * DRIVING2_WHEEL_RADIUS);
	can_motor[2].outputGain = 2 * M_PI * DRIVING2_WHEEL_RADIUS / (float)(MOTOR_DRIVING2_RED * MOTOR_ENCODER_RESOLUTION);
	can_motor[2].name = "driving 2 (droite)";
	can_motor[2].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_2;

	can_motor[3].nodeid = CAN_MOTOR_STEERING2_NODEID;
	can_motor[3].static_conf = can_motor_steering_configuration;
	can_motor[3].conf_size = ARRAY_SIZE(can_motor_steering_configuration);
	can_motor[3].inputGain = 60 * MOTOR_STEERING2_RED / (float)(2 * M_PI);
	can_motor[3].outputGain = 2 * M_PI / (float)(MOTOR_STEERING2_RED * MOTOR_ENCODER_RESOLUTION);
	can_motor[3].positionOffset = MOTOR_STEERING2_OFFSET;
	can_motor[3].name = "steering 2 (droite)";
	can_motor[3].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_3;

	can_motor[4].nodeid = CAN_MOTOR_DRIVING3_NODEID;
	can_motor[4].static_conf = can_motor_driving_configuration;
	can_motor[4].conf_size = ARRAY_SIZE(can_motor_driving_configuration);
	can_motor[4].inputGain = 60 * MOTOR_DRIVING3_RED / (float)(2 * M_PI * DRIVING3_WHEEL_RADIUS);
	can_motor[4].outputGain = 2 * M_PI * DRIVING3_WHEEL_RADIUS / (float)(MOTOR_DRIVING3_RED * MOTOR_ENCODER_RESOLUTION);
	can_motor[4].name = "driving 3 (arriere)";
	can_motor[4].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_4;

	can_motor[5].nodeid = CAN_MOTOR_STEERING3_NODEID;
	can_motor[5].static_conf = can_motor_steering_configuration;
	can_motor[5].conf_size = ARRAY_SIZE(can_motor_steering_configuration);
	can_motor[5].inputGain = 60 * MOTOR_STEERING3_RED / (float)(2 * M_PI);
	can_motor[5].outputGain = 2 * M_PI / (float)(MOTOR_STEERING3_RED * MOTOR_ENCODER_RESOLUTION);
	can_motor[5].positionOffset = MOTOR_STEERING3_OFFSET;
	can_motor[5].name = "steering 3 (arriere)";
	can_motor[5].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_5;

	for(int i = 0; i < 6; i++)
	{
		canopen_register_node(&can_motor[i]);
	}

	return 0;
}

module_init(can_motor_module_init, INIT_CAN_MOTOR);
/*
static void can_motor_tx_pdo1(int node, uint16_t control_word)
{
	struct can_msg msg;

	msg.id = 0x200 + node;
	msg.size = 2;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = control_word & 0xff;
	msg.data[1] = (control_word >> 8) & 0xff;

	can_write(&msg, 0);
}*/

static void can_motor_tx_pdo2(int node, uint8_t cmd, uint32_t param)
{
	struct can_msg msg;

	msg.id = 0x300 + node;
	msg.size = 5;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.data[0] = cmd;
	msg.data[1] = param & 0xff;
	msg.data[2] = (param >> 8) & 0xff;
	msg.data[3] = (param >> 16) & 0xff;
	msg.data[4] = (param >> 24) & 0xff;

	can_write(&msg, 0);
}

CanMotor::CanMotor()
{
	inputGain = 1;
	outputGain = 1;
	kinematics.pos = 0;
	kinematics.v = 0;
	kinematics.a = 0;
	homingStatus = CAN_MOTOR_HOMING_NONE;
	positionOffset = 0;
	status_word = 0;
	last_status_word = 0;
	connected = false;
}

void CanMotor::update(portTickType absTimeout)
{
	CanopenNode::update(absTimeout);

	int res = wait_update_until(absTimeout);
	if( res )
	{
		// defaut, moteur ne repond pas
		fault(fault_disconnected_id, FAULT_ACTIVE);
		connected = false;
		systime t = systick_get_time();
		if( (t - last_communication_time).ms > 1000 && (t - last_reset_node_time).ms > 1000) // TODO define pour le 1000 ms
		{
			resetNode();
		}
	}
	else
	{
		fault(fault_disconnected_id, FAULT_CLEAR);
		connected = true;
	}

	if( state == NMT_OPERATIONAL )
	{
		update_state();
	}
	else
	{
		status_word = 0;
		last_status_word = 0;
		homingStatus = CAN_MOTOR_HOMING_NONE;
	}
}

void CanMotor::update_state()
{
	if( state == NMT_OPERATIONAL )
	{
		// gestion machine a etat du moteur
		if( (status_word & 0x6f) == 0x27 )
		{
			// etat op enable
			// rien a faire, on souhaite y rester
			if( status_word != last_status_word )
			{
				log_format(LOG_INFO, "op enable %x", nodeid);
			}
		}
		else if( (status_word & 0x6f) == 0x07 )
		{
			// etat quick stop active
			// TODO : a voir, pour repasser en op enable:
			//can_motor_rx_pdo1(nodeid, 0x0f);
		}
		else if( (status_word & 0x4f) == 0x08 )
		{
			// etat fault
			// TODO notifier un probleme. Pour aller en switch on disable :
			//can_motor_rx_pdo1(nodeid, 0x80);
		}
		else if( (status_word & 0x6f) == 0x23 )
		{
			// etat switch on
			// on veut aller en op enable
			if( status_word != last_status_word )
			{
				log_format(LOG_INFO, "switch on %x", nodeid);
			}
			//can_motor_tx_pdo1(nodeid, 0x0f);
		}
		else if( (status_word & 0x6f) == 0x21 )
		{
			// etat ready to switch on
			// on veut aller en switch on
			if( status_word != last_status_word )
			{
				log_format(LOG_INFO, "ready to switch on %x", nodeid);
			}
			//can_motor_tx_pdo1(nodeid, 7);
		}
		else if( (status_word & 0x4f) == 0x40 )
		{
			// etat switch on disable
			// on veut aller en ready to switch on
			//can_motor_tx_pdo1(nodeid, 6);

			if( status_word != last_status_word )
			{
				log_format(LOG_INFO, "switch on disable %x", nodeid);
			}
			// command faulhaber qui va direct en OP ENABLE
			//can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_EN, 0);
		}
//		else if( (motor->status_word & 0x4f) == 0x00 )
//		{
		// etat not ready to switch on
		// rien a faire, le moteur va passer en switch on disable automatiquement
//		}
//		else if( (motor->status_word & 0x4f) == 0x0f )
//		{
		// etat fault reaction active
		// rien a faire, on va passer en fault automatiquement
//		}
	}

	if( status_word & 0x4000 )
	{
		if( homingStatus != CAN_MOTOR_HOMING_DONE)
		{
			log_format(LOG_INFO, "homing end %s", name);
			homingStatus = CAN_MOTOR_HOMING_DONE;
		}
	}

	last_status_word = status_word;
}

void CanMotor::rx_pdo(struct can_msg *msg, int type)
{
	if( type == CANOPEN_RX_PDO3 )
	{
		uint32_t pos;
		memcpy(&pos, msg->data, 4);
		float dt = msg->data[6] / 1000.0f;
		float v = ((int)(pos - raw_position)) * outputGain / dt;
		raw_position = pos;
		current = msg->data[4] + (msg->data[5] << 8);

		kinematics.a = (v - kinematics.v) / dt;
		kinematics.v = v;
		kinematics.pos = positionOffset + ((int32_t)raw_position) * outputGain;
		xSemaphoreGive(sem);
	}
	else if( type == CANOPEN_RX_PDO1 )
	{
		state = NMT_OPERATIONAL;
		status_word = (msg->data[1] << 8) + msg->data[0];
	}
}

void CanMotor::enable(bool enable)
{
	if(enable)
	{
		if( ! is_op_enable() )
		{
			// command faulhaber qui va direct en OP ENABLE
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_EN, 0);
		}
	}
	else if( is_op_enable() )
	{
		can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_DI, 0);
	}
}

void CanMotor::set_speed(float v)
{
	int32_t speed = v * inputGain;
	can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_V, speed);
}

void CanMotor::set_position(float pos)
{
	int32_t pos_raw = (pos - positionOffset) / outputGain;
	can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_LA, pos_raw);
	can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_M, 0);
}

//! courant max en A
void CanMotor::set_max_current(float val)
{
	// conversion en mA pour le moteur
	can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_LPC, 1000 * val);
}

void CanMotor::update_homing(float v)
{
	int32_t speed = v * inputGain;

	switch(homingStatus)
	{
		case CAN_MOTOR_HOMING_NONE:
			log_format(LOG_INFO, "start homing %s", name);
		case CAN_MOTOR_HOMING_SETTTL:
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_SETTTL, 0);
			homingStatus = CAN_MOTOR_HOMING_RUN_HP;
			break;
		case CAN_MOTOR_HOMING_RUN_HP:
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_HP, 0); // polarite : front descendant
			homingStatus = CAN_MOTOR_HOMING_RUN_SHL;
			break;
		case CAN_MOTOR_HOMING_RUN_SHL:
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_SHL, 0x01); // arret moteur sur front descendant pin AN
			homingStatus = CAN_MOTOR_HOMING_RUN_SHA;
			break;
		case CAN_MOTOR_HOMING_RUN_SHA:
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_SHA, 0x01); // notifier dans status word sur front descendant pin AN
			homingStatus = CAN_MOTOR_HOMING_RUN_SHN;
			break;
		case CAN_MOTOR_HOMING_RUN_SHN:
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_SHN, 0x01); // mise a 0 de l'encodeur sur front descendant pin AN
			homingStatus = CAN_MOTOR_HOMING_RUN_HOSP;
			break;
		case CAN_MOTOR_HOMING_RUN_HOSP:
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_HOSP, speed); // vitesse homing
			homingStatus = CAN_MOTOR_HOMING_RUN_GOHOSEQ;
			break;
		case CAN_MOTOR_HOMING_RUN_GOHOSEQ:
			log_format(LOG_INFO, "homing %s - GOHOSEQ", name);
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_GOHOSEQ, 0); // lancement homing
			homingStatus = CAN_MOTOR_HOMING_RUNING_GOHOSEQ;
			break;
		case CAN_MOTOR_HOMING_RUNING_GOHOSEQ:
		default:
		case CAN_MOTOR_HOMING_DONE:
			break;
	}
}
