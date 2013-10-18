#include "can_motor.h"
#include "canopen.h"
#include "kernel/module.h"
#include "kernel/log.h"

#define ARRAY_SIZE(a)        (sizeof(a)/sizeof(a[0]))

#define CAN_MOTOR_CMD_DI   0x08      //!< enable
#define CAN_MOTOR_CMD_EN   0x0f      //!< disable
#define CAN_MOTOR_CMD_M    0x3c      //!< debut du mouvement
#define CAN_MOTOR_CMD_LCC  0x80      //!< limitation courant continu
#define CAN_MOTOR_CMD_LPC  0x81      //!< limitation courant max
#define CAN_MOTOR_CMD_V    0x93      //!< commande de vitesse
#define CAN_MOTOR_CMD_LA   0xb4      //!< commande de position

const struct canopen_configuration can_motor_driving_configuration[] =
{
	{0x1802, 2, 1, 1},               // pdo 3 sur SYNC
	{0x2303, 1, 1, 0xc8},            // pdo 3 - position en parametre 1
	{0x2303, 2, 1, 4},               // pdo 3 - courant en parametre 2
	{0x6060, 0, 1, -1},              // mode faulhaber
	{0x6083, 0, 4, 1500},            // acceleration
	{0x6084, 0, 4, 1500},            // deceleration
};

const struct canopen_configuration can_motor_steering_configuration[] =
{
	{0x1802, 2, 1, 1},               // pdo 3 sur SYNC
	{0x2303, 1, 1, 0xc8},            // pdo 3 - position en parametre 1
	{0x2303, 2, 1, 4},               // pdo 3 - courant en parametre 2
	{0x6060, 0, 1, -1},              // mode faulhaber
	{0x6083, 0, 4, 500},             // acceleration
	{0x6084, 0, 4, 500},             // deceleration
};

static void can_motor_callback(void* data, struct can_msg *msg, int id, int type);

struct can_motor_data
{
	uint16_t status_word;
	uint32_t position;
	uint16_t current;
	int speed;
};

static struct can_motor_data can_motor_data[6];

int can_motor_module_init()
{
	canopen_register_node(CAN_MOTOR_DRIVING1_NODEID, can_motor_driving_configuration, ARRAY_SIZE(can_motor_driving_configuration), &can_motor_data[0], &can_motor_callback);
	canopen_register_node(CAN_MOTOR_STEERING1_NODEID, can_motor_steering_configuration, ARRAY_SIZE(can_motor_steering_configuration), &can_motor_data[1], &can_motor_callback);
	canopen_register_node(CAN_MOTOR_DRIVING2_NODEID, can_motor_driving_configuration, ARRAY_SIZE(can_motor_driving_configuration), &can_motor_data[2], &can_motor_callback);
	canopen_register_node(CAN_MOTOR_STEERING2_NODEID, can_motor_steering_configuration, ARRAY_SIZE(can_motor_steering_configuration), &can_motor_data[3], &can_motor_callback);
	canopen_register_node(CAN_MOTOR_DRIVING3_NODEID, can_motor_driving_configuration, ARRAY_SIZE(can_motor_driving_configuration), &can_motor_data[4], &can_motor_callback);
	canopen_register_node(CAN_MOTOR_STEERING3_NODEID, can_motor_steering_configuration, ARRAY_SIZE(can_motor_steering_configuration), &can_motor_data[5], &can_motor_callback);

	return 0;
}

module_init(can_motor_module_init, INIT_CAN_MOTOR);

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
}

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

void can_motor_callback(void* data, struct can_msg *msg, int nodeid, int type)
{
	struct can_motor_data* motor = data;

	if( type == CANOPEN_RX_PDO3 )
	{
		uint32_t pos = msg->data[0] + (msg->data[1] << 8) + (msg->data[2] << 16) + (msg->data[3] << 24);
		motor->speed = ((int)(pos - motor->position)) * 1000 / msg->data[6];
		motor->position = pos;
		motor->current = msg->data[4] + (msg->data[5] << 8);
	}
	else if( type == CANOPEN_RX_PDO1 )
	{
		motor->status_word = (msg->data[1] << 8) + msg->data[0];

		// le status word a change
		// gestion machine a etat du moteur
		if( (motor->status_word & 0x6f) == 0x27 )
		{
			// etat op enable
			// rien a faire, on souhaite y rester
		}
		else if( (motor->status_word & 0x6f) == 0x07 )
		{
			// etat quick stop active
			// TODO : a voir, pour repasser en op enable:
			//can_motor_rx_pdo1(nodeid, 0x0f);
		}
		else if( (motor->status_word & 0x4f) == 0x08 )
		{
			// etat fault
			// TODO notifier un probleme. Pour aller en switch on disable :
			//can_motor_rx_pdo1(nodeid, 0x80);
		}
		else if( (motor->status_word & 0x6f) == 0x23 )
		{
			// etat switch on
			// on veut aller en op enable
			can_motor_tx_pdo1(nodeid, 0x0f);
		}
		else if( (motor->status_word & 0x6f) == 0x21 )
		{
			// etat ready to switch on
			// on veut aller en switch on
			can_motor_tx_pdo1(nodeid, 7);
		}
		else if( (motor->status_word & 0x4f) == 0x40 )
		{
			// etat switch on disable
			// on veut aller en ready to switch on
			//can_motor_tx_pdo1(nodeid, 6);

			// command faulhaber qui va direct en OP ENABLE
			can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_EN, 0);
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
}

void can_motor_set_speed(uint8_t nodeid, int32_t speed)
{
	can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_V, speed);
}
