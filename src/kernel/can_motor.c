#include "can_motor.h"
#include "canopen.h"
#include "kernel/module.h"
#include "kernel/log.h"

#define ARRAY_SIZE(a)        (sizeof(a)/sizeof(a[0]))

enum can_motor_mode
{
	CAN_MOTOR_POSITION = 0x01,
	CAN_MOTOR_SPEED    = 0x03,
	CAN_MOTOR_HOMING   = 0x06
};

const struct canopen_configuration can_motor_driving_configuration[] =
{
	{0x1802, 2, 1, 1},               // pdo 3 sur SYNC
	{0x2303, 1, 1, 0xc8},            // pdo 3 - position en parametre 1
	{0x2303, 2, 1, 4},               // pdo 3 - courant en parametre 2
	{0x6060, 0, 1, CAN_MOTOR_SPEED}, // mode vitesse
	{0x6083, 0, 4, 1500},            // acceleration
	{0x6084, 0, 4, 1500},            // deceleration
};

const struct canopen_configuration can_motor_steering_configuration[] =
{
	{0x1802, 2, 1, 1},               // pdo 3 sur SYNC
	{0x2303, 1, 1, 0xc8},            // pdo 3 - position en parametre 1
	{0x2303, 2, 1, 4},               // pdo 3 - courant en parametre 2
	{0x6060, 0, 1, CAN_MOTOR_SPEED}, // mode vitesse
	{0x6083, 0, 4, 500},             // acceleration
	{0x6084, 0, 4, 500},             // deceleration
};

static void can_motor_callback(struct can_msg *msg, int id, int type);

struct can_motor_data
{
	uint16_t status_word;
	uint16_t configureStep;
};

static struct can_motor_data can_motor_data[4];

int can_motor_module_init()
{
	canopen_register_node(0x20, can_motor_driving_configuration, ARRAY_SIZE(can_motor_driving_configuration), &can_motor_callback);
	canopen_register_node(0x21, can_motor_steering_configuration, ARRAY_SIZE(can_motor_steering_configuration), &can_motor_callback);

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

void can_motor_callback(struct can_msg *msg, int id, int type)
{
	if( type == CANOPEN_RX_PDO1 )
	{
		can_motor_data[0].status_word = (msg->data[1] << 8) + msg->data[0];

		// le status word a change
		// gestion machine a etat du moteur
		if( (can_motor_data[0].status_word & 0x6f) == 0x27 )
		{
			// etat op enable
			// rien a faire, on souhaite y rester
		}
		else if( (can_motor_data[0].status_word & 0x6f) == 0x07 )
		{
			// etat quick stop active
			// TODO : a voir, pour repasser en op enable:
			//can_motor_rx_pdo1(id, 0x0f);
		}
		else if( (can_motor_data[0].status_word & 0x4f) == 0x08 )
		{
			// etat fault
			// TODO notifier un probleme. Pour aller en switch on disable :
			//can_motor_rx_pdo1(id, 0x80);
		}
		else if( (can_motor_data[0].status_word & 0x6f) == 0x23 )
		{
			// etat switch on
			// on veut aller en op enable
			can_motor_tx_pdo1(id, 0x0f);
		}
		else if( (can_motor_data[0].status_word & 0x6f) == 0x21 )
		{
			// etat ready to switch on
			// on veut aller en switch on
			can_motor_tx_pdo1(id, 7);
		}
		else if( (can_motor_data[0].status_word & 0x4f) == 0x40 )
		{
			// etat switch on disable
			// on veut aller en ready to switch on
			can_motor_tx_pdo1(id, 6);
		}
//		else if( (can_motor_data[0].status_word & 0x4f) == 0x00 )
//		{
			// etat not ready to switch on
			// rien a faire, le moteur va passer en switch on disable automatiquement
//		}
//		else if( (can_motor_data[0].status_word & 0x4f) == 0x0f )
//		{
			// etat fault reaction active
			// rien a faire, on va passer en fault automatiquement
//		}

	}
}
