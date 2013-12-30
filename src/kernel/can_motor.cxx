#include "can_motor.h"
#include "canopen.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include <math.h>

#define ARRAY_SIZE(a)        (sizeof(a)/sizeof(a[0]))

#define CAN_MOTOR_CMD_DI   0x08      //!< disable
#define CAN_MOTOR_CMD_EN   0x0f      //!< enable
#define CAN_MOTOR_CMD_M    0x3c      //!< debut du mouvement
#define CAN_MOTOR_CMD_LCC  0x80      //!< limitation courant continu
#define CAN_MOTOR_CMD_LPC  0x81      //!< limitation courant max
#define CAN_MOTOR_CMD_V    0x93      //!< commande de vitesse
#define CAN_MOTOR_CMD_LA   0xb4      //!< commande de position

#define DRIVING1_WHEEL_RADIUS       33
#define DRIVING2_WHEEL_RADIUS       33
#define DRIVING3_WHEEL_RADIUS       33

#define MOTOR_RED                   14         //!< reducteur moteur (mettre 676.0 / 49.0 ?)
#define MOTOR_DRIVING1_RED          MOTOR_RED  //!< reduction moteur
#define MOTOR_DRIVING2_RED          MOTOR_RED
#define MOTOR_DRIVING3_RED          MOTOR_RED
#define MOTOR_STEERING1_RED         (4.375f*MOTOR_RED)
#define MOTOR_STEERING2_RED         (4.375f*MOTOR_RED)
#define MOTOR_STEERING3_RED         (4.375f*MOTOR_RED)

#define MOTOR_ENCODER_RESOLUTION         3000

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

	can_motor[1].nodeid = CAN_MOTOR_STEERING1_NODEID;
	can_motor[1].static_conf = can_motor_steering_configuration;
	can_motor[1].conf_size = ARRAY_SIZE(can_motor_steering_configuration);
	can_motor[1].inputGain = 60 * MOTOR_STEERING1_RED / (float)(2 * M_PI);
	can_motor[1].outputGain = 2 * M_PI / (float)(MOTOR_STEERING1_RED * MOTOR_ENCODER_RESOLUTION);

	can_motor[2].nodeid = CAN_MOTOR_DRIVING2_NODEID;
	can_motor[2].static_conf = can_motor_driving_configuration;
	can_motor[2].conf_size = ARRAY_SIZE(can_motor_driving_configuration);
	can_motor[2].inputGain = 60 * MOTOR_DRIVING2_RED / (float)(2 * M_PI * DRIVING2_WHEEL_RADIUS);
	can_motor[2].outputGain = 2 * M_PI * DRIVING2_WHEEL_RADIUS / (float)(MOTOR_DRIVING2_RED * MOTOR_ENCODER_RESOLUTION);

	can_motor[3].nodeid = CAN_MOTOR_STEERING2_NODEID;
	can_motor[3].static_conf = can_motor_steering_configuration;
	can_motor[3].conf_size = ARRAY_SIZE(can_motor_steering_configuration);
	can_motor[3].inputGain = 60 * MOTOR_STEERING2_RED / (float)(2 * M_PI);
	can_motor[3].outputGain = 2 * M_PI / (float)(MOTOR_STEERING2_RED * MOTOR_ENCODER_RESOLUTION);

	can_motor[4].nodeid = CAN_MOTOR_DRIVING3_NODEID;
	can_motor[4].static_conf = can_motor_driving_configuration;
	can_motor[4].conf_size = ARRAY_SIZE(can_motor_driving_configuration);
	can_motor[4].inputGain = 60 * MOTOR_DRIVING3_RED / (float)(2 * M_PI * DRIVING3_WHEEL_RADIUS);
	can_motor[4].outputGain = 2 * M_PI * DRIVING3_WHEEL_RADIUS / (float)(MOTOR_DRIVING3_RED * MOTOR_ENCODER_RESOLUTION);

	can_motor[5].nodeid = CAN_MOTOR_STEERING3_NODEID;
	can_motor[5].static_conf = can_motor_steering_configuration;
	can_motor[5].conf_size = ARRAY_SIZE(can_motor_steering_configuration);
	can_motor[5].inputGain = 60 * MOTOR_STEERING3_RED / (float)(2 * M_PI);
	can_motor[5].outputGain = 2 * M_PI / (float)(MOTOR_STEERING3_RED * MOTOR_ENCODER_RESOLUTION);

	for(int i = 0; i < 6; i++)
	{
		canopen_register_node(&can_motor[i]);
	}

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

CanMotor::CanMotor()
{
	vSemaphoreCreateBinary(sem);
	xSemaphoreTake(sem, 0);
	inputGain = 1;
	outputGain = 1;
	kinematics.pos = 0;
	kinematics.v = 0;
	kinematics.a = 0;
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
		kinematics.pos = ((int32_t)raw_position) * outputGain;
		xSemaphoreGive(sem);
	}
	else if( type == CANOPEN_RX_PDO1 )
	{
		status_word = (msg->data[1] << 8) + msg->data[0];

		// le status word a change
		// gestion machine a etat du moteur
		if( (status_word & 0x6f) == 0x27 )
		{
			// etat op enable
			// rien a faire, on souhaite y rester
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
			can_motor_tx_pdo1(nodeid, 0x0f);
		}
		else if( (status_word & 0x6f) == 0x21 )
		{
			// etat ready to switch on
			// on veut aller en switch on
			can_motor_tx_pdo1(nodeid, 7);
		}
		else if( (status_word & 0x4f) == 0x40 )
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

void CanMotor::set_speed(float v)
{
	int32_t speed = v * inputGain;
	can_motor_tx_pdo2(nodeid, CAN_MOTOR_CMD_V, speed);
}

//! attente de la mise a jour du moteur
//! @return 0 si c'est bon -1 si timeout
int CanMotor::wait_update(portTickType timeout)
{
	int res = 0;

	if( xSemaphoreTake(sem, timeout) == pdFALSE )
	{
		res = -1;
	}

	return res;
}

//! attente de la mise a jour du moteur
//! @return 0 si c'est bon -1 si timeout
int CanMotor::wait_update_until(portTickType t)
{
	int res = 0;

	systime currentTime = systick_get_time();
	int timeout = t - currentTime.ms;
	if( timeout < 0)
	{
		timeout = 0;
	}

	if( xSemaphoreTake(sem, timeout) == pdFALSE )
	{
		res = -1;
	}

	return res;
}
