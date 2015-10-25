//! @file dynamixel.cxx
//! @brief Gestion des dynamixel (ax12 et rx24)
//! @author Atlantronic

#define WEAK_DYNAMIXEL
#include "dynamixel.h"
#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/match.h"
#include "kernel/driver/power.h"
#include <stdlib.h>
#include <math.h>

#define DYNAMIXEL_INSTRUCTION_PING             0x01
#define DYNAMIXEL_INSTRUCTION_READ_DATA        0x02
#define DYNAMIXEL_INSTRUCTION_WRITE_DATA       0x03
#define DYNAMIXEL_INSTRUCTION_REG_WRITE        0x04
#define DYNAMIXEL_INSTRUCTION_ACTION           0x05
#define DYNAMIXEL_INSTRUCTION_RESET            0x06
#define DYNAMIXEL_INSTRUCTION_SYNC_WRITE       0x83

#define DYNAMIXEL_STACK_SIZE            350
#define DYNAMIXEL_READ_TIMEOUT           15 // en ms
#define DYNAMIXEL_MOVE_TIMEOUT         3000 // en ms

static uint8_t dynamixel_checksum(uint8_t* buffer, uint8_t size);

int DynamixelManager::init(const char* name, enum usart_id usart_id, uint32_t frequency, int Max_devices_id, uint8_t Type)
{
	m_usart = usart_id;
	m_type = Type;
	int res = usart_open(m_usart, frequency);
	if( res )
	{
		return -1;
	}

	if( Max_devices_id > 2)
	{
		portBASE_TYPE err = xTaskCreate(task_wrapper, name, DYNAMIXEL_STACK_SIZE, this, PRIORITY_TASK_DYNAMIXEL, NULL);

		if(err != pdPASS)
		{
			return ERR_INIT_DYNAMIXEL;
		}
	}

	m_mutex = xSemaphoreCreateMutex();

	if( m_mutex == 0)
	{
		return ERR_INIT_DYNAMIXEL;
	}

	m_usartMutex = xSemaphoreCreateMutex();

	if( m_usartMutex == 0)
	{
		return ERR_INIT_DYNAMIXEL;
	}

	usart_set_write_dma_buffer(m_usart, m_writeDmaBuffer);
	usart_set_read_dma_buffer(m_usart, m_readDmaBuffer);

	m_devicesCount = 0;

	if(m_type == DYNAMIXEL_TYPE_AX12)
	{
		usb_add_cmd(USB_CMD_AX12, &cmd, this);
	}
	else if( m_type == DYNAMIXEL_TYPE_RX24 )
	{
		usb_add_cmd(USB_CMD_RX24, &cmd, this);
	}
	return 0;
}

int DynamixelManager::registerDynamixel(Dynamixel* dynamixel)
{
	int res = 0;

	xSemaphoreTake(m_mutex, portMAX_DELAY);

	if( m_devicesCount < DYNAMIXEL_MAX_ON_BUS )
	{
		m_devices[m_devicesCount] = dynamixel;
		m_devicesCount++;
	}
	else
	{
		res = -1;
	}

	xSemaphoreGive(m_mutex);

	return res;
}

int Dynamixel::init(DynamixelManager* manager, int Id)
{
	m_manager = manager;
	id = Id;
	last_error.transmit_error = 0xff;
	max_goal = 0x3ff;
	goal_pos = 0x1ff;
	target_reached_threshold = 2;
	flags = DYNAMIXEL_FLAG_TARGET_REACHED | DYNAMIXEL_FLAG_CONTROL_OFF | DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
	m_manager->registerDynamixel(this);
	return 0;
}

void DynamixelManager::task_wrapper(void* arg)
{
	DynamixelManager* manager = (DynamixelManager*) arg;
	manager->task();
}

void DynamixelManager::task()
{
	struct DynamixelRequest req;
	int id = 0;

	while(1)
	{
		Dynamixel* currentDynamixel = m_devices[id];
		// lecture de la position du dynamixel
		req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
		req.arg[0] = DYNAMIXEL_PRESENT_POSITION_L;
		req.arg[1] = 0x02;
		req.argc = 2;
		req.id = currentDynamixel->id;

		// pas de log d erreur de com si pas de puissance
		if( ! power_get() )
		{
			send(&req);
		}
		else
		{
			req.status.error.transmit_error = ERR_DYNAMIXEL_POWER_OFF;
		}

		if( !req.status.error.transmit_error )
		{
			int pos = req.status.arg[0] + (req.status.arg[1] << 8);
			xSemaphoreTake(m_mutex, portMAX_DELAY);
			currentDynamixel->pos = pos;
			int goal_pos = currentDynamixel->goal_pos;

			uint16_t pos_err = abs(pos - goal_pos);
			systime t = systick_get_time();
			if( currentDynamixel->flags & DYNAMIXEL_FLAG_CONTROL_OFF )
			{
				currentDynamixel->timeStartMoving_ms = t.ms;
			}
			if( pos_err <= currentDynamixel->target_reached_threshold)
			{
				currentDynamixel->flags |= DYNAMIXEL_FLAG_TARGET_REACHED;
				currentDynamixel->timeStartMoving_ms = t.ms;
				if( currentDynamixel->flags & DYNAMIXEL_FLAG_STUCK )
				{
					log_format(LOG_ERROR, "%s id %d unstucked", pcTaskGetTaskName(NULL), currentDynamixel->id);
					currentDynamixel->flags &= ~DYNAMIXEL_FLAG_STUCK;
				}
			}
			else if( pos_err > currentDynamixel->target_reached_threshold + 1)
			{
				if( currentDynamixel->flags & DYNAMIXEL_FLAG_TARGET_REACHED )
				{
					currentDynamixel->timeStartMoving_ms = t.ms;
					currentDynamixel->flags &= ~DYNAMIXEL_FLAG_TARGET_REACHED;
				}

				if( t.ms - currentDynamixel->timeStartMoving_ms > DYNAMIXEL_MOVE_TIMEOUT )
				{
					if( ! (currentDynamixel->flags & DYNAMIXEL_FLAG_STUCK) )
					{
						log_format(LOG_ERROR, "%s id %d stucked", pcTaskGetTaskName(NULL), currentDynamixel->id);
						currentDynamixel->flags |= DYNAMIXEL_FLAG_STUCK;
					}
				}
				else if( currentDynamixel->flags & DYNAMIXEL_FLAG_STUCK )
				{
					log_format(LOG_ERROR, "%s id %d unstucked", pcTaskGetTaskName(NULL), currentDynamixel->id);
					currentDynamixel->flags &= ~DYNAMIXEL_FLAG_STUCK;
				}
			}

			// renvoi du couple uniquement si ! control_off et torque_to_update
			if( (currentDynamixel->flags & (DYNAMIXEL_FLAG_TORQUE_TO_UPDATE | DYNAMIXEL_FLAG_CONTROL_OFF)) == DYNAMIXEL_FLAG_TORQUE_TO_UPDATE || m_disabled)
			{
				uint16_t max_torque = 0;
				if( ! m_disabled )
				{
					max_torque = currentDynamixel->max_torque;
				}
				DynamixelError err = write16(currentDynamixel->id, DYNAMIXEL_TORQUE_LIMIT_L, max_torque);
				if( ! err.transmit_error && ! err.internal_error )
				{
					err = write8(currentDynamixel->id, DYNAMIXEL_TORQUE_ENABLE, 0x01);
					if( ! err.transmit_error )
					{
						currentDynamixel->flags &= ~DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
					}
				}
			}
			xSemaphoreGive(m_mutex);

			if( pos_err > 0 && ! (currentDynamixel->flags & DYNAMIXEL_FLAG_CONTROL_OFF) && ! m_disabled)
			{
				// on va envoyer la position désirée
				req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
				req.arg[0] = DYNAMIXEL_GOAL_POSITION_L;
				req.arg[1] = (uint8_t) (goal_pos & 0xFF);
				req.arg[2] = (uint8_t) ((goal_pos >> 8) & 0xFF);
				req.argc = 3;
				send(&req);
			}
		}

		xSemaphoreTake(m_mutex, portMAX_DELAY);
		if( currentDynamixel->last_error.transmit_error )
		{
			currentDynamixel->flags |= DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
		}
		if(currentDynamixel->last_error.transmit_error != req.status.error.transmit_error || currentDynamixel->last_error.internal_error != req.status.error.internal_error)
		{
			print_error(req.id, req.status.error);
			currentDynamixel->last_error = req.status.error;
		}

		xSemaphoreGive(m_mutex);

		id = (id + 1) % m_devicesCount;
		if(id == 0)
		{
			vTaskDelay(5);
		}
	}
}

void DynamixelManager::updateUsbData(struct DynamixelUsbData* dynamixelData)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	for(int i = 0; i < m_devicesCount; i++)
	{
		dynamixelData->dynamixel[i].id = m_devices[i]->id;
		dynamixelData->dynamixel[i].pos = m_devices[i]->pos;
		dynamixelData->dynamixel[i].flags = m_devices[i]->flags;
		dynamixelData->dynamixel[i].error = m_devices[i]->last_error;
	}
	xSemaphoreGive(m_mutex);
}

void DynamixelManager::send(struct DynamixelRequest *req)
{
	uint32_t res = 0;
	uint8_t write_size = 6 + req->argc;
	uint8_t read_size = 0;
	uint8_t i;
	uint8_t* buffer;

	xSemaphoreTake(m_usartMutex, portMAX_DELAY);

	m_writeDmaBuffer[0] = 0xFF;
	m_writeDmaBuffer[1] = 0xFF;
	m_writeDmaBuffer[2] = req->id;
	m_writeDmaBuffer[3] = 0x02 + req->argc;
	m_writeDmaBuffer[4] = req->instruction;
	for( i = 0; i < req->argc ; i++)
	{
		m_writeDmaBuffer[5+i] = req->arg[i];
	}
	m_writeDmaBuffer[write_size-1] = dynamixel_checksum(m_writeDmaBuffer, write_size);

	read_size = write_size;

	if(req->id != 0xFE)
	{
		if(req->instruction != DYNAMIXEL_INSTRUCTION_READ_DATA)
		{
			read_size += 6;
		}
		else
		{
			read_size += 6 + req->arg[1];
		}
	}

	usart_set_read_dma_size(m_usart, read_size);
	usart_send_dma_buffer(m_usart, write_size);

	res = usart_wait_read(m_usart, DYNAMIXEL_READ_TIMEOUT);
	if( res )
	{
		goto end_unlock;
	}

	buffer = m_readDmaBuffer;
	// verification des données envoyées
	for(i = 0; i< write_size ; i++)
	{
		if( m_writeDmaBuffer[i] != m_readDmaBuffer[i] )
		{
			// erreur, on n'a pas lus ce qui a été envoyé
			res = ERR_DYNAMIXEL_SEND_CHECK;
			goto end_unlock;
		}
	}

	buffer += write_size;

	// pas de broadcast => réponse attendue
	if(req->id != 0xFE)
	{
		int size = read_size - write_size;
		if(buffer[0] != 0xFF || buffer[1] != 0xFF || buffer[2] != req->id || buffer[3] != size - 4)
		{
			// erreur protocole
			res = ERR_DYNAMIXEL_PROTO;
			goto end_unlock;
		}

		if( buffer[size - 1] != dynamixel_checksum(buffer, size))
		{
			// erreur checksum
			res = ERR_DYNAMIXEL_CHECKSUM;
			goto end_unlock;
		}

		req->status.error.internal_error = buffer[4];

		if(size == 7)
		{
			req->status.arg[0] = buffer[5];
		}
		else if(size == 8)
		{
			req->status.arg[0] = buffer[5];
			req->status.arg[1] = buffer[6];
		}
	}

end_unlock:
	if(res && ! (res & ERR_USART_TIMEOUT) )
	{
		// on vide tout ce qui traine dans le buffer de reception
		usart_set_read_dma_size(m_usart, sizeof(m_readDmaBuffer));
		usart_wait_read(m_usart, DYNAMIXEL_READ_TIMEOUT);
	}
	xSemaphoreGive(m_usartMutex);
	req->status.error.transmit_error = res;
}

static uint8_t dynamixel_checksum(uint8_t* buffer, uint8_t size)
{
	uint8_t i = 2;
	uint8_t checksum = 0;

	for(; i< size - 1 ; i++)
	{
		checksum += buffer[i];
	}
	checksum = ~checksum;

	return checksum;
}

void DynamixelManager::print_error(int id, DynamixelError err)
{
	if(err.transmit_error)
	{
		if(err.transmit_error == ERR_DYNAMIXEL_SEND_CHECK)
		{
			log_format(LOG_ERROR, "%s %3d : échec de la verification des octets envoyés", pcTaskGetTaskName(NULL), id);
		}
		else if(err.transmit_error == ERR_DYNAMIXEL_PROTO)
		{
			log_format(LOG_ERROR, "%s %3d : erreur protocole", pcTaskGetTaskName(NULL), id);
		}
		else if( err.transmit_error == ERR_DYNAMIXEL_CHECKSUM)
		{
			log_format(LOG_ERROR, "%s %3d : somme de verification incompatible", pcTaskGetTaskName(NULL), id);
		}
		else if( err.transmit_error == ERR_DYNAMIXEL_POWER_OFF)
		{
			log_format(LOG_ERROR, "%s %3d : power off", pcTaskGetTaskName(NULL), id);
		}
		else
		{
			if(err.transmit_error & ERR_USART_TIMEOUT)
			{
				log_format(LOG_ERROR, "%s %3d : timeout", pcTaskGetTaskName(NULL), id);
			}
			else if(err.transmit_error & ERR_USART_READ_SR_FE)
			{
				log_format(LOG_ERROR, "%s %3d : desynchro, bruit ou octet \"break\" sur l'usart", pcTaskGetTaskName(NULL), id);
			}
			if(err.transmit_error & ERR_USART_READ_SR_NE)
			{
				log_format(LOG_ERROR, "%s %3d : bruit sur l'usart", pcTaskGetTaskName(NULL), id);
			}
			if(err.transmit_error & ERR_USART_READ_SR_ORE)
			{
				log_format(LOG_ERROR, "%s %3d : overrun sur l'usart", pcTaskGetTaskName(NULL), id);
			}
		}
	}
	else if(err.internal_error)
	{
		if( err.internal_error & DYNAMIXEL_INPUT_VOLTAGE_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - problème de tension", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_ANGLE_LIMIT_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - angle invalide", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_OVERHEATING_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - surchauffe", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_RANGE_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - valeur non admissible", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_CHECKSUM_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - somme de verification incompatible", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_OVERLOAD_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - surcharge de l'actioneur", pcTaskGetTaskName(NULL), id);
		}
		if( err.internal_error & DYNAMIXEL_INSTRUCTION_ERROR_MASK)
		{
			log_format(LOG_ERROR, "%s %3d : erreur interne - instruction invalide", pcTaskGetTaskName(NULL), id);
		}
	}
	else
	{
		log_format(LOG_INFO, "%s %3d ok", pcTaskGetTaskName(NULL), id);
	}
}

uint8_t DynamixelManager::read8(uint8_t id, uint8_t offset, DynamixelError* error)
{
	struct DynamixelRequest req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x01;
	req.argc = 2;

	send(&req);
	*error = req.status.error;

	return req.status.arg[0];
}

uint16_t DynamixelManager::read16(uint8_t id, uint8_t offset, DynamixelError* error)
{
	struct DynamixelRequest req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x02;
	req.argc = 2;

	send(&req);
	*error = req.status.error;

	return req.status.arg[0] + (req.status.arg[1] << 8);
}

DynamixelError DynamixelManager::write8(uint8_t id, uint8_t offset, uint8_t data)
{
	struct DynamixelRequest req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = data;
	req.argc = 2;

	send(&req);
	return req.status.error;
}

DynamixelError DynamixelManager::write16(uint8_t id, uint8_t offset, uint16_t data)
{
	struct DynamixelRequest req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = (uint8_t) (data & 0xFF);
	req.arg[2] = (uint8_t) ((data >> 8) & 0xFF);
	req.argc = 3;

	send(&req);
	return req.status.error;
}

DynamixelError DynamixelManager::ping(uint8_t id)
{
	struct DynamixelRequest req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_PING;
	req.argc = 0;

	send(&req);
	return req.status.error;
}

DynamixelError DynamixelManager::action(uint8_t id)
{
	struct DynamixelRequest req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_ACTION;
	req.argc = 0;

	send(&req);
	return req.status.error;
}

DynamixelError DynamixelManager::reset(uint8_t id)
{
	struct DynamixelRequest req;
	req.id = id;
	req.instruction = DYNAMIXEL_INSTRUCTION_RESET;
	req.argc = 0;

	send(&req);
	return req.status.error;
}

DynamixelError DynamixelManager::setLed(uint8_t id, uint8_t on)
{
	return write8(id, DYNAMIXEL_LED, on);
}

DynamixelError DynamixelManager::set_moving_speed(uint8_t id, float speed)
{
	uint16_t speed16 = (uint16_t)fabsf(DYNAMIXEL_RD_TO_POS * speed);
	if( speed16 > 0x3ff )
	{
		speed16 = 0x3ff;
	}
	if( speed < 0 )
	{
		speed16 |= 1<<10;
	}

	DynamixelError error = write16(id, DYNAMIXEL_MOVING_SPEED_L, speed16);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission dynamixel id %d", id);
	}
	return error;
}

DynamixelError Dynamixel::set_goal_position(float theta)
{
	DynamixelError err;

	// on met theta dans [-M_PI ; M_PI[
	theta = fmodf(theta, 2*M_PI);
	if( theta > M_PI)
	{
		theta -= 2*M_PI;
	}
	else if(theta < -M_PI)
	{
		theta += 2*M_PI;
	}

	// passage en unité dynamixel entre 0 et 0x3ff (de -150 degre a 150 degre)
	// zero au milieu qui vaut 0x1ff
	int32_t alpha = 0x1ff + theta * DYNAMIXEL_RD_TO_POS;

	// saturation
	if(alpha < min_goal)
	{
		alpha = min_goal;
	}
	else if( alpha > max_goal)
	{
		alpha = max_goal;
	}

	// utilisation de la tache pour la mise à jour
//	xSemaphoreTake(mutex, portMAX_DELAY); // TODO gestion mutex
	flags &= ~(DYNAMIXEL_FLAG_CONTROL_OFF | DYNAMIXEL_FLAG_STUCK);
	goal_pos = alpha;
	if( abs(goal_pos - pos) < target_reached_threshold)
	{
		flags |= DYNAMIXEL_FLAG_TARGET_REACHED;
	}
	else
	{
		flags &= ~DYNAMIXEL_FLAG_TARGET_REACHED;
	}
	err = last_error;
//	xSemaphoreGive(mutex);

	return err;
}

DynamixelError Dynamixel::set_torque_limit(float torque_limit)
{
	DynamixelError err;

	if( match_end || torque_limit < 0)
	{
		torque_limit = 0;
	}

	if( torque_limit > 1)
	{
		torque_limit = 1;
	}

	// utilisation de la tache pour la mise à jour
//	xSemaphoreTake(mutex, portMAX_DELAY); // TODO mutex
	max_torque = torque_limit * DYNAMIXEL_MAX_TORQUE_LIMIT;
	flags |= DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
	err = last_error;
//	xSemaphoreGive(mutex);

	return err;
}

DynamixelError DynamixelManager::set_torque_limit_eeprom(uint8_t id, float torque_limit)
{
	if( torque_limit < 0)
	{
		torque_limit = 0;
	}

	if( torque_limit > 1)
	{
		torque_limit = 1;
	}

	return write16(id, DYNAMIXEL_MAX_TORQUE_L, torque_limit * DYNAMIXEL_MAX_TORQUE_LIMIT);
}

DynamixelError DynamixelManager::set_torque_enable(uint8_t id, uint8_t enable)
{
	return write8(id, DYNAMIXEL_TORQUE_ENABLE, enable & 0x01);
}

DynamixelError DynamixelManager::set_cw_angle_limit(uint8_t id, uint16_t val)
{
	DynamixelError error = write16(id, DYNAMIXEL_CW_ANGLE_LIMIT_L, val);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission dynamixel id %d", id);
	}
	else
	{
		log_format(LOG_INFO, "cw angle limit %d - status = %#.2x", val, error.internal_error);
	}

	return error;
}

DynamixelError DynamixelManager::set_ccw_angle_limit(uint8_t id, uint16_t val)
{
	DynamixelError error = write16(id, DYNAMIXEL_CCW_ANGLE_LIMIT_L, val);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission dynamixel id %d", id);
	}
	else
	{
		log_format(LOG_INFO, "ccw angle limit %d - status = %#.2x", val, error.internal_error);
	}

	return error;
}

void Dynamixel::set_goal_limit(float min, float max)
{
	min = 0x1ff + min * DYNAMIXEL_RD_TO_POS;
	max = 0x1ff + max * DYNAMIXEL_RD_TO_POS;

	if(min < 0)
	{
		min = 0;
	}
	else if(min > 0x3ff)
	{
		min = 0x3ff;
	}

	if( max < 0)
	{
		max = 0;
	}
	else if( max > 0x3ff)
	{
		max = 0x3ff;
	}

	min_goal = min;
	max_goal = max;
}

void Dynamixel::set_target_reached_threshold(float threshold)
{
	target_reached_threshold = threshold * DYNAMIXEL_RD_TO_POS;
}

bool Dynamixel::isFlagActive(uint32_t mask)
{
	bool res = false;
	//xSemaphoreTake(mutex, portMAX_DELAY); // TODO
	res = flags & mask;
	//xSemaphoreGive(mutex);

	return res;
}

float Dynamixel::get_position(DynamixelError* error)
{
	int32_t alpha = 0x1ff;
//	xSemaphoreTake(mutex, portMAX_DELAY); // TODO
	alpha = pos;
	*error = last_error;
//	xSemaphoreGive(mutex);

	// passage en rd
	// zero au milieu qui vaut "0x1ff"
	return (alpha - 0x1ff) * DYNAMIXEL_POS_TO_RD;
}

__OPTIMIZE_SIZE__ void DynamixelManager::cmd(void* arg, void* data)
{
	struct dynamixel_cmd_param* param = (struct dynamixel_cmd_param*)data;
//	struct dynamixel_error err;
//	float theta;

	DynamixelManager* manager = (DynamixelManager*) arg;

	switch(param->cmd_id)
	{
		case DYNAMIXEL_CMD_SCAN:
			manager->cmd_scan();
			break;
		case DYNAMIXEL_CMD_SET_ID:
			manager->cmd_set_id(param->id, ((int)param->param) & 0xff);
			break;
		case DYNAMIXEL_CMD_SET_GOAL_POSITION:
			//manager->set_goal_position(param->id, param->param); // TODO
			break;
		case DYNAMIXEL_CMD_SET_SPEED:
			manager->set_moving_speed(param->id, param->param);
			break;
		case DYNAMIXEL_CMD_SET_BAUDRATE:
			// on les met a 200kb, commande usb pour la configuration
			manager->write8(param->id, DYNAMIXEL_BAUD_RATE, 9);
			break;
		case DYNAMIXEL_CMD_SET_MANAGER_BAUDRATE:
			usart_set_frequency(manager->m_usart, (uint32_t)(param->param));
			break;
		case DYNAMIXEL_CMD_SET_MAX_TORQUE:
			manager->set_torque_enable(param->id, 1);
			//manager->set_torque_limit(param->id, param->param); // TODO
			break;
		case DYNAMIXEL_CMD_SET_TARGET_REACHED_THRESHOLD:
			//manager->set_target_reached_threshold(param->id, param->param); // TODO
			break;
		case DYNAMIXEL_CMD_GET_POSITION:
			/*theta = manager->get_position(param->id, &err);
			if(!err.transmit_error)
			{
				theta *= 180 / M_PI;
				log_format(LOG_INFO, "ax%d %u - pos %d", param->type, param->id, (int)theta);
			}*/ // TODO
			break;
		case DYNAMIXEL_CMD_ENABLE_ENDLESS_TURN_MODE:
			manager->set_cw_angle_limit(param->id, 0);
			manager->set_ccw_angle_limit(param->id, 0);
			break;
		case DYNAMIXEL_CMD_DISABLE_ENDLESS_TURN_MODE:
			manager->set_cw_angle_limit(param->id, 0);
			manager->set_ccw_angle_limit(param->id, 0x3ff);
			break;
		default:
			log_format(LOG_ERROR, "unknown dynamixel command : %d", param->cmd_id);
			break;
	}
}

void DynamixelManager::cmd_scan()
{
	int i;
	DynamixelError error;

	log(LOG_INFO, "dynamixel - scan");

	for(i = 1; i<254; i++)
	{
		error = ping(i);
		if(! error.transmit_error)
		{
			log_format(LOG_INFO, "dynamixel type %d id %3d détecté - status %#.2x", m_type, i, error.internal_error);
		}
	}

	log(LOG_INFO, "dynamixel - end scan");
}

void DynamixelManager::cmd_set_id(uint8_t old_id, uint8_t id)
{
	DynamixelError error;

	if(id <= 0xfd)
	{
		error = write8(old_id, DYNAMIXEL_ID, id);
		if( error.transmit_error)
		{
			log_format(LOG_ERROR, "erreur de transmission dynamixel id %d", id);
		}
		else
		{
			log_format(LOG_INFO, "id %d -> %d - status = %#.2x", old_id, id, error.internal_error);
		}
	}
	else
	{
		log(LOG_ERROR, "changement d'id par broadcast non autorisé");
	}
}
