//! @file DynamixelManager.cxx
//! @brief Gestion des dynamixel (ax12 et rx24)
//! @author Atlantronic

#include "DynamixelManager.h"
#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/match.h"
#include "kernel/driver/power.h"
#include <stdlib.h>
#include <math.h>

#define DYNAMIXEL_STACK_SIZE            350
#define DYNAMIXEL_READ_TIMEOUT           15 // en ms

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

	for(int i = 0; i < m_devicesCount; i++)
	{
		if( m_devices[i]->id() == dynamixel->id() )
		{
			log_format(LOG_ERROR, "dynamixel %d already registered", dynamixel->id());
			res = -1;
			goto unlock;
		}
	}

	if( m_devicesCount < DYNAMIXEL_MAX_ON_BUS )
	{
		m_devices[m_devicesCount] = dynamixel;
		m_devicesCount++;
	}
	else
	{
		res = -1;
	}

unlock:
	xSemaphoreGive(m_mutex);

	return res;
}

Dynamixel* DynamixelManager::findDynamixel(int id)
{
	Dynamixel* res = NULL;

	xSemaphoreTake(m_mutex, portMAX_DELAY);

	for(int i = 0; i < m_devicesCount; i++)
	{
		if( m_devices[i]->id() == id )
		{
			res = m_devices[i];
			break;
		}
	}

	xSemaphoreGive(m_mutex);

	return res;
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

void DynamixelManager::task_wrapper(void* arg)
{
	DynamixelManager* manager = (DynamixelManager*) arg;
	manager->task();
}

void DynamixelManager::task()
{
	int id = 0;

	while(1)
	{
		Dynamixel* currentDynamixel = m_devices[id];

		currentDynamixel->update();

		id = (id + 1) % m_devicesCount;
		if(id == 0)
		{
			vTaskDelay(5);
		}
	}
}

void DynamixelManager::updateUsbData(DynamixelUsbData* dynamixelData)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	for(int i = 0; i < m_devicesCount; i++)
	{
		m_devices[i]->updateUsbData(&dynamixelData->dynamixel[i]);
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

__OPTIMIZE_SIZE__ void DynamixelManager::cmd(void* arg, void* data)
{
	struct dynamixel_cmd_param* param = (struct dynamixel_cmd_param*)data;
	DynamixelError err;
	float theta;

	DynamixelManager* manager = (DynamixelManager*) arg;

	if( param->cmd_id == DYNAMIXEL_CMD_SCAN )
	{
		manager->cmd_scan();
		return;
	}

	if( param->cmd_id == DYNAMIXEL_CMD_SET_MANAGER_BAUDRATE )
	{
		usart_set_frequency(manager->m_usart, (uint32_t)(param->param));
		return;
	}

	Dynamixel* dynamixel = manager->findDynamixel(param->id);
	Dynamixel dynTmp;
	if( ! dynamixel )
	{
		// si le dynamixel n'est pas enregistre, on en cree un temporairement
		dynTmp.init(manager, param->id, false);
		dynamixel = &dynTmp;
	}

	switch(param->cmd_id)
	{
		case DYNAMIXEL_CMD_SET_ID:
			dynamixel->setId(((int)param->param) & 0xff);
			break;
		case DYNAMIXEL_CMD_SET_GOAL_POSITION:
			dynamixel->setGoalPosition(param->param);
			dynamixel->update();
			break;
		case DYNAMIXEL_CMD_SET_SPEED:
			dynamixel->setMovingSpeed(param->param);
			break;
		case DYNAMIXEL_CMD_SET_BAUDRATE:
			// on les met a 200kb, commande usb pour la configuration
			dynamixel->setBaudRate(9);
			break;
		case DYNAMIXEL_CMD_SET_MAX_TORQUE:
			dynamixel->setTorqueEnable(1);
			dynamixel->setTorqueLimit(param->param);
			dynamixel->update();
			break;
		case DYNAMIXEL_CMD_SET_TARGET_REACHED_THRESHOLD:
			dynamixel->setTargetReachedThreshold(param->param);
			break;
		case DYNAMIXEL_CMD_GET_POSITION:
			theta = dynamixel->getPosition(&err);
			if(!err.transmit_error)
			{
				theta *= 180 / M_PI;
				log_format(LOG_INFO, "ax%d %u - pos %d", manager->m_type, param->id, (int)theta);
			}
			break;
		case DYNAMIXEL_CMD_ENABLE_ENDLESS_TURN_MODE:
			dynamixel->setCwAngleLimit(0);
			dynamixel->setCcwAngleLimit(0);
			break;
		case DYNAMIXEL_CMD_DISABLE_ENDLESS_TURN_MODE:
			dynamixel->setCwAngleLimit(0);
			dynamixel->setCcwAngleLimit(0x3ff);
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

