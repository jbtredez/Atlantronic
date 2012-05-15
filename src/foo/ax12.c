//! @file ax12.c
//! @brief Gestion AX12
//! @author Atlantronic

#include "ax12.h"
#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/semphr.h"
#include <stdlib.h>

#define AX12_INSTRUCTION_PING             0x01
#define AX12_INSTRUCTION_READ_DATA        0x02
#define AX12_INSTRUCTION_WRITE_DATA       0x03
#define AX12_INSTRUCTION_REG_WRITE        0x04
#define AX12_INSTRUCTION_ACTION           0x05
#define AX12_INSTRUCTION_RESET            0x06
#define AX12_INSTRUCTION_SYNC_WRITE       0x83

#define AX12_STACK_SIZE            350
#define AX12_ARG_MAX                 3
#define AX12_READ_TIMEOUT        ms_to_tick(10)
#define AX12_INTER_FRAME_TIME    us_to_tick(15)
#define AX12_MAX_RETRY              10
#define AX12_MAX_REQUEST            10

struct ax12_status
{
	struct ax12_error error;
	uint8_t complete;
	uint8_t argc;
	uint8_t arg[AX12_ARG_MAX];
};

struct ax12_request
{
	uint8_t id;
	uint8_t instruction;
	uint8_t argc;
	uint8_t arg[AX12_ARG_MAX];
	struct ax12_status* status;
};

struct ax12_device
{
	uint8_t update; //!< lecture et envoi automatique de la position
	//!< bit 7 à 1 : ERR_AX12_SEND_CHECK, ERR_AX12_PROTO ou ERR_AX12_CHECKSUM
	//!< bit 7 à 0 : erreur usart sur les 4 bits de poids faible
	uint8_t transmit_error;
	//!< erreur interne ax12 (champ de bit)
	uint8_t internal_error;
	struct ax12_request request_fifo[AX12_MAX_REQUEST];
	uint8_t request_size;
	uint8_t request_begin;
	uint8_t request_end;
	uint16_t min_goal;
	uint16_t max_goal;
	uint16_t goal_pos;
	uint16_t pos;
};

static void ax12_task(void *arg);
static uint8_t ax12_write_dma_buffer[6 + AX12_ARG_MAX];
static uint8_t ax12_read_dma_buffer[2*(6 + AX12_ARG_MAX)];
static uint8_t ax12_checksum(uint8_t* buffer, uint8_t size);
static void ax12_send(struct ax12_request *req);
static void ax12_send_request(struct ax12_request* request);

// fonctions de commandes (utilisation par usb uniquement)
static void ax12_cmd(void* arg);
static void ax12_cmd_scan();
static void ax12_cmd_set_id(uint8_t old_id, uint8_t id);

static xSemaphoreHandle ax12_mutex;
static struct ax12_device ax12_device[AX12_MAX_ID];


static int ax12_module_init()
{
	usart_open(UART4_HALF_DUPLEX, 1000000);

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(ax12_task, "ax12", AX12_STACK_SIZE, NULL, PRIORITY_TASK_AX12, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_AX12;
	}

	ax12_mutex = xSemaphoreCreateMutex();

	if( ax12_mutex == NULL)
	{
		return ERR_INIT_AX12;
	}

	usart_set_write_dma_buffer(UART4_HALF_DUPLEX, ax12_write_dma_buffer);
	usart_set_read_dma_buffer(UART4_HALF_DUPLEX, ax12_read_dma_buffer);

	usb_add_cmd(USB_CMD_AX12, &ax12_cmd);

	int i;
	for(i = 0; i < AX12_MAX_ID; i++)
	{
		ax12_device[i].transmit_error = 0xff;
		ax12_device[i].max_goal = 0x3ff;
		ax12_device[i].goal_pos = 0x1ff;
	}

	return 0;
}

module_init(ax12_module_init, INIT_AX12);

static void ax12_task(void* arg)
{
	(void) arg;

	struct ax12_status status;
	struct ax12_error err;
	struct ax12_request req;
	int id = 0;

	while(1)
	{
		xSemaphoreTake(ax12_mutex, portMAX_DELAY);

		// on regarde s'il y a des choses à envoyer dans la fifo (plutôt de la configuration)
		if( ax12_device[id].request_size)
		{
			req = ax12_device[id].request_fifo[ax12_device[id].request_begin];
			ax12_device[id].request_begin = (ax12_device[id].request_begin + 1) % AX12_MAX_REQUEST;
			ax12_device[id].request_size--;
			xSemaphoreGive(ax12_mutex);

			// la tache qui envoi le message ne veut pas de reponse
			if(req.status == 0)
			{
				req.status = &status;
			}

			uint8_t i = 0;
			do
			{
				ax12_send(&req);

				// delai entre 2 messages sur le bus
				vTaskDelay(AX12_INTER_FRAME_TIME);
				i++;
			}while(req.status->error.transmit_error && i < AX12_MAX_RETRY && req.status->error.transmit_error != ERR_USART_TIMEOUT);

			err = req.status->error;
			req.status->complete = 1;
			vTaskSetEvent(EVENT_AX12_SEND_COMPLETE);
		}
		else
		{
			xSemaphoreGive(ax12_mutex);

			if(id > 0 && ax12_device[id].update)
			{
				// rien dans la fifo => lecture de la position de l'ax12
				req.instruction = AX12_INSTRUCTION_READ_DATA;
				req.arg[0] = AX12_PRESENT_POSITION;
				req.arg[1] = 0x02;
				req.argc = 2;
				req.status = &status;
				req.id = id;
				ax12_send(&req);
				vTaskDelay(AX12_INTER_FRAME_TIME);
				err = status.error;
				if( !err.transmit_error )
				{
					ax12_device[id].pos = status.arg[0] + (status.arg[1] << 8);

					if( abs((int)ax12_device[id].pos - (int)ax12_device[id].goal_pos) > 1)
					{
						// on va envoyer la position désirée
						req.instruction = AX12_INSTRUCTION_WRITE_DATA;
						req.arg[0] = AX12_GOAL_POSITION;
						req.arg[1] = (uint8_t) (ax12_device[id].goal_pos & 0xFF);
						req.arg[2] = (uint8_t) ((ax12_device[id].goal_pos >> 8) & 0xFF);
						req.argc = 3;
						ax12_send(&req);
						vTaskDelay(AX12_INTER_FRAME_TIME);
					}
				}
				err = status.error;
			}
		}

		if(req.id < AX12_MAX_ID)
		{
			if(ax12_device[req.id].transmit_error != err.transmit_error || ax12_device[req.id].internal_error != err.internal_error)
			{
				ax12_print_error(req.id, err);
				ax12_device[req.id].transmit_error = err.transmit_error;
				ax12_device[req.id].internal_error = err.internal_error;
			}
		}

		if(err.transmit_error && err.transmit_error != ERR_USART_TIMEOUT)
		{
			// on vide tout ce qui traine dans le buffer de reception
			usart_set_read_dma_size(UART4_HALF_DUPLEX, sizeof(ax12_read_dma_buffer));
			usart_wait_read(UART4_HALF_DUPLEX, AX12_READ_TIMEOUT);
		}

		id = (id + 1) % AX12_MAX_ID;
	}
}

void ax12_send(struct ax12_request *req)
{
	uint32_t res = 0;
	uint8_t write_size = 6 + req->argc;
	uint8_t read_size = write_size;
	uint8_t i;
	uint8_t* ax12_buffer;

	ax12_write_dma_buffer[0] = 0xFF;
	ax12_write_dma_buffer[1] = 0xFF;
	ax12_write_dma_buffer[2] = req->id;
	ax12_write_dma_buffer[3] = 0x02 + req->argc;
	ax12_write_dma_buffer[4] = req->instruction;
	for( i = 0; i < req->argc ; i++)
	{
		ax12_write_dma_buffer[5+i] = req->arg[i];
	}
	ax12_write_dma_buffer[write_size-1] = ax12_checksum(ax12_write_dma_buffer, write_size);

	if(req->id != 0xFE)
	{
		if(req->instruction != AX12_INSTRUCTION_READ_DATA)
		{
			read_size += 6;
		}
		else
		{
			read_size += 6 + req->arg[1];
		}
	}

	usart_set_read_dma_size(UART4_HALF_DUPLEX, read_size);
	usart_send_dma_buffer(UART4_HALF_DUPLEX, write_size);

	res = usart_wait_read(UART4_HALF_DUPLEX, AX12_READ_TIMEOUT);
	if( res )
	{
		goto end;
	}

	// verification des données envoyées
	for(i = 0; i< write_size ; i++)
	{
		if( ax12_write_dma_buffer[i] != ax12_read_dma_buffer[i] )
		{
			// erreur, on n'a pas lus ce qui a été envoyé
			res = ERR_AX12_SEND_CHECK;
			goto end;
		}
	}

	ax12_buffer = ax12_read_dma_buffer + write_size;
	// pas de broadcast => réponse attendue
	if(req->id != 0xFE)
	{
		int size = read_size - write_size;
		if(ax12_buffer[0] != 0xFF || ax12_buffer[1] != 0xFF || ax12_buffer[2] != req->id || ax12_buffer[3] != size - 4)
		{
			// erreur protocole
			res = ERR_AX12_PROTO;
			goto end;
		}

		if( ax12_buffer[size - 1] != ax12_checksum(ax12_buffer, size))
		{
			// erreur checksum
			res = ERR_AX12_CHECKSUM;
			goto end;
		}

		req->status->error.internal_error = ax12_buffer[4];

		if(size == 7)
		{
			req->status->arg[0] = ax12_buffer[5];
		}
		else if(size == 8)
		{
			req->status->arg[0] = ax12_buffer[5];
			req->status->arg[1] = ax12_buffer[6];
		}
	}

end:
	req->status->error.transmit_error = res;
}

void ax12_print_error(int id, struct ax12_error err)
{
	if(err.transmit_error)
	{
		if(err.transmit_error == ERR_AX12_SEND_CHECK)
		{
			log_format(LOG_ERROR, "ax12 %3d : échec de la verification des octets envoyés", id);
		}
		else if(err.transmit_error == ERR_AX12_PROTO)
		{
			log_format(LOG_ERROR, "ax12 %3d : erreur protocole", id);
		}
		else if( err.transmit_error == ERR_AX12_CHECKSUM)
		{
			log_format(LOG_ERROR, "ax12 %3d : somme de verification incompatible", id);
		}
		else
		{
			if(err.transmit_error & ERR_USART_TIMEOUT)
			{
				// TODO trop de log
				//log_format(LOG_ERROR, "ax12 %3d : timeout", id);
			}
			if(err.transmit_error & ERR_USART_READ_SR_FE)
			{
				log_format(LOG_ERROR, "ax12 %3d : desynchro, bruit ou octet \"break\" sur l'usart", id);
			}
			if(err.transmit_error & ERR_USART_READ_SR_NE)
			{
				log_format(LOG_ERROR, "ax12 %3d : bruit sur l'usart", id);
			}
			if(err.transmit_error & ERR_USART_READ_SR_ORE)
			{
				log_format(LOG_ERROR, "ax12 %3d : overrun sur l'usart", id);
			}
		}
	}
	else if(err.internal_error)
	{
		if( err.internal_error & AX12_INPUT_VOLTAGE_ERROR_MASK)
		{
			log_format(LOG_ERROR, "ax12 %3d : erreur interne - problème de tension", id);
		}
		if( err.internal_error & AX12_ANGLE_LIMIT_ERROR_MASK)
		{
			log_format(LOG_ERROR, "ax12 %3d : erreur interne - angle invalide", id);
		}
		if( err.internal_error & AX12_OVERHEATING_ERROR_MASK)
		{
			log_format(LOG_ERROR, "ax12 %3d : erreur interne - surchauffe", id);
		}
		if( err.internal_error & AX12_RANGE_ERROR_MASK)
		{
			log_format(LOG_ERROR, "ax12 %3d : erreur interne - valeur non admissible", id);
		}
		if( err.internal_error & AX12_CHECKSUM_ERROR_MASK)
		{
		// TODO trop de log
//			log_format(LOG_ERROR, "ax12 %3d : erreur interne - somme de verification incompatible", id);
		}
		if( err.internal_error & AX12_OVERLOAD_ERROR_MASK)
		{
			log_format(LOG_ERROR, "ax12 %3d : erreur interne - surcharge de l'actioneur", id);
		}
		if( err.internal_error & AX12_INSTRUCTION_ERROR_MASK)
		{
			log_format(LOG_ERROR, "ax12 %3d : erreur interne - instruction invalide", id);
		}
	}
	else
	{
		// TODO trop de log
//		log_format(LOG_INFO, "ax12 %3d ok", id);
	}
}

struct ax12_error ax12_ping(uint8_t id)
{
	struct ax12_request req;
	struct ax12_status status;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_PING;
	req.argc = 0;
	req.status = &status;

	ax12_send_request(&req);
	return status.error;
}

static void ax12_send_request(struct ax12_request* request)
{
	struct ax12_device* dev;

	if( request->id < AX12_MAX_ID )
	{
		dev = &ax12_device[request->id];
	}
	else
	{
		dev = &ax12_device[0];
	}

	// on souhaite attendre la reponse
	if(request->status)
	{
		memset(request->status, 0x00, sizeof(request->status));
		vTaskClearEvent(EVENT_AX12_SEND_COMPLETE);
	}

	// si c'est plein, on va attendre
	do
	{
		xSemaphoreTake(ax12_mutex, portMAX_DELAY);

		if( dev->request_size < AX12_MAX_REQUEST )
		{
			dev->request_fifo[ dev->request_end ] = *request;
			dev->request_end = (dev->request_end + 1) % AX12_MAX_REQUEST;
			dev->request_size++;
			xSemaphoreGive(ax12_mutex);
			break;
		}
		else
		{
			xSemaphoreGive(ax12_mutex);
			vTaskDelay(ms_to_tick(10));
		}
	}while(1);

	if(request->status)
	{
		do
		{
			vTaskWaitEvent(EVENT_AX12_SEND_COMPLETE, portMAX_DELAY);
			vTaskClearEvent(EVENT_AX12_SEND_COMPLETE);
		}while(	!request->status->complete );
	}
}

uint8_t ax12_read8(uint8_t id, uint8_t offset, struct ax12_error* error)
{
	struct ax12_request req;
	struct ax12_status status;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x01;
	req.argc = 2;
	req.status = &status;

	ax12_send_request(&req);
	*error = status.error;

	return status.arg[0];
}

uint16_t ax12_read16(uint8_t id, uint8_t offset, struct ax12_error* error)
{
	struct ax12_request req;
	struct ax12_status status;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x02;
	req.argc = 2;
	req.status = &status;

	ax12_send_request(&req);
	*error = status.error;

	return status.arg[0] + (status.arg[1] << 8);
}

struct ax12_error ax12_write8(uint8_t id, uint8_t offset, uint8_t data)
{
	struct ax12_request req;
	struct ax12_status status;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = data;
	req.argc = 2;
	req.status = &status;

	ax12_send_request(&req);
	return status.error;
}

struct ax12_error ax12_write16(uint8_t id, uint8_t offset, uint16_t data)
{
	struct ax12_request req;
	struct ax12_status status;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = (uint8_t) (data & 0xFF);
	req.arg[2] = (uint8_t) ((data >> 8) & 0xFF);
	req.argc = 3;
	req.status = &status;

	ax12_send_request(&req);
	return status.error;
}

struct ax12_error ax12_action(uint8_t id)
{
	struct ax12_request req;
	struct ax12_status status;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_ACTION;
	req.argc = 0;
	req.status = &status;

	ax12_send_request(&req);
	return status.error;
}

struct ax12_error ax12_reset(uint8_t id)
{
	struct ax12_request req;
	struct ax12_status status;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_RESET;
	req.argc = 0;
	req.status = 0;

	ax12_send_request(&req);
	return status.error;
}

static uint8_t ax12_checksum(uint8_t* buffer, uint8_t size)
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

struct ax12_error ax12_set_led(uint8_t id, uint8_t on)
{
	return ax12_write8(id, AX12_LED, on);
}

// fonction non inline pour permettre l'utilisation depuis gdb
struct ax12_error ax12_set_moving_speed(uint8_t id, uint16_t speed)
{
	return ax12_write16(id, AX12_MOVING_SPEED, speed & AX12_MAX_MOVING_SPEED);
}

struct ax12_error ax12_set_goal_position(uint8_t id, int32_t alpha)
{
	struct ax12_error err;

	// modulo 1 tour => retour dans [ 0 ; 1 tour = 2^26 [
	if(alpha < 0)
	{
		alpha = 0x4000000 - ((-alpha) & 0x3ffffff);
	}
	else
	{
		alpha &= 0x3ffffff;
	}

	// retour dans [ -0.5 ; 0.5 ] tour
	if( alpha & 0x2000000 )
	{
		alpha -= 0x4000000;
	}

	// passage en unité ax12
	// zero au milieu qui vaut "0x1ff"
	alpha = (((alpha >> 12) * 1228) >> 14) + 0x1ff;

	uint16_t min = 0;
	uint16_t max = 0x3ff;

	// si c'est un ax12 connu, on regarde les limites
	if( id < AX12_MAX_ID )
	{
		min = ax12_device[id].min_goal;
		max = ax12_device[id].max_goal;
	}

	// saturation
	if(alpha < min)
	{
		alpha = min;
	}
	else if( alpha > max)
	{
		alpha = max;
	}

	if( id < AX12_MAX_ID )
	{
		// utilisation de la tache pour la mise à jour
		xSemaphoreTake(ax12_mutex, portMAX_DELAY);
		ax12_device[id].goal_pos = alpha;
		err.transmit_error = ax12_device[id].transmit_error;
		err.internal_error = ax12_device[id].internal_error;
		xSemaphoreGive(ax12_mutex);
	}
	else
	{
		// envoi simple
		err = ax12_write16(id, AX12_GOAL_POSITION, (uint16_t) alpha);
	}

	return err;
}

struct ax12_error ax12_set_torque_limit(uint8_t id, uint16_t torque_limit)
{
	if( vTaskGetEvent() & EVENT_END )
	{
		torque_limit = 0x00;
	}

	return ax12_write16(id, AX12_TORQUE_LIMIT, torque_limit & AX12_MAX_TORQUE_LIMIT);
}

struct ax12_error ax12_set_torque_limit_eeprom(uint8_t id, uint16_t torque_limit)
{
	return ax12_write16(id, AX12_TORQUE_LIMIT_EEPROM, torque_limit & AX12_MAX_TORQUE_LIMIT);
}

struct ax12_error ax12_set_torque_enable(uint8_t id, uint8_t enable)
{
	return ax12_write8(id, AX12_TORQUE_ENABLE, enable & 0x01);
}

int32_t ax12_get_position(uint8_t id, struct ax12_error* error)
{
	int32_t alpha = 0x1ff;
	if(id < AX12_MAX_ID )
	{
		xSemaphoreTake(ax12_mutex, portMAX_DELAY);
		alpha = ax12_device[id].pos;
		error->transmit_error = ax12_device[id].transmit_error;
		error->internal_error = ax12_device[id].internal_error;
		xSemaphoreGive(ax12_mutex);
	}
	else
	{
		alpha = ax12_read16(id, AX12_PRESENT_POSITION, error);
	}

	// passage en unité fx
	// zero au milieu qui vaut "0x1ff"
	return (((alpha - 0x1ff) << 14) / 1288) << 12;
}

static void ax12_cmd(void* arg)
{
	struct ax12_cmd_param* param = (struct ax12_cmd_param*)arg;
	struct ax12_error err;
	int32_t alpha;

	switch(param->cmd_id)
	{
		case AX12_CMD_SCAN:
			ax12_cmd_scan();
			break;
		case AX12_CMD_SET_ID:
			ax12_cmd_set_id(param->id, param->param & 0xff);
			break;
		case AX12_CMD_SET_GOAL_POSITION:
			ax12_set_goal_position(param->id, param->param);
			break;
		case AX12_CMD_GET_POSITION:
			alpha = ax12_get_position(param->id, &err);
			if(!err.transmit_error)
			{
				log_format(LOG_INFO, "ax12 %u - pos %d", param->id, (int)alpha);
			}
			break;
		default:
			log_format(LOG_ERROR, "unknown ax12 command : %d", param->cmd_id);
			break;
	}
}

static void ax12_cmd_scan()
{
	int i;
	struct ax12_error error;

	log(LOG_INFO, "ax12 - scan");

	for(i = 1; i<254; i++)
	{
		error = ax12_ping(i);
		if(! error.transmit_error)
		{
			log_format(LOG_INFO, "ax12 %3d détecté - status %#.2x", i, error.internal_error);
		}
	}

	log(LOG_INFO, "ax12 - end scan");
}

static void ax12_cmd_set_id(uint8_t old_id, uint8_t id)
{
	struct ax12_error error;

	if(id <= 0xfd)
	{
		error = ax12_write8(old_id, AX12_ID, id);
		if( error.transmit_error)
		{
			log(LOG_ERROR, "erreur de transmission");
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

void ax12_set_goal_limit(uint8_t id, uint16_t min, uint16_t max)
{
	if( id >= AX12_MAX_ID )
	{
		return;
	}

	if(min > 0x3ff)
	{
		min = 0x3ff;
	}

	if( max > 0x3ff)
	{
		max = 0x3ff;
	}

	ax12_device[id].min_goal = min;
	ax12_device[id].max_goal = max;
}

void ax12_auto_update(uint8_t id, uint8_t update)
{
	if( id < AX12_MAX_ID )
	{
		ax12_device[id].update = update;
	}
}