//! @file ax12.c
//! @brief Gestion AX12
//! @author Atlantronic

#include "ax12.h"
#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/event.h"
#include "kernel/error.h"

#define AX12_INSTRUCTION_READ_COMPLETE    0x00
#define AX12_INSTRUCTION_PING             0x01
#define AX12_INSTRUCTION_READ_DATA        0x02
#define AX12_INSTRUCTION_WRITE_DATA       0x03
#define AX12_INSTRUCTION_REG_WRITE        0x04
#define AX12_INSTRUCTION_ACTION           0x05
#define AX12_INSTRUCTION_RESET            0x06
#define AX12_INSTRUCTION_SYNC_WRITE       0x83

#define AX12_QUEUE_SIZE             10
#define AX12_STACK_SIZE            100
#define AX12_ARG_MAX                 3
#define AX12_READ_TIMEOUT       720000
#define AX12_INTER_FRAME_TIME      720
#define AX12_MAX_RETRY               3

struct ax12_request
{
	uint8_t id;
	uint8_t instruction;
	uint8_t argc;
	uint8_t arg[AX12_ARG_MAX];
	struct ax12_request* rep;
};

static void ax12_task(void *arg);
static xQueueHandle ax12_queue;
static uint8_t ax12_write_dma_buffer[6 + AX12_ARG_MAX];
static uint8_t ax12_read_dma_buffer[2*(6 + AX12_ARG_MAX)];
static uint8_t ax12_checksum(uint8_t* buffer, uint8_t size);
static uint32_t ax12_send(struct ax12_request *req);

static int ax12_module_init()
{
	usart_open(UART4_HALF_DUPLEX, 1000000);

	ax12_queue = xQueueCreate(AX12_QUEUE_SIZE, sizeof(struct ax12_request));

	if(ax12_queue == 0)
	{
		return ERR_INIT_AX12;
	}

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(ax12_task, "ax12", AX12_STACK_SIZE, NULL, PRIORITY_TASK_AX12, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_AX12;
	}

	usart_set_write_dma_buffer(UART4_HALF_DUPLEX, ax12_write_dma_buffer);
	usart_set_read_dma_buffer(UART4_HALF_DUPLEX, ax12_read_dma_buffer);

	return 0;
}

module_init(ax12_module_init, INIT_AX12);

static void ax12_module_exit()
{
//	ax12_set_torque_limit(0xfe, 0x00);
//	ax12_set_torque_limit(0xfe, 0x00);
//	ax12_set_torque_limit(0xfe, 0x00);
}

module_exit(ax12_module_exit, EXIT_AX12);

static void ax12_task(void* arg)
{
	(void) arg;

	struct ax12_request req;
	uint32_t res;

	while(1)
	{
		if(xQueueReceive(ax12_queue, &req, portMAX_DELAY))
		{
			uint8_t i = 0;
			do
			{
				res = ax12_send(&req);
				// delai entre 2 messages sur le bus
				vTaskDelay(AX12_INTER_FRAME_TIME);
			}while(res && i < AX12_MAX_RETRY);
		}
	}
}

uint32_t ax12_send(struct ax12_request *req)
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
		// TODO code erreur
		//error_raise(res);
		goto end;
	}

	// verification des données envoyées
	for(i = 0; i< write_size ; i++)
	{
		if( ax12_write_dma_buffer[i] != ax12_read_dma_buffer[i] )
		{
			// erreur, on n'a pas lus ce qui a été envoyé
			res = ERR_AX12_SEND_CHECK;
			error_raise(ERR_AX12_SEND_CHECK);
			goto end;
		}
	}

	ax12_buffer = ax12_read_dma_buffer + write_size;
	// pas de broadcast (et status des ax12 à 2) => réponse attendue
	if(req->id != 0xFE)
	{
		int size = read_size - write_size;
		if(ax12_buffer[0] != 0xFF || ax12_buffer[1] != 0xFF || ax12_buffer[2] != req->id || ax12_buffer[3] != size - 4)
		{
			// erreur protocole
			res = -1;
			goto end;
		}

		if( ax12_buffer[size - 1] != ax12_checksum(ax12_buffer, size))
		{
			// erreur checksum
			res = -1;
			goto end;
		}

		// traitement du message reçu
		if(ax12_buffer[4])
		{
			// erreur ax12
			res = -1;
			goto end;
		}

		if(size == 7)
		{
			req->rep->arg[0] = ax12_buffer[5];
			req->rep->instruction = AX12_INSTRUCTION_READ_COMPLETE;
			vTaskSetEvent(EVENT_AX12_READ_COMPLETE);
		}
		else if(size == 8)
		{
			req->rep->arg[0] = ax12_buffer[5];
			req->rep->arg[1] = ax12_buffer[6];
			req->rep->instruction = AX12_INSTRUCTION_READ_COMPLETE;
			vTaskSetEvent(EVENT_AX12_READ_COMPLETE);
		}
	}

end:
	return res;
}

void ax12_ping(uint8_t id)
{
	struct ax12_request req;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_PING;
	req.argc = 0;

	// si c'est plein, on va attendre
	xQueueSendToBack(ax12_queue, &req, portMAX_DELAY);
}

uint8_t ax12_read8(uint8_t id, uint8_t offset)
{
	volatile struct ax12_request req;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x01;
	req.argc = 2;
	req.rep = (struct ax12_request*) &req;

	vTaskClearEvent(EVENT_AX12_READ_COMPLETE);

	// si c'est plein, on va attendre
	xQueueSendToBack(ax12_queue, (void*) &req, portMAX_DELAY);

	do
	{
		vTaskWaitEvent(EVENT_AX12_READ_COMPLETE, portMAX_DELAY);
		vTaskClearEvent(EVENT_AX12_READ_COMPLETE);
	}while(	req.instruction != AX12_INSTRUCTION_READ_COMPLETE );

	return req.arg[0];
}

uint16_t ax12_read16(uint8_t id, uint8_t offset)
{
	volatile struct ax12_request req;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x02;
	req.argc = 2;
	req.rep = (struct ax12_request*) &req;

	vTaskClearEvent(EVENT_AX12_READ_COMPLETE);

	// si c'est plein, on va attendre
	xQueueSendToBack(ax12_queue, (void*) &req, portMAX_DELAY);

	do
	{
		vTaskWaitEvent(EVENT_AX12_READ_COMPLETE, portMAX_DELAY);
		vTaskClearEvent(EVENT_AX12_READ_COMPLETE);
	}while(	req.instruction != AX12_INSTRUCTION_READ_COMPLETE );

	return req.arg[0] + (req.arg[1] << 8);
}

void ax12_write8(uint8_t id, uint8_t offset, uint8_t data)
{
	struct ax12_request req;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = data;
	req.argc = 2;

	// si c'est plein, on va attendre
	xQueueSendToBack(ax12_queue, &req, portMAX_DELAY);
}

void ax12_write16(uint8_t id, uint8_t offset, uint16_t data)
{
	struct ax12_request req;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = (uint8_t) (data & 0xFF);
	req.arg[2] = (uint8_t) ((data >> 8) & 0xFF);
	req.argc = 3;

	// si c'est plein, on va attendre
	xQueueSendToBack(ax12_queue, &req, portMAX_DELAY);
}

void ax12_action(uint8_t id)
{
	struct ax12_request req;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_ACTION;
	req.argc = 0;

	// si c'est plein, on va attendre
	xQueueSendToBack(ax12_queue, &req, portMAX_DELAY);
}

void ax12_reset(uint8_t id)
{
	struct ax12_request req;
	req.id = id;
	req.instruction = AX12_INSTRUCTION_RESET;
	req.argc = 0;

	// si c'est plein, on va attendre
	xQueueSendToBack(ax12_queue, &req, portMAX_DELAY);
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

// fonction non inline pour permettre l'utilisation depuis gdb
void ax12_set_led(uint8_t id, uint8_t on)
{
	ax12_write8(id, AX12_LED, on);
}

// fonction non inline pour permettre l'utilisation depuis gdb
void ax12_set_moving_speed(uint8_t id, uint16_t speed)
{
	ax12_write16(id, AX12_MOVING_SPEED, speed & AX12_MAX_MOVING_SPEED);
}

// fonction non inline pour permettre l'utilisation depuis gdb
void ax12_set_goal_position(uint8_t id, uint16_t goal)
{
	ax12_write16(id, AX12_GOAL_POSITION, goal & AX12_MAX_GOAL_POSITION);
}

void ax12_set_id(uint8_t old_id, uint8_t id)
{
	if(id <= 0xfd)
	{
		ax12_write8(old_id, AX12_ID, id);
	}
}

void ax12_set_torque_limit(uint8_t id, uint16_t torque_limit)
{
	if( vTaskGetEvent() & EVENT_END )
	{
		torque_limit = 0x00;
	}

	ax12_write16(id, AX12_TORQUE_LIMIT, torque_limit & AX12_MAX_TORQUE_LIMIT);
}

void ax12_set_torque_limit_eeprom(uint8_t id, uint16_t torque_limit)
{
	ax12_write16(id, AX12_TORQUE_LIMIT_EEPROM, torque_limit & AX12_MAX_TORQUE_LIMIT);
}

void ax12_set_torque_enable(uint8_t id, uint8_t enable)
{
	ax12_write8(id, AX12_TORQUE_ENABLE, enable & 0x01);
}

uint16_t ax12_get_position(uint8_t id)
{
	return ax12_read16(id, AX12_PRESENT_POSITION);
}