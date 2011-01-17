//! @file ax12.c
//! @brief Gestion AX12
//! @author Jean-Baptiste Trédez

#include "io/ax12.h"
#include "module.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define AX12_INSTRUCTION_PING             0x01
#define AX12_INSTRUCTION_READ_DATA        0x02
#define AX12_INSTRUCTION_WRITE_DATA       0x03
#define AX12_INSTRUCTION_REG_WRITE        0x04
#define AX12_INSTRUCTION_ACTION           0x05
#define AX12_INSTRUCTION_RESET            0x06
#define AX12_INSTRUCTION_SYNC_WRITE       0x83

#define AX12_QUEUE_SIZE     10
#define AX12_STACK_SIZE     64
#define AX12_ARG_MAX         3

struct ax12_request
{
	uint8_t id;
	uint8_t instruction;
	uint8_t argc;
	uint8_t arg[AX12_ARG_MAX];
};

static void ax12_task(void *arg);
static xQueueHandle ax12_queue;
static uint8_t ax12_buffer[6 + AX12_ARG_MAX];
static uint8_t ax12_checksum(uint8_t* buffer, uint8_t size);


static int ax12_module_init()
{
	ax12_queue = xQueueCreate(AX12_QUEUE_SIZE, sizeof(struct ax12_request));

	if(ax12_queue == 0)
	{
		return ERR_INIT_AX12;
	}

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(ax12_task, (const signed char *) "ax12", AX12_STACK_SIZE, NULL, PRIORITY_TASK_AX12, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_AX12;
	}

	ax12_reset(0xFE);
	ax12_ping(1);
//	ax12_action(1);
//	ax12_write8(1, 0x02, 0x06);
//	ax12_write16(1, 0x02, 0x1516);
	return 0;
}

module_init(ax12_module_init, INIT_AX12);

static void ax12_task(void* arg)
{
	(void) arg;

	struct ax12_request req;

	while(1)
	{
		if(xQueueReceive(ax12_queue, &req, portMAX_DELAY))
		{
			uint8_t size = 6 + req.argc;
			uint8_t i;

			ax12_buffer[0] = 0xFF;
			ax12_buffer[1] = 0xFF;
			ax12_buffer[2] = req.id;
			ax12_buffer[3] = 0x02 + req.argc;
			ax12_buffer[4] = req.instruction;
			for( i = 5; i < size - 1 ; i++)
			{
				ax12_buffer[i] = req.arg[i];
			}
			ax12_buffer[i] = ax12_checksum(ax12_buffer, size);
			usart_write(ax12_buffer, size);

			// TODO read
		}
	}
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

void ax12_read(uint8_t id, uint8_t start, uint8_t length)
{
	// TODO
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
