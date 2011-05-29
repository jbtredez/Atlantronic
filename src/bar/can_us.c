//! @file can_us.c
//! @brief CAN - US
//! @author Atlantronic

//! interface : us.h
#include "bar/priority.h"

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#include "kernel/event.h"
#include "kernel/rcc.h"

///include all files for can third party
#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include "kernel/can/can_us.h"

#include <string.h>

#include "kernel/portmacro.h"

#define US_STACK_SIZE           64
#define US_QUEUE_SIZE             10
#define US_INTER_FRAME_TIME      720
#define US_MAX_RETRY               3

uint16_t can_us_sample_value[US_MAX];

static xQueueHandle us_queue;
void can_us_callback(struct can_msg *msg);
static uint8_t us_send(uint32_t id, enum us_id us_id);

static void us_task(void* arg)
{
	(void) arg;
	uint8_t i=0;
	uint8_t req;
	int8_t res;

	while(1)
	{
		while(xQueueReceive(us_queue, &req, 0))
		{
			uint8_t i = 0;
			  
			do
			{
			      if(req >= US_MAX) 
				  error_raise(ERR_US_UNKNOWN_US);
			      else
			      {
				res = us_send(CAN_US_RESPONSE_ID, req);
				// delai entre 2 messages sur le bus
				vTaskDelay(US_INTER_FRAME_TIME);
			      }
			}while(res && i < US_MAX_RETRY);
		}
		
		for(i=0; i<US_MAX; i++)
		  can_us_sample_value[i] = get_US(i);
		
	}
}

int can_us_module_init()
{
	can_register(CAN_US_RESPONSE_ID, CAN_STANDARD_FORMAT, can_us_callback);
	
	us_queue = xQueueCreate(US_QUEUE_SIZE, sizeof(uint8_t));

	if(us_queue == 0)
	{
		return ERR_INIT_US;
	}
	
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(us_task, "can_us", US_STACK_SIZE, NULL, PRIORITY_TASK_US, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_US;
	}

	return 0;
}


void can_us_callback(struct can_msg *msg)
{
	if(msg->data[0] < US_MAX && msg->size == 5)
	{
		xQueueSendToBack(us_queue, &(msg->data[0]), 0);
	}
	else
	{
		 error_raise(ERR_US_UNKNOWN_US);
		// erreur TODO log
		// on fait rien, attente du prochain message CAN
	}
}

uint16_t us_get_state(enum us_id us_id)
{
	uint32_t res = 0;

	if(us_id < US_MAX)
	{
		portENTER_CRITICAL();
		res = can_us_sample_value[us_id];
		portEXIT_CRITICAL();
	}
	else 	error_raise(ERR_US_UNKNOWN_US);
			    
	return res;
}

uint8_t us_send(uint32_t can_id, enum us_id us_id)
{
    uint32_t res = 0;
    uint16_t data = 0;
    struct can_msg msg;
    portTickType timeout = ms_to_tick(100);
    
    msg.id = can_id;
    msg.data[0] = us_id;
    data = us_get_state(us_id);
    
    memcpy(&(msg.data) + 1, &data, 2);
    res = can_write(&msg,  timeout);
    
    return 0;
}