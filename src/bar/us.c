#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
#include "bar/gpio.h"
#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include <string.h>

//! @todo réglage au pif
#define US_STACK_SIZE       100
#define US_TIMEOUT                  200 
#define US_DELAY_AFTER_TIMEOUT      100

static void us_task_right();
static void us_task_back();
static void us_task_front();
static int us_module_init();
static uint16_t us_state[US_MAX];
static struct can_msg can_us_msg[US_MAX];

static int us_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(us_task_right, "us_right", US_STACK_SIZE, NULL, PRIORITY_TASK_TEST_US, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	err = xTaskCreate(us_task_back, "us_back", US_STACK_SIZE, NULL, PRIORITY_TASK_TEST_US, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	err = xTaskCreate(us_task_front, "us_front", US_STACK_SIZE, NULL, PRIORITY_TASK_TEST_US, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	int i = 0;
	for( ; i < US_MAX ; i++)
	{
		can_us_msg[i].format = CAN_STANDARD_FORMAT;
		can_us_msg[i].type = CAN_DATA_FRAME;
		can_us_msg[i].size = 5;
		can_us_msg[i].id = CAN_US;
	}

	return 0;
}

module_init(us_module_init, INIT_TEST_PINCE);

volatile uint8_t mask = 0;

static void us_task_right()
{
	int timeout;

	can_us_msg[US_RIGHT].data[0] = US_RIGHT;

	while(1)
	{
		timeout = 0;
		vTaskDelay(ms_to_tick(50));
		gpio_send_us( GPIO_US0 );

		do
		{
			vTaskDelay(ms_to_tick(1));
			timeout += 1;
			gpio_get_us(us_state, sizeof(us_state));
		}
		while( us_state[US_RIGHT] == 0 && timeout < US_TIMEOUT);

		if( us_state[US_RIGHT] < 350 && us_state[US_RIGHT] > 0)
		{
			mask |= 0x02;
		}
		else
		{
			mask &= ~0x02;
		}
		
		setLed(mask);

		memcpy(can_us_msg[US_RIGHT].data + 1, &us_state[US_RIGHT], 2);
		can_write(&can_us_msg[US_RIGHT], portMAX_DELAY);

		if( us_state[US_RIGHT] > 10000 )
		{
			vTaskDelay(ms_to_tick(US_DELAY_AFTER_TIMEOUT));
		}
	}

	vTaskDelete(NULL);
}

static void us_task_front()
{
	int timeout;

	can_us_msg[US_FRONT].data[0] = US_FRONT;

	while(1)
	{
		timeout = 0;
		vTaskDelay(ms_to_tick(50));
		gpio_send_us( GPIO_US1 );

		do
		{
			vTaskDelay(ms_to_tick(1));
			timeout += 1;
			gpio_get_us(us_state, sizeof(us_state));
		}
		while( us_state[US_FRONT] == 0 && timeout < US_TIMEOUT);

		if( us_state[US_FRONT] < 350 && us_state[US_FRONT] > 0)
		{
			mask |= 0x04;
		}
		else
		{
			mask &= ~0x04;
		}
		
		setLed(mask);

		memcpy(can_us_msg[US_FRONT].data + 1, &us_state[US_FRONT], 2);
		can_write(&can_us_msg[US_FRONT], portMAX_DELAY);

		if( us_state[US_FRONT] > 10000 )
		{
			vTaskDelay(ms_to_tick(US_DELAY_AFTER_TIMEOUT));
		}
	}

	vTaskDelete(NULL);
}

static void us_task_back()
{
	int timeout;

	can_us_msg[US_BACK].data[0] = US_BACK;

	while(1)
	{
		timeout = 0;
		vTaskDelay(ms_to_tick(50));
		gpio_send_us( GPIO_US2 );

		do
		{
			vTaskDelay(ms_to_tick(1));
			timeout += 1;
			gpio_get_us(us_state, sizeof(us_state));
		}
		while( us_state[US_BACK] == 0 && timeout < US_TIMEOUT);

		if( us_state[US_BACK] < 350 && us_state[US_BACK] > 0)
		{
			mask |= 0x08;
		}
		else
		{
			mask &= ~0x08;
		}
		
		setLed(mask);

		memcpy(can_us_msg[US_BACK].data + 1, &us_state[US_BACK], 2);
		can_write(&can_us_msg[US_BACK], portMAX_DELAY);

		if( us_state[US_BACK] > 10000 )
		{
			vTaskDelay(ms_to_tick(US_DELAY_AFTER_TIMEOUT));
		}
	}

	vTaskDelete(NULL);
}

