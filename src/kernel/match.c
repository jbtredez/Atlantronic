//! @file end.c
//! @brief Task waiting during the math, will send halt event.
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "kernel/driver/power.h"
#include "kernel/driver/exti.h"
#include "kernel/queue.h"
#include "kernel/driver/io.h"
#include "match.h"

#define MATCH_STACK_SIZE           100
uint32_t match_time = 90000; //!< duree du match en ms
volatile int match_color;
volatile uint8_t match_go;
volatile uint8_t match_color_change_enable;
volatile uint8_t match_enable_go = 0;
static volatile struct systime match_color_change_time;
static xQueueHandle match_queue_go;
volatile int match_end;

static void match_task(void *arg);

static void match_cmd_set_time(void* arg);
static void match_cmd_go(void* arg);
static void match_cmd_color(void* arg);

static int match_module_init()
{
	match_end = 0;
	match_color = COLOR_UNKNOWN;
	match_go = 0;
	match_queue_go = xQueueCreate(1, 0);
	match_color_change_enable = 1;

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(match_task, "match", MATCH_STACK_SIZE, NULL, PRIORITY_TASK_MATCH, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_END;
	}

	usb_add_cmd(USB_CMD_GO, &match_cmd_go);
	usb_add_cmd(USB_CMD_COLOR, &match_cmd_color);
	usb_add_cmd(USB_CMD_MATCH_TIME, &match_cmd_set_time);

	return 0;
}

module_init(match_module_init, INIT_END);

static void match_task(void *arg)
{
	(void) arg;

	match_wait_go();
	uint32_t msg[3];
	struct systime t = systick_get_time();
	msg[0] = t.ms;
	msg[1] = t.ns;
	msg[2] = match_time;
	usb_add(USB_GO, &msg, sizeof(msg));
	vTaskDelay(match_time);
	match_end = 1;
	log(LOG_INFO, "Fin du match");

	power_set(POWER_OFF_END_MATCH);
	exitModules();

	vTaskSuspend(0);
}

void match_wait_go()
{
	xQueuePeek(match_queue_go, NULL, portMAX_DELAY);
}

static void match_cmd_set_time(void* arg)
{
	// temps passé en ms
	uint32_t time = *((uint32_t*) arg);
	if( ! match_go )
	{
		match_time = time;
		log_format(LOG_INFO, "duree du match => %d ms", (int)match_time);
	}
}

static void match_cmd_go(void * arg)
{
	struct gpio_cmd_go_arg* cmd_arg = (struct gpio_cmd_go_arg*) arg;

	switch(cmd_arg->cmd)
	{
		case MATCH_CMD_ENABLE_GO:
			if( match_enable_go != 1 )
			{
				match_enable_go = 1;
				log(LOG_INFO, "enable GO");
			}
			break;
		case MATCH_CMD_GO:
			if(match_enable_go)
			{
				log(LOG_INFO, "usb go");
				match_go = 1;
				systick_start_match();
				xQueueSend(match_queue_go, NULL, 0);
			}
			break;
		default:
			log_format(LOG_ERROR, "unknown go cmd %d", cmd_arg->cmd);
			break;
	}
}

static void match_cmd_color(void* arg)
{
	uint8_t new_color = *((uint8_t*) arg);
	if(match_go == 0 && match_color_change_enable)
	{
		if(new_color == COLOR_GREEN)
		{
			match_color = COLOR_GREEN;
			log(LOG_INFO, "couleur => vert");
		}
		else
		{
			match_color = COLOR_YELLOW;
			log(LOG_INFO, "couleur => jaune");
		}
	}
}

portBASE_TYPE match_go_from_isr(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	if( match_enable_go )
	{
		match_go = 1;
		systick_start_match_from_isr();
		xQueueSendFromISR(match_queue_go, NULL, &xHigherPriorityTaskWoken);
	}

	return xHigherPriorityTaskWoken;
}

portBASE_TYPE match_set_color_from_isr(void)
{
	if(match_go == 0 && match_color_change_enable)
	{
		struct systime t = systick_get_time_from_isr();
		struct systime dt = timediff(t, match_color_change_time);
		if( dt.ms > 300)
		{
			match_color_change_time = t;
			if(match_color == COLOR_YELLOW)
			{
				match_color = COLOR_GREEN;
			}
			else
			{
				match_color = COLOR_YELLOW;
			}
		}
	}
	return 0;
}
