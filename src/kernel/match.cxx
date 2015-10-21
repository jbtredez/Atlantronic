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
#include "kernel/driver/pwm.h"
#include "match.h"
#include "disco/recalage.h"

#define MATCH_STACK_SIZE           300
uint32_t match_time = 90000; //!< duree du match en ms
volatile int match_color;
volatile uint8_t match_go;
static volatile uint8_t match_color_change_enable;
volatile uint8_t match_enable_go = 0;
static xQueueHandle match_queue_go;
static xQueueHandle match_queue_recal;
volatile int match_end;

static void match_task(void *arg);

static void match_cmd_set_time(void* arg, void* data);
static void match_cmd_go(void* arg, void* data);
static void match_cmd_color(void* arg, void* data);
static void match_wait_recal();

static int match_module_init()
{
	match_end = 0;
	match_color = COLOR_GREEN;
	match_go = 0;
	match_queue_go = xQueueCreate(1, 0);
	match_queue_recal = xQueueCreate(1, 0);
	match_color_change_enable = 1;

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(match_task, "match", MATCH_STACK_SIZE, NULL, PRIORITY_TASK_MATCH, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_END;
	}

	usb_add_cmd(USB_CMD_MATCH, &match_cmd_go, NULL);
	usb_add_cmd(USB_CMD_COLOR, &match_cmd_color, NULL);
	usb_add_cmd(USB_CMD_MATCH_TIME, &match_cmd_set_time, NULL);

	// on force la lecture io au boot
	match_set_color_from_isr();

	return 0;
}

module_init(match_module_init, INIT_END);

static void match_task(void *arg)
{
	(void) arg;

	match_wait_recal();
	recalage();
	match_color_change_enable = 0;
	match_enable_go = 1;
	match_wait_go();

	uint32_t msg[3];
	struct systime t = systick_get_time();
	msg[0] = t.ms;
	msg[1] = t.ns;
	msg[2] = match_time;
	usb_add(USB_GO, &msg, sizeof(msg));

	vTaskDelay(match_time);
	log(LOG_INFO, "Fin du match");
	exitModules();
	match_end = 1;

	vTaskSuspend(0);
}

void match_wait_go()
{
	xQueuePeek(match_queue_go, NULL, portMAX_DELAY);
}

void match_wait_recal()
{
	xQueuePeek(match_queue_recal, NULL, portMAX_DELAY);
}

static void match_cmd_set_time(void* /*arg*/, void* data)
{
	// temps passé en ms
	uint32_t time = *((uint32_t*) data);
	if( ! match_go )
	{
		match_time = time;
		log_format(LOG_INFO, "duree du match => %d ms", (int)match_time);
	}
}

static void match_cmd_go(void* /*arg*/, void* data)
{
	struct gpio_cmd_match_arg* cmd_arg = (struct gpio_cmd_match_arg*) data;

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
		case MATCH_CMD_RECALAGE:
			xQueueSend(match_queue_recal, NULL, 0);
			break;
		default:
			log_format(LOG_ERROR, "unknown go cmd %d", cmd_arg->cmd);
			break;
	}
}

static void match_cmd_color(void* /*arg*/, void* data)
{
	int8_t new_color = *((int8_t*) data);
	if(match_go == 0 && match_color_change_enable)
	{
		if(new_color == COLOR_GREEN)
		{
			pwm_set(PWM_1, 0.5);
			pwm_set(PWM_2, 0);
			match_color = COLOR_GREEN;
			log(LOG_INFO, "couleur => vert");
		}
		else
		{
			pwm_set(PWM_1, 0);
			pwm_set(PWM_2, 0.5);
			match_color = COLOR_PURPLE;
			log(LOG_INFO, "couleur => violet");
		}
	}
}

portBASE_TYPE match_go_from_isr(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	bool ioActive = gpio_get(IO_GO);

	if( ioActive )
	{
		if( match_enable_go )
		{
			match_go = 1;
			systick_start_match_from_isr();
			xQueueSendFromISR(match_queue_go, NULL, &xHigherPriorityTaskWoken);
		}
	}
	else
	{
		// recalage
		xQueueSendFromISR(match_queue_recal, NULL, &xHigherPriorityTaskWoken);
	}

	return xHigherPriorityTaskWoken;
}

portBASE_TYPE match_set_color_from_isr(void)
{
	if(match_go == 0 && match_color_change_enable)
	{
		int ioColor = GPIO_MASK(IO_COLOR) & gpio_get_state();
		if(ioColor)
		{
			pwm_set(PWM_1, 0.5);
			pwm_set(PWM_2, 0);
			match_color = COLOR_GREEN;
		}
		else
		{
			pwm_set(PWM_1, 0);
			pwm_set(PWM_2, 0.5);
			match_color = COLOR_PURPLE;
		}
	}
	return 0;
}
