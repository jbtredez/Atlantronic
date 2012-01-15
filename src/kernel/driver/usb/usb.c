#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"
#include "kernel/event.h"
#include "kernel/driver/usb/usb_lib.h"
#include "kernel/driver/usb/usb_istr.h"
#include "kernel/driver/usb/usb_pwr.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

#define USB_BUFER_SIZE          4096
#define USB_STACK_SIZE           350

static unsigned char usb_buffer[USB_BUFER_SIZE];
static int usb_buffer_begin;
static int usb_buffer_end;
static unsigned int usb_write_size;
static unsigned char usb_rx_buffer[64];
static unsigned int usb_read_size;
static xSemaphoreHandle usb_mutex;
static void (*usb_cmd[USB_CMD_NUM])(void*);

void usb_task(void *);
static volatile unsigned int usb_endpoint_ready;

static int usb_module_init(void)
{
	int i;

	usb_buffer_begin = 0;
	usb_buffer_end = 0;
	usb_write_size = 0;
	usb_endpoint_ready = 1;
	usb_read_size = 0;

	for(i = 0; i < USB_CMD_NUM; i++)
	{
		usb_cmd[i] = NULL;
	}

	usb_mutex = xSemaphoreCreateMutex();

	if(usb_mutex == NULL)
	{
		return ERR_INIT_USB;
	}

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->AHBENR |= RCC_AHBENR_OTGFSEN; // USB OTG FS clock enable

	NVIC_SetPriority(OTG_FS_IRQn, PRIORITY_IRQ_USB);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	USB_Init();

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(usb_task, "usb", USB_STACK_SIZE, NULL, PRIORITY_TASK_USB, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_USB;
	}

	return 0;
}

module_init(usb_module_init, INIT_USB);

void usb_write_byte(unsigned char byte)
{
	usb_buffer[usb_buffer_end] = byte;
	usb_buffer_end = (usb_buffer_end + 1) % USB_BUFER_SIZE;
	if( usb_buffer_end == usb_buffer_begin )
	{
		// buffer circulaire plein, on ecrase les vieux trucs
		// FIXME Attention à la synchro de l'autre coté
		usb_buffer_begin = (usb_buffer_begin + 1) % USB_BUFER_SIZE;
	}
}

void usb_add(uint16_t type, void* msg, uint16_t size)
{
	if(size == 0)
	{
		return;
	}

	// on se reserve le buffer circulaire que pour les log et erreurs s'il n'y a personne sur l'usb
	if( bDeviceState != CONFIGURED && type != USB_LOG && type != USB_ERR )
	{
		return;
	}

	xSemaphoreTake(usb_mutex, portMAX_DELAY);

	usb_write_byte( type >> 8 );
	usb_write_byte( type & 0xff );
	usb_write_byte( size >> 8 );
	usb_write_byte( size & 0xff );

	for( ; size--; )
	{
		usb_write_byte(*((unsigned char*)msg));
		msg++;
	}
	vTaskSetEvent(EVENT_USB);

	xSemaphoreGive(usb_mutex);
}

void usb_add_cmd(enum usb_cmd id, void (*cmd)(void*))
{
	usb_cmd[id] = cmd;
}

//! Usb task
void usb_task(void * arg)
{
	(void) arg;

	while(1)
	{
		while( bDeviceState != CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		if( usb_read_size )
		{
			int id = usb_rx_buffer[0];
			if( id < USB_CMD_NUM )
			{
				if( usb_cmd[id] )
				{
					usb_cmd[id](usb_rx_buffer+1);
				}
				else
				{
					log_format(LOG_ERROR, "command %d not found", id);
				}
			}
			else
			{
				log_format(LOG_ERROR, "command %d not found", id);
			}

			usb_read_size = 0;
		}

		if( usb_endpoint_ready )
		{
			xSemaphoreTake(usb_mutex, portMAX_DELAY);
			if(usb_buffer_begin != usb_buffer_end)
			{
				int size = usb_buffer_end - usb_buffer_begin;
				if(size < 0)
				{
					// on envoi juste la fin du buffer sur ce cycle
					size = USB_BUFER_SIZE - usb_buffer_begin;
				}

				usb_endpoint_ready = 0;
				usb_write_size = size;
				USB_SIL_Write(EP1_IN, usb_buffer + usb_buffer_begin, size);
			}
			xSemaphoreGive(usb_mutex);
		}

		vTaskWaitEvent(EVENT_USB, portMAX_DELAY);
		vTaskClearEvent(EVENT_USB);
	}
}

void Enter_LowPowerMode(void)
{
	// Set the device state to suspend
	bDeviceState = SUSPENDED;
}

void Leave_LowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	// Set the device state to the correct state
	if (pInfo->Current_Configuration != 0)
	{
		// Device configured
		bDeviceState = CONFIGURED;
	}
	else 
	{
		bDeviceState = ATTACHED;
	}
}

void isr_otg_fs(void)
{
	STM32_PCD_OTG_ISR_Handler(); 
}

void EP1_IN_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK();

	usb_endpoint_ready = 1;
	usb_buffer_begin = (usb_buffer_begin + usb_write_size) % USB_BUFER_SIZE;
	xHigherPriorityTaskWoken = vTaskSetEventFromISR(EVENT_USB);

	if( xHigherPriorityTaskWoken )
	{
		vPortYieldFromISR();
	}

	portCLEAR_INTERRUPT_MASK();
}

void EP2_OUT_Callback(void)
{
	// pas de commande en cours de traitement
	if( usb_read_size == 0)
	{
		usb_read_size = USB_SIL_Read(EP2_OUT, usb_rx_buffer);
	}
}
