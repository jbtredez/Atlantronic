#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "priority.h"
#include "kernel/event.h"
#include "kernel/driver/usb/usb_lib.h"
#include "kernel/driver/usb/usb_istr.h"
#include "kernel/driver/usb/usb_pwr.h"
#include "kernel/driver/usb.h"

#define USB_BUFER_SIZE          4096

//! @todo réglage au pif
#define USB_STACK_SIZE            64

static unsigned char usb_buffer[USB_BUFER_SIZE];
static int usb_buffer_begin;
static int usb_buffer_end;
static unsigned int usb_write_size;

void usb_task(void *);
static volatile unsigned int usb_endpoint_ready;

static int usb_module_init(void)
{
	usb_buffer_begin = 0;
	usb_buffer_end = 0;
	usb_write_size = 0;
	usb_endpoint_ready = 1;

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->AHBENR |= RCC_AHBENR_OTGFSEN; // USB OTG FS clock enable

	NVIC_SetPriority(OTG_FS_IRQn, PRIORITY_IRQ_USB);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	USB_Init();

//	log_info("Création de la tache usb");

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(usb_task, "usb", USB_STACK_SIZE, NULL, PRIORITY_TASK_USB, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_LOG;
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

	// on se reserve le buffer circulaire que pour les log s'il n'y a personne sur l'usb
	if( type != USB_LOG && bDeviceState != CONFIGURED )
	{
		return;
	}

	portENTER_CRITICAL();

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
	portEXIT_CRITICAL();
}

//! Log task
void usb_task(void * arg)
{
	(void) arg;

	while(1)
	{
		while( bDeviceState != CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		if( usb_endpoint_ready )
		{
			portENTER_CRITICAL();
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
			portEXIT_CRITICAL();
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
	portSET_INTERRUPT_MASK();

	usb_endpoint_ready = 1;
	usb_buffer_begin = (usb_buffer_begin + usb_write_size) % USB_BUFER_SIZE;
	vTaskSetEventFromISR(EVENT_USB);

	portCLEAR_INTERRUPT_MASK();
}
