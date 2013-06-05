#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"
#include "kernel/driver/usb/usb_lib.h"
#include "kernel/driver/usb/usb_istr.h"
#include "kernel/driver/usb/usb_pwr.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

// Attention, pour l'envoi de commandes par usb, on suppose que c'est envoyé en une seule trame usb

#define USB_BUFER_SIZE          4096
#define USB_READ_STACK_SIZE      350
#define USB_WRITE_STACK_SIZE      50

// variables statiques => segment bss, initialisation a 0

static unsigned char usb_buffer[USB_BUFER_SIZE];
static int usb_buffer_begin;
static int usb_buffer_end;
static unsigned int usb_write_size;
static unsigned char usb_rx_buffer[64]; //!< buffer usb de reception
static unsigned char usb_rx_buffer2[64]; //!< buffer usb de reception (second)
static unsigned int usb_read_size; //!< taille du buffer usb de reception
static unsigned int usb_read_size2; //!< taille du buffer usb de reception (second)
static unsigned int usb_rx_buffer_id; //!< id courant du buffer usb de reception (à traiter par la tache)
static volatile int usb_rx_waiting; //!< les 2 buffers de reception sont pleins, on fait attendre le pc pour ne pas perdre de messages
static xSemaphoreHandle usb_mutex;
static void (*usb_cmd[USB_CMD_NUM])(void*);

void usb_read_task(void *);
void usb_write_task(void *);


static xSemaphoreHandle usb_write_sem;
static xSemaphoreHandle usb_read_sem;
static volatile unsigned int usb_endpoint_ready;

static int usb_module_init(void)
{
	usb_endpoint_ready = 1;

	usb_mutex = xSemaphoreCreateMutex();

	if(usb_mutex == NULL)
	{
		return ERR_INIT_USB;
	}

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->AHBENR |= RCC_AHBENR_OTGFSEN; // USB OTG FS clock enable

	USB_Init();

	NVIC_SetPriority(OTG_FS_IRQn, PRIORITY_IRQ_USB);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	vSemaphoreCreateBinary(usb_write_sem);
	if( usb_write_sem == NULL )
	{
		return ERR_INIT_USB;
	}
	xSemaphoreTake(usb_write_sem, 0);

	vSemaphoreCreateBinary(usb_read_sem);
	if( usb_read_sem == NULL )
	{
		return ERR_INIT_USB;
	}
	xSemaphoreTake(usb_read_sem, 0);

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(usb_read_task, "usb_r", USB_READ_STACK_SIZE, NULL, PRIORITY_TASK_USB, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_USB;
	}

	err = xTaskCreate(usb_write_task, "usb_w", USB_WRITE_STACK_SIZE, NULL, PRIORITY_TASK_USB, &xHandle);

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

	// on se reserve le buffer circulaire pour les log s'il n'y a personne sur l'usb
	if( bDeviceState != CONFIGURED && type != USB_LOG )
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
	xSemaphoreGive(usb_mutex);

	xSemaphoreGive(usb_write_sem);
}

void usb_add_cmd(enum usb_cmd id, void (*cmd)(void*))
{
	usb_cmd[id] = cmd;
}

//! Usb read task
void usb_read_task(void * arg)
{
	(void) arg;
	unsigned char* rx_buffer;
	unsigned int* read_size;

	while(1)
	{
		while( bDeviceState != CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		// choix du bon buffer
		if( usb_rx_buffer_id == 0)
		{
			rx_buffer = usb_rx_buffer;
			read_size = &usb_read_size;
		}
		else
		{
			rx_buffer = usb_rx_buffer2;
			read_size = &usb_read_size2;
		}

		if( *read_size )
		{
			int id = rx_buffer[0];
			if( id < USB_CMD_NUM )
			{
				if( usb_cmd[id] )
				{
					usb_cmd[id](rx_buffer+1);
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

			*read_size = 0;
			usb_rx_buffer_id = (usb_rx_buffer_id + 1) & 0x01;
		}

		// on a fait attendre un 3ieme message de la part du pc.
		// donc le second buffer est déjà plein et on va depiler directement le message
		// dans le buffer courant qu'on vient de traiter
		if(usb_rx_waiting)
		{
			usb_rx_waiting = 0;
			*read_size = USB_SIL_Read(EP2_OUT, rx_buffer);
		}

		xSemaphoreTake(usb_read_sem, portMAX_DELAY);
	}
}

//! Usb write task
void usb_write_task(void * arg)
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

		xSemaphoreTake(usb_write_sem, portMAX_DELAY);
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

	if( ! usb_endpoint_ready )
	{
		usb_endpoint_ready = 1;
		usb_buffer_begin = (usb_buffer_begin + usb_write_size) % USB_BUFER_SIZE;
		xSemaphoreGiveFromISR(usb_write_sem, &xHigherPriorityTaskWoken);
	}

	if( xHigherPriorityTaskWoken )
	{
		vPortYieldFromISR();
	}

	portCLEAR_INTERRUPT_MASK();
}

void EP2_OUT_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK();

	// pas de commande en cours de traitement
	if( usb_rx_buffer_id == 0)
	{
		if( usb_read_size == 0)
		{
			usb_read_size = USB_SIL_Read(EP2_OUT, usb_rx_buffer);
		}
		else if( usb_read_size2 == 0)
		{
			// perte du message precedent si if( usb_read_size2 == 0)
			usb_read_size2 = USB_SIL_Read(EP2_OUT, usb_rx_buffer2);
		}
		else
		{
			usb_rx_waiting = 1;
		}
	}
	else
	{
		if( usb_read_size2 == 0)
		{
			usb_read_size2 = USB_SIL_Read(EP2_OUT, usb_rx_buffer2);
		}
		else if( usb_read_size == 0)
		{
			// perte du message precedent si if( usb_read_size == 0)
			usb_read_size = USB_SIL_Read(EP2_OUT, usb_rx_buffer);
		}
		else
		{
			usb_rx_waiting = 1;
		}
	}
	xSemaphoreGiveFromISR(usb_read_sem, &xHigherPriorityTaskWoken);

	if( xHigherPriorityTaskWoken )
	{
		vPortYieldFromISR();
	}

	portCLEAR_INTERRUPT_MASK();
}
