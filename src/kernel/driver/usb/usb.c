#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"

#include "kernel/driver/usb/stm32f4xx/usbd_atlantronic_core.h"
#include "kernel/driver/usb/stm32f4xx/usbd_usr.h"
#include "kernel/driver/usb/stm32f4xx/usbd_desc.h"
#include "kernel/driver/usb/stm32f4xx/usb_dcd_int.h"
#include "gpio.h"

#include "kernel/driver/usb.h"
#include "kernel/log.h"

// Attention, pour l'envoi de commandes par usb, on suppose que c'est envoyé en une seule trame usb

#define USB_BUFER_SIZE          4096
#define USB_READ_STACK_SIZE      400
#define USB_WRITE_STACK_SIZE      75

// variables statiques => segment bss, initialisation a 0

static unsigned char usb_buffer[USB_BUFER_SIZE];
static int usb_buffer_begin;
static int usb_buffer_end;
static int usb_buffer_size;
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
void usb_cmd_ptask(void*);

static xSemaphoreHandle usb_write_sem;
static xSemaphoreHandle usb_read_sem;
static volatile unsigned int usb_endpoint_ready;
static __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;
void USB_OTG_BSP_mDelay (const uint32_t msec);

static int usb_module_init(void)
{
	usb_endpoint_ready = 1;

	usb_mutex = xSemaphoreCreateMutex();

	if(usb_mutex == NULL)
	{
		return ERR_INIT_USB;
	}

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	gpio_pin_init(GPIOA,  9, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // VBUS
	gpio_pin_init(GPIOA, 10, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_OD, GPIO_PUPD_UP);     // ID
	gpio_pin_init(GPIOA, 11, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DM
	gpio_pin_init(GPIOA, 12, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DP

	gpio_af_config(GPIOA,  9, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 10, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 11, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 12, GPIO_AF_OTG_FS);
	USBD_Init(&USB_OTG_dev, 1, &USR_desc, &USBD_atlantronic_cb, &USR_cb);
	DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer, sizeof(usb_rx_buffer));

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

	portBASE_TYPE err = xTaskCreate(usb_read_task, "usb_r", USB_READ_STACK_SIZE, NULL, PRIORITY_TASK_USB, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_USB;
	}

	err = xTaskCreate(usb_write_task, "usb_w", USB_WRITE_STACK_SIZE, NULL, PRIORITY_TASK_USB, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_USB;
	}

	usb_add_cmd(USB_CMD_PTASK, usb_cmd_ptask);

	return 0;
}

module_init(usb_module_init, INIT_USB);

void usb_write_byte(unsigned char byte)
{
	usb_buffer[usb_buffer_end] = byte;
	usb_buffer_end = (usb_buffer_end + 1) % USB_BUFER_SIZE;
	usb_buffer_size++;

	if( usb_buffer_size > USB_BUFER_SIZE)
	{
		usb_buffer_size = USB_BUFER_SIZE;
		usb_buffer_begin = usb_buffer_end;
	}
}

void usb_write(const void* buffer, int size)
{
	int nMax = USB_BUFER_SIZE - usb_buffer_end;

	usb_buffer_size += size;

	if( likely(size <= nMax) )
	{
		memcpy(&usb_buffer[usb_buffer_end], buffer, size);
		usb_buffer_end = (usb_buffer_end + size) % USB_BUFER_SIZE;
	}
	else
	{
		memcpy(&usb_buffer[usb_buffer_end], buffer, nMax);
		size -= nMax;
		memcpy(&usb_buffer[0], buffer + nMax, size);
		usb_buffer_end = size;
	}

	if( usb_buffer_size > USB_BUFER_SIZE)
	{
		usb_buffer_size = USB_BUFER_SIZE;
		usb_buffer_begin = usb_buffer_end;
	}
}

void usb_add(uint16_t type, void* msg, uint16_t size)
{
	if(size == 0)
	{
		return;
	}

	// on se reserve le buffer circulaire pour les log s'il n'y a personne sur l'usb
	if( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
	{
		return;
	}
	struct usb_header header = {type, size};

	xSemaphoreTake(usb_mutex, portMAX_DELAY);

	usb_write(&header, sizeof(header));
	usb_write(msg, size);

	xSemaphoreGive(usb_mutex);

	xSemaphoreGive(usb_write_sem);
}

void usb_add_log(unsigned char level, const char* func, uint16_t line, const char* msg)
{
	uint16_t msg_size = strlen(msg);
	char log_header[32];

	if(msg_size == 0)
	{
		return;
	}

	struct systime current_time = systick_get_time();
	memcpy(log_header, &current_time, 8);
	log_header[8] = level;
	memcpy(log_header+9, &line, 2);

	int len = strlen(func);

	if(len > 20)
	{
		len = 20;
	}

	memcpy(log_header + 11, func, len);
	len += 11;
	log_header[len] = ':';
	len++;

	uint16_t size = msg_size + len + 1;
	struct usb_header usb_header = {USB_LOG, size};

	xSemaphoreTake(usb_mutex, portMAX_DELAY);

	usb_write(&usb_header, sizeof(usb_header));
	usb_write(log_header, len);
	usb_write(msg, msg_size);
	usb_write_byte((unsigned  char)'\n');

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
		while( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
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
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer, sizeof(usb_rx_buffer));
			*read_size = USBD_GetRxCount(&USB_OTG_dev, 0x02);
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
		while( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		if( usb_endpoint_ready )
		{
			xSemaphoreTake(usb_mutex, portMAX_DELAY);
			if(usb_buffer_size > 0)
			{
				int sizeMax = USB_BUFER_SIZE - usb_buffer_begin;
				if(usb_buffer_size < sizeMax )
				{
					sizeMax = usb_buffer_size;
				}

				usb_endpoint_ready = 0;
				DCD_EP_Tx(&USB_OTG_dev, 0x81, usb_buffer + usb_buffer_begin, sizeMax);
				usb_buffer_size -= sizeMax;
				usb_buffer_begin = (usb_buffer_begin + sizeMax) % USB_BUFER_SIZE;
			}
			xSemaphoreGive(usb_mutex);
		}

		xSemaphoreTake(usb_write_sem, portMAX_DELAY);
	}
}

void isr_otg_fs(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

void EP1_IN_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( ! usb_endpoint_ready )
	{
		usb_endpoint_ready = 1;
		xSemaphoreGiveFromISR(usb_write_sem, &xHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void EP2_OUT_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	// pas de commande en cours de traitement
	if( usb_rx_buffer_id == 0)
	{
		if( usb_read_size == 0)
		{
			usb_read_size = USBD_GetRxCount(&USB_OTG_dev, 0x02);
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer2, sizeof(usb_rx_buffer2));
		}
		else if( usb_read_size2 == 0)
		{
			// perte du message precedent si if( usb_read_size2 == 0)
			usb_read_size2 = USBD_GetRxCount(&USB_OTG_dev, 0x02);
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer, sizeof(usb_rx_buffer));
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
			usb_read_size2 = USBD_GetRxCount(&USB_OTG_dev, 0x02);
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer, sizeof(usb_rx_buffer));
		}
		else if( usb_read_size == 0)
		{
			// perte du message precedent si if( usb_read_size == 0)
			usb_read_size = USBD_GetRxCount(&USB_OTG_dev, 0x02);
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer2, sizeof(usb_rx_buffer2));
		}
		else
		{
			usb_rx_waiting = 1;
		}
	}
	xSemaphoreGiveFromISR(usb_read_sem, &xHigherPriorityTaskWoken);

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void usb_cmd_ptask(void* arg)
{
	(void) arg;
	char usb_ptask_buffer[400];
	vTaskGetRunTimeStats(usb_ptask_buffer, sizeof(usb_ptask_buffer));
	log(LOG_INFO, usb_ptask_buffer);
}
