#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"

#ifdef STM32F10X_CL
#include "kernel/driver/usb/stm32f1xx/usb_lib.h"
#include "kernel/driver/usb/stm32f1xx/usb_istr.h"
#include "kernel/driver/usb/stm32f1xx/usb_pwr.h"
#endif

#ifdef STM32F4XX
#include "kernel/driver/usb/stm32f4xx/usbd_atlantronic_core.h"
#include "kernel/driver/usb/stm32f4xx/usbd_usr.h"
#include "kernel/driver/usb/stm32f4xx/usbd_desc.h"
#include "kernel/driver/usb/stm32f4xx/usb_dcd_int.h"
#include "gpio.h"
#endif
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
#ifdef STM32F4XX
static __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;
void USB_OTG_BSP_mDelay (const uint32_t msec);
#endif

static int usb_module_init(void)
{
	usb_endpoint_ready = 1;

	usb_mutex = xSemaphoreCreateMutex();

	if(usb_mutex == NULL)
	{
		return ERR_INIT_USB;
	}

#if defined( STM32F10X_CL )
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->AHBENR |= RCC_AHBENR_OTGFSEN; // USB OTG FS clock enable

	USB_Init();
#elif defined( STM32F4XX )
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
#endif
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
#if defined( STM32F10X_CL )
	// on se reserve le buffer circulaire pour les log s'il n'y a personne sur l'usb
	if( bDeviceState != CONFIGURED && type != USB_LOG )
#elif defined( STM32F4XX )
	if( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED && type != USB_LOG )
#endif
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
#if defined( STM32F10X_CL )
		while( bDeviceState != CONFIGURED )
#elif defined( STM32F4XX )
		while( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
#endif
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
#if defined( STM32F10X_CL )
			*read_size = USB_SIL_Read(EP2_OUT, rx_buffer);
#elif defined( STM32F4XX )
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer, sizeof(usb_rx_buffer));
			*read_size = USBD_GetRxCount(&USB_OTG_dev, 0x02);
#endif
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
#if defined( STM32F10X_CL )
		while( bDeviceState != CONFIGURED )
#elif defined( STM32F4XX )
		while( USB_OTG_dev.dev.device_status != USB_OTG_CONFIGURED )
#endif
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
#if defined( STM32F10X_CL )
				USB_SIL_Write(EP1_IN, usb_buffer + usb_buffer_begin, size);
#elif defined( STM32F4XX )
				DCD_EP_Tx(&USB_OTG_dev, 0x81, usb_buffer + usb_buffer_begin, size);
#endif
			}
			xSemaphoreGive(usb_mutex);
		}

		xSemaphoreTake(usb_write_sem, portMAX_DELAY);
	}
}

void isr_otg_fs(void)
{
#if defined( STM32F10X_CL )
	STM32_PCD_OTG_ISR_Handler(); 
#elif defined( STM32F4XX )
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
#endif
}

void EP1_IN_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( ! usb_endpoint_ready )
	{
		usb_endpoint_ready = 1;
		usb_buffer_begin = (usb_buffer_begin + usb_write_size) % USB_BUFER_SIZE;
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
#if defined( STM32F10X_CL )
			usb_read_size = USB_SIL_Read(EP2_OUT, usb_rx_buffer);
#elif defined( STM32F4XX )
			usb_read_size = USBD_GetRxCount(&USB_OTG_dev, 0x02);
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer2, sizeof(usb_rx_buffer2));
#endif
		}
		else if( usb_read_size2 == 0)
		{
			// perte du message precedent si if( usb_read_size2 == 0)
#if defined( STM32F10X_CL )
			usb_read_size2 = USB_SIL_Read(EP2_OUT, usb_rx_buffer2);
#elif defined( STM32F4XX )
			usb_read_size2 = USBD_GetRxCount(&USB_OTG_dev, 0x02);
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer, sizeof(usb_rx_buffer));
#endif
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
#if defined( STM32F10X_CL )
			usb_read_size2 = USB_SIL_Read(EP2_OUT, usb_rx_buffer2);
#elif defined( STM32F4XX )
			usb_read_size2 = USBD_GetRxCount(&USB_OTG_dev, 0x02);
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer, sizeof(usb_rx_buffer));
#endif
		}
		else if( usb_read_size == 0)
		{
#if defined( STM32F10X_CL )
			// perte du message precedent si if( usb_read_size == 0)
			usb_read_size = USB_SIL_Read(EP2_OUT, usb_rx_buffer);
#elif defined( STM32F4XX )
			usb_read_size = USBD_GetRxCount(&USB_OTG_dev, 0x02);
			DCD_EP_PrepareRx(&USB_OTG_dev, 2, usb_rx_buffer2, sizeof(usb_rx_buffer2));
#endif
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
