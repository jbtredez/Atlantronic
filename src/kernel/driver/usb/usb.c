#define WEAK_USB
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "priority.h"

#include "kernel/driver/usb/stm32f4xx/usbd_core.h"
#include "kernel/driver/usb/stm32f4xx/usbd_def.h"
#include "kernel/driver/usb/stm32f4xx/stm32f4xx_hal_pcd.h"
#include "usb_descriptor.h"
#include "kernel/driver/gpio.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

// Attention, pour l'envoi de commandes par usb, on suppose que c'est envoyé en une seule trame usb

#define USB_TX_BUFER_SIZE       4096
#define USB_RX_BUFER_SIZE        512
#define USB_READ_STACK_SIZE      400
#define USB_WRITE_STACK_SIZE      75

#define USBD_MANUFACTURER_STRING      "Atlantronic"
#define USBD_PRODUCT_STRING           ARCH
#define USBD_SERIALNUMBER_STRING      "00000000011C"
#define USBD_CONFIGURATION_STRING      "Atlantronic config"
#define USBD_INTERFACE_STRING          "Atlantronic interface"

// variables statiques => segment bss, initialisation a 0

static unsigned char usb_buffer[USB_TX_BUFER_SIZE];
static int usb_buffer_begin;
static int usb_buffer_end;
static int usb_buffer_size;
static unsigned char usb_rx_buffer_one_msg[256]; //!< buffer usb de reception avec un seul message mis a plat pour le traitement par la tache usb_read (en cas de bouclage sur le buffer circulaire)
static unsigned char usb_rx_buffer_ep[64]; //!< buffer usb de reception d'un endpoint si on n'a pas 64 octets contigus pour le mettre directement dans le buffer circulaire
static unsigned char usb_rx_buffer[USB_RX_BUFER_SIZE]; //!< buffer usb de reception (circulaire)
static unsigned int usb_rx_buffer_head; //!< position ou on doit ajouter les nouveaux octets
static unsigned int usb_rx_buffer_tail; //!< position ou on doit lire les octets
static volatile unsigned int usb_rx_buffer_count; //!< taille du buffer usb de reception
static unsigned int usb_rx_buffer_ep_used;
static int usb_rx_waiting; //!< overflow sur usb - reception. Vaut 1 si on doit relancer la reception
static xSemaphoreHandle usb_mutex;
static void (*usb_cmd[USB_CMD_NUM])(void*);
static const char version[41] = VERSION;
unsigned char usb_get_version_done = 0;
static unsigned char usb_ready = 0;

void usb_read_task(void *);
void usb_write_task(void *);
void usb_cmd_ptask(void*);
void usb_cmd_get_version(void*);
uint8_t* usb_get_device_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_lang_id_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_manufacturer_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_product_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_serial_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_configuration_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t* usb_get_interface_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length);

static xSemaphoreHandle usb_write_sem;
static xSemaphoreHandle usb_read_sem;
static volatile unsigned int usb_endpoint_ready;
static USBD_HandleTypeDef usb_handle;
static USBD_DescriptorsTypeDef usb_descriptors =
{
	usb_get_device_descriptor,
	usb_get_lang_id_str_descriptor,
	usb_get_manufacturer_str_descriptor,
	usb_get_product_str_descriptor,
	usb_get_serial_str_descriptor,
	usb_get_configuration_str_descriptor,
	usb_get_interface_str_descriptor,
};

uint8_t usb_cb_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx);
uint8_t usb_cb_de_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx);
uint8_t usb_cb_Setup(struct _USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req);
//uint8_t usb_cb_EP0_TxSent(struct _USBD_HandleTypeDef *pdev );
//uint8_t usb_cb_EP0_RxReady(struct _USBD_HandleTypeDef *pdev );
/* Class Specific Endpoints*/
uint8_t usb_cb_DataIn(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
uint8_t usb_cb_DataOut(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
//uint8_t usb_cb_SOF(struct _USBD_HandleTypeDef *pdev);
//uint8_t usb_cb_IsoINIncomplete(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);
//uint8_t usb_cb_IsoOUTIncomplete(struct _USBD_HandleTypeDef *pdev , uint8_t epnum);

uint8_t* usb_cb_GetHSConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetFSConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetOtherSpeedConfigDescriptor(uint16_t *length);
uint8_t* usb_cb_GetDeviceQualifierDescriptor(uint16_t *length);

static USBD_ClassTypeDef usb_cb =
{
		usb_cb_init,
		usb_cb_de_init,
		usb_cb_Setup,
		0,//usb_cb_EP0_TxSent,
		0,//usb_cb_EP0_RxReady,
		usb_cb_DataIn,
		usb_cb_DataOut,
		0,//usb_cb_SOF,
		0,//usb_cb_IsoINIncomplete,
		0,//usb_cb_IsoOUTIncomplete,
		usb_cb_GetHSConfigDescriptor,
		usb_cb_GetFSConfigDescriptor,
		usb_cb_GetOtherSpeedConfigDescriptor,
		usb_cb_GetDeviceQualifierDescriptor,
};

static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

static int usb_module_init(void)
{
	usb_endpoint_ready = 1;

	usb_mutex = xSemaphoreCreateMutex();

	if(usb_mutex == NULL)
	{
		return ERR_INIT_USB;
	}
#ifdef STM32F4_USB_HS
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;
	gpio_pin_init(GPIOB, 12, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // ID
	gpio_pin_init(GPIOB, 13, GPIO_MODE_IN, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // VBUS
	gpio_pin_init(GPIOB, 14, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DM
	gpio_pin_init(GPIOB, 15, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DP

	gpio_af_config(GPIOB, 12, GPIO_AF_OTG_HS_FS);
	//gpio_af_config(GPIOB, 13, GPIO_AF_OTG_HS_FS);
	gpio_af_config(GPIOB, 14, GPIO_AF_OTG_HS_FS);
	gpio_af_config(GPIOB, 15, GPIO_AF_OTG_HS_FS);
	USBD_Init(&usb_handle, &usb_descriptors, 0);
	USBD_RegisterClass(&usb_handle, &usb_cb);
	USBD_Start(&usb_handle);
	USBD_LL_PrepareReceive(&usb_handle, 2, usb_rx_buffer, sizeof(usb_rx_buffer));

	NVIC_SetPriority(OTG_HS_IRQn, PRIORITY_IRQ_USB);
	NVIC_EnableIRQ(OTG_HS_IRQn);
#else
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	//gpio_pin_init(GPIOA, 9, GPIO_MODE_IN, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // VBUS
	//gpio_pin_init(GPIOA, 10, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // ID
	gpio_pin_init(GPIOA, 11, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DM
	gpio_pin_init(GPIOA, 12, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL); // DP

	//gpio_af_config(GPIOA, 9, GPIO_AF_OTG_FS);
	//gpio_af_config(GPIOA, 10, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 11, GPIO_AF_OTG_FS);
	gpio_af_config(GPIOA, 12, GPIO_AF_OTG_FS);
	USBD_Init(&usb_handle, &usb_descriptors, 1);
	USBD_RegisterClass(&usb_handle, &usb_cb);
	USBD_Start(&usb_handle);
	USBD_LL_PrepareReceive(&usb_handle, 2, usb_rx_buffer, sizeof(usb_rx_buffer));

	NVIC_SetPriority(OTG_FS_IRQn, PRIORITY_IRQ_USB);
	NVIC_EnableIRQ(OTG_FS_IRQn);
#endif

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

	usb_add_cmd(USB_CMD_GET_VERSION, usb_cmd_get_version);
	usb_add_cmd(USB_CMD_PTASK, usb_cmd_ptask);
	usb_add_cmd(USB_CMD_REBOOT, (void (*)(void*))reboot);

	return 0;
}

module_init(usb_module_init, INIT_USB);

uint8_t* usb_get_device_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	*length = sizeof(usb_device_descriptor);
	return (uint8_t*)usb_device_descriptor;
}

uint8_t* usb_get_lang_id_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	*length =  sizeof(usb_string_langID);
	return (uint8_t*)usb_string_langID;
}

uint8_t* usb_get_manufacturer_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t* usb_get_product_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_PRODUCT_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t* usb_get_serial_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_SERIALNUMBER_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t* usb_get_configuration_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_CONFIGURATION_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t* usb_get_interface_str_descriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
	(void) speed;
	USBD_GetString ((uint8_t*)USBD_INTERFACE_STRING, USBD_StrDesc, length);
	return USBD_StrDesc;
}

uint8_t usb_cb_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx)
{
	(void) cfgidx;
	USBD_LL_OpenEP(pdev, 0x81, USBD_EP_TYPE_BULK, 0x40);
	USBD_LL_OpenEP(pdev, 0x02, USBD_EP_TYPE_BULK, 0x40);
	return USBD_OK;
}

uint8_t usb_cb_de_init(struct _USBD_HandleTypeDef *pdev , uint8_t cfgidx)
{
	(void) cfgidx;
	USBD_LL_CloseEP(pdev, 0x81);
	USBD_LL_CloseEP(pdev, 0x02);
	return USBD_OK;
}

uint8_t usb_cb_Setup(struct _USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req)
{
	(void) pdev;
	(void) req;
	return USBD_OK;
}

uint8_t usb_cb_DataIn(struct _USBD_HandleTypeDef *pdev , uint8_t epnum)
{
	(void) pdev;
	if( epnum == 0x01)
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
	return USBD_OK;
}

uint8_t usb_cb_DataOut(struct _USBD_HandleTypeDef *pdev , uint8_t epnum)
{
	(void) pdev;
	if( epnum == 0x02)
	{
		portBASE_TYPE xHigherPriorityTaskWoken = 0;
		portSET_INTERRUPT_MASK_FROM_ISR();

		// pas de commande en cours de traitement
		int count = USBD_GetRxCount(&usb_handle, 0x02);
		if( unlikely(usb_rx_buffer_ep_used) )
		{
			int nMax = sizeof(usb_rx_buffer) - usb_rx_buffer_head;
			if( count <= nMax )
			{
				memcpy(&usb_rx_buffer[usb_rx_buffer_head], usb_rx_buffer_ep, count);
			}
			else
			{
				memcpy(&usb_rx_buffer[usb_rx_buffer_head], usb_rx_buffer_ep, nMax);
				memcpy(usb_rx_buffer, &usb_rx_buffer_ep[nMax], count - nMax);
			}
		}
		usb_rx_buffer_ep_used = 0;
		usb_rx_buffer_count += count;
		usb_rx_buffer_head = (usb_rx_buffer_head + count) % sizeof(usb_rx_buffer);

		int nMax = sizeof(usb_rx_buffer) - usb_rx_buffer_head;
		count = sizeof(usb_rx_buffer) - usb_rx_buffer_count;
		if( count < nMax )
		{
			nMax = count;
		}

		if( likely(nMax >= 64) )
		{
			USBD_LL_PrepareReceive(&usb_handle, 2, &usb_rx_buffer[usb_rx_buffer_head], nMax);
		}
		else if( count > 64)
		{
			USBD_LL_PrepareReceive(&usb_handle, 2, usb_rx_buffer_ep, nMax);
			usb_rx_buffer_ep_used = 1;
		}
		else
		{
			// overflow
			usb_rx_waiting = 1;
		}

		xSemaphoreGiveFromISR(usb_read_sem, &xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
	}
	return USBD_OK;
}

uint8_t* usb_cb_GetHSConfigDescriptor(uint16_t *length)
{
	*length = sizeof(usb_config_descriptor);
	return (uint8_t*) usb_config_descriptor;
}

uint8_t* usb_cb_GetFSConfigDescriptor(uint16_t *length)
{
	*length = sizeof(usb_config_descriptor);
	return (uint8_t*) usb_config_descriptor;
}

uint8_t* usb_cb_GetOtherSpeedConfigDescriptor(uint16_t *length)
{
	*length = sizeof(usb_config_descriptor);
	return (uint8_t*) usb_config_descriptor;
}

uint8_t* usb_cb_GetDeviceQualifierDescriptor(uint16_t *length)
{
	*length = sizeof(usb_device_qualifier_desc);
	return usb_device_qualifier_desc;
}

void usb_write_byte(unsigned char byte)
{
	usb_buffer[usb_buffer_end] = byte;
	usb_buffer_end = (usb_buffer_end + 1) % USB_TX_BUFER_SIZE;
	usb_buffer_size++;

	if( usb_buffer_size > USB_TX_BUFER_SIZE)
	{
		usb_buffer_size = USB_TX_BUFER_SIZE;
		usb_buffer_begin = usb_buffer_end;
	}
}

void usb_write(const void* buffer, int size)
{
	int nMax = USB_TX_BUFER_SIZE - usb_buffer_end;

	usb_buffer_size += size;

	if( likely(size <= nMax) )
	{
		memcpy(&usb_buffer[usb_buffer_end], buffer, size);
		usb_buffer_end = (usb_buffer_end + size) % USB_TX_BUFER_SIZE;
	}
	else
	{
		memcpy(&usb_buffer[usb_buffer_end], buffer, nMax);
		size -= nMax;
		memcpy(&usb_buffer[0], buffer + nMax, size);
		usb_buffer_end = size;
	}

	if( usb_buffer_size > USB_TX_BUFER_SIZE)
	{
		usb_buffer_size = USB_TX_BUFER_SIZE;
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
	if( (usb_handle.dev_state != USBD_STATE_CONFIGURED || !usb_ready) && type != USB_CMD_GET_VERSION)
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

	while(1)
	{
		while( usb_handle.dev_state != USBD_STATE_CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		if( usb_rx_buffer_count )
		{
			// lecture header
			int id = usb_rx_buffer[usb_rx_buffer_tail];
			int size = usb_rx_buffer[(usb_rx_buffer_tail + 1)%sizeof(usb_rx_buffer)];
			if( size <= (int)usb_rx_buffer_count && size >= 2)
			{
				// on a recu tout le message, on va le traiter
				if( id < USB_CMD_NUM && usb_cmd[id])
				{
					int nMax = sizeof(usb_rx_buffer) - usb_rx_buffer_tail;
					// mise "a plat" dans un seul buffer pour le traitement si necessaire
					if( size <= nMax )
					{
						// message deja contigu en memoire
						usb_cmd[id](&usb_rx_buffer[usb_rx_buffer_tail+2]);
					}
					else
					{
						memcpy(usb_rx_buffer_one_msg, &usb_rx_buffer[usb_rx_buffer_tail], nMax);
						memcpy(&usb_rx_buffer_one_msg[nMax], usb_rx_buffer, size - nMax);
						usb_cmd[id](&usb_rx_buffer_one_msg[2]);
					}
				}
				else
				{
					log_format(LOG_ERROR, "command %d not found", id);
				}

				__sync_sub_and_fetch(&usb_rx_buffer_count, size);
				usb_rx_buffer_tail = (usb_rx_buffer_tail + size) % sizeof(usb_rx_buffer);
			}
			else
			{
				log_format(LOG_ERROR, "usb protocol error : size = %d", size);
				usb_rx_buffer_count = 0;
				usb_rx_buffer_tail = usb_rx_buffer_head;
			}

			if( unlikely(usb_rx_waiting != 0))
			{
				int nMax = sizeof(usb_rx_buffer) - usb_rx_buffer_head;
				int count = sizeof(usb_rx_buffer) - usb_rx_buffer_count;
				if( count < nMax )
				{
					nMax = count;
				}

				if( nMax > 0 )
				{
					// on a eu un overflow, il faut relancer la reception des messages
					USBD_LL_PrepareReceive(&usb_handle, 2, &usb_rx_buffer[usb_rx_buffer_head], nMax);
					usb_rx_waiting = 0;
				}
			}

			if( usb_rx_buffer_count )
			{
				// on a traite un message et il en reste
				// on enchaine sans prendre la semaphore
				continue;
			}
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
		while( usb_handle.dev_state != USBD_STATE_CONFIGURED )
		{
			vTaskDelay( ms_to_tick(100) );
		}

		if( usb_endpoint_ready )
		{
			xSemaphoreTake(usb_mutex, portMAX_DELAY);
			if(usb_buffer_size > 0)
			{
				int sizeMax = USB_TX_BUFER_SIZE - usb_buffer_begin;
				if(usb_buffer_size < sizeMax )
				{
					sizeMax = usb_buffer_size;
				}

				usb_endpoint_ready = 0;
				USBD_LL_Transmit(&usb_handle, 0x81, usb_buffer + usb_buffer_begin, sizeMax);
				usb_buffer_size -= sizeMax;
				usb_buffer_begin = (usb_buffer_begin + sizeMax) % USB_TX_BUFER_SIZE;
			}
			else
			{
				usb_ready = usb_get_version_done;
			}
			xSemaphoreGive(usb_mutex);
		}

		xSemaphoreTake(usb_write_sem, portMAX_DELAY);
	}
}

#ifdef STM32F4_USB_HS
void isr_otg_hs(void)
#else
void isr_otg_fs(void)
#endif
{
	HAL_PCD_IRQHandler(usb_handle.pData);
}

void usb_cmd_ptask(void* arg)
{
	(void) arg;
	char usb_ptask_buffer[400];
	vTaskGetRunTimeStats(usb_ptask_buffer, sizeof(usb_ptask_buffer));
	log(LOG_INFO, usb_ptask_buffer);
}

void usb_cmd_get_version(void* arg)
{
	(void) arg;
	usb_get_version_done = 1;
	usb_add(USB_CMD_GET_VERSION, (void*)version, 41);
}
