#define WEAK_ESP8266
#include "ESP8266.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include <string.h>

#define ESP8266_STACK_SIZE       350
#define ESP8266_TX_BUFER_SIZE       4096

static void esp8266_task(void* arg);
static unsigned char esp8266_tx_buffer_dma[256];
static unsigned char esp8266_rx_buffer_dma[256];
static uint32_t esp8266_configure(uint16_t at_cmd, uint32_t val);
static uint32_t esp8266_send_data_api(const unsigned char* msg, uint16_t size, uint32_t addr_h, uint32_t addr_l);
static uint32_t esp8266_wait_send_data_api();
static void esp8266_cmd(void* arg, void* data);
static ESP8266Status esp8266_init();


ESP8266Status esp8266_status;
static xSemaphoreHandle esp8266_mutex;
static xSemaphoreHandle esp8266_write_sem;
static unsigned char esp8266_buffer[ESP8266_TX_BUFER_SIZE];
static int esp8266_buffer_begin;
static int esp8266_buffer_end;
static int esp8266_buffer_size;

int ESP8266_module_init()
{
	portBASE_TYPE err = xTaskCreate(esp8266_task, "esp8266", ESP8266_STACK_SIZE, NULL, PRIORITY_TASK_ESP8266, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_ESP8266;
	}

	esp8266_mutex = xSemaphoreCreateMutex();

	if(esp8266_mutex == NULL)
	{
		return ERR_INIT_XBEE;
	}

	vSemaphoreCreateBinary(esp8266_write_sem);
	if( esp8266_write_sem == NULL )
	{
		return ERR_INIT_ESP8266;
	}
	xSemaphoreTake(esp8266_write_sem, 0);

	esp8266_status = ESP8266_STATUS_DISCONNECTED;
	usb_add_cmd(USB_CMD_ESP8266, &esp8266_cmd, NULL);

	return 0;
}

module_init(ESP8266_module_init, INIT_ESP8266);

void esp8266_task(void* arg)
{
	(void) arg;

	vTaskDelay(ms_to_tick(1000));



	while(1)
	{
		if( esp8266_status == ESP8266_STATUS_DISCONNECTED)
		{
			esp8266_status = esp8266_init();
			if( esp8266_status == ESP8266_STATUS_DISCONNECTED)
			{
				vTaskDelay(ms_to_tick(500));
				continue;
			}
		}

		xSemaphoreTake(esp8266_mutex, portMAX_DELAY);
		if(esp8266_buffer_size > 0)
		{
			int sizeMax = ESP8266_TX_BUFER_SIZE - esp8266_buffer_begin;
			if(esp8266_buffer_size < sizeMax )
			{
				sizeMax = esp8266_buffer_size;
			}

			// limitation par paquets de 100 (buffer dma de 256 avec entete de message xbee)
			if( sizeMax > 100 )
			{
				sizeMax = 100;
			}
/*
			esp8266_send_data_api(esp8266_buffer + esp8266_buffer_begin, sizeMax);
			esp8266_buffer_size -= sizeMax;
			esp8266_buffer_begin = (esp8266_buffer_begin + sizeMax) % XBEE_TX_BUFER_SIZE;
			*/
		}

		xSemaphoreGive(esp8266_mutex);

		esp8266_wait_send_data_api();
		vTaskDelay(10);

		if( esp8266_buffer_size == 0)
		{
			xSemaphoreTake(esp8266_write_sem, portMAX_DELAY);
		}
	}
}

static ESP8266Status esp8266_init()
{
	uint32_t res = 0;// =// esp8266_configure(XBEE_AT_NETWORK_ID, XBEE_NETWORK_ID);
	if( res )
	{
		return ESP8266_STATUS_DISCONNECTED;
	}

	log(LOG_INFO, "esp8266 configured");
	return ESP8266_STATUS_CONNECTED;
}

static uint32_t esp8266_configure(uint16_t at_cmd, uint32_t val)
{
	uint32_t res = at_cmd;
	val = val+1;
	///COnfigurationdu module ESP8266
	///TODO
	//Modification de l'adresse IP du module
	//Modification du nom du réseau
	//Paramètre du client TCP avec Tentative de connection au simu
	//Lancement du serveur TCP
	///

	return res;
}

// TODO factoriser avec usb.c
void esp8266_write(const void* buffer, int size)
{
	int nMax =ESP8266_TX_BUFER_SIZE - esp8266_buffer_end;

	esp8266_buffer_size += size;

	if( likely(size <= nMax) )
	{
		memcpy(&esp8266_buffer[esp8266_buffer_end], buffer, size);
		esp8266_buffer_end = (esp8266_buffer_end + size) % ESP8266_TX_BUFER_SIZE;
	}
	else
	{
		memcpy(&esp8266_buffer[esp8266_buffer_end], buffer, nMax);
		size -= nMax;
		memcpy(&esp8266_buffer[0], buffer + nMax, size);
		esp8266_buffer_end = size;
	}

	if( esp8266_buffer_size > ESP8266_TX_BUFER_SIZE)
	{
		esp8266_buffer_size = ESP8266_TX_BUFER_SIZE;
		esp8266_buffer_begin =esp8266_buffer_end;
	}
}

void esp8266_write_byte(unsigned char byte)
{
	esp8266_buffer[esp8266_buffer_end] = byte;
	esp8266_buffer_end = (esp8266_buffer_end + 1) % ESP8266_TX_BUFER_SIZE;
	esp8266_buffer_size++;

	if( esp8266_buffer_size > ESP8266_TX_BUFER_SIZE)
	{
		esp8266_buffer_size = ESP8266_TX_BUFER_SIZE;
		esp8266_buffer_begin = esp8266_buffer_end;
	}
}

void esp8266_add(uint16_t type, void* msg, uint16_t size)
{
	if(size == 0)
	{
		return;
	}

	// on se reserve le buffer circulaire pour les log si le xbee n'est pas pret
	if( esp8266_status == ESP8266_STATUS_DISCONNECTED)
	{
		return;
	}

	struct usb_header header = {type, size};

	xSemaphoreTake(esp8266_mutex, portMAX_DELAY);

	esp8266_write(&header, sizeof(header));
	esp8266_write(msg, size);

	xSemaphoreGive(esp8266_mutex);

	xSemaphoreGive(esp8266_write_sem);
}

// TODO mettre en commun les fonctions protocole avec le fichier usb.c
void esp8266_add_log(unsigned char level, const char* func, uint16_t line, const char* msg)
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

	xSemaphoreTake(esp8266_mutex, portMAX_DELAY);

	esp8266_write(&usb_header, sizeof(usb_header));
	esp8266_write(log_header, len);
	esp8266_write(msg, msg_size);
	esp8266_write_byte((unsigned  char)'\n');

	xSemaphoreGive(esp8266_mutex);

	xSemaphoreGive(esp8266_write_sem);
}

static uint32_t esp8266_send_data_api(const unsigned char* msg, uint16_t size, uint32_t addr_h, uint32_t addr_l)
{
	uint16_t api_specific_size = size + 14;

	esp8266_tx_buffer_dma[0] = 0x7e;
	esp8266_tx_buffer_dma[1] = (api_specific_size >> 8) & 0xff;
	esp8266_tx_buffer_dma[2] = api_specific_size & 0xff;
	esp8266_tx_buffer_dma[3] = ESP8266_CMD_WRITE  ;  // type de trame
	esp8266_tx_buffer_dma[4] = 1;              // id de la trame
	esp8266_tx_buffer_dma[5] = addr_h >> 24;   // adresse destination 64 bits
	esp8266_tx_buffer_dma[6] = addr_h >> 16;   // adresse destination 64 bits
	esp8266_tx_buffer_dma[7] = addr_h >> 8;    // adresse destination 64 bits
	esp8266_tx_buffer_dma[8] = addr_h;         // adresse destination 64 bits
	esp8266_tx_buffer_dma[9] = addr_l >> 24;   // adresse destination 64 bits
	esp8266_tx_buffer_dma[10] = addr_l >> 16;  // adresse destination 64 bits
	esp8266_tx_buffer_dma[11] = addr_l >> 8;   // adresse destination 64 bits
	esp8266_tx_buffer_dma[12] = addr_l;        // adresse destination 64 bits
	esp8266_tx_buffer_dma[13] = 0xff;          // adresse destination 16 bit (0xfffe si pas connue)
	esp8266_tx_buffer_dma[14] = 0xfe;          // adresse destination 16 bit (0xfffe si pas connue)
	esp8266_tx_buffer_dma[15] = 0;             // broadcast radius
	esp8266_tx_buffer_dma[16] = 0;             // options
	memcpy(&esp8266_tx_buffer_dma[17], msg, size);

	uint8_t checksum = 0;
	int i;
	for(i = 3; i < size + 17; i++)
	{
		checksum += esp8266_tx_buffer_dma[i];
	}

	esp8266_tx_buffer_dma[size+17] = 0xff - checksum;
//	usart_set_read_dma_size(XBEE_USART, 11);
//	usart_send_dma_buffer(XBEE_USART, size + 18);

	return 0;
}

static uint32_t esp8266_wait_send_data_api()
{
	//REAds
	int ret = 0; //usart_wait_read(XBEE_USART, 500);
	if( ret != 0)
	{
		// pas de log xbee pour ne pas partir en boucle d'erreur...
		//usb_add_log(LOG_ERROR, __FUNCTION__, __LINE__, "usart_wait_read error");
		//log_format(LOG_ERROR, "xbee send data usart error %d", ret);
		return -1;
	}

	/*log_format(LOG_INFO, "tx status %x %x %x %x %x %x %x %x %x %x %x",
			xbee_rx_buffer_dma[0], xbee_rx_buffer_dma[1], xbee_rx_buffer_dma[2], xbee_rx_buffer_dma[3],
			xbee_rx_buffer_dma[4], xbee_rx_buffer_dma[5], xbee_rx_buffer_dma[6], xbee_rx_buffer_dma[7],
			xbee_rx_buffer_dma[8], xbee_rx_buffer_dma[9], xbee_rx_buffer_dma[10]);*/
	return 0;
}

static void esp8266_cmd(void* arg, void* data)
{
	(void) arg;
	(void) data;

/*	struct xbee_cmd_param* cmd_arg = (struct xbee_cmd_param*) data;
	switch(cmd_arg->cmd_id)
	{
		case XBEE_CMD_SET_MANAGER_BAUDRATE:
			//usart_set_frequency(XBEE_USART, (uint32_t)cmd_arg->param);
			break;
		case XBEE_CMD_SET_OP_BAUDRATE:
			//xbee_configure(XBEE_AT_BAUDRATE, XBEE_OP_BAUDRATE);
			//usart_set_frequency(XBEE_USART, XBEE_OP_BAUDRATE);
			//vTaskDelay(ms_to_tick(25));
			//xbee_configure(XBEE_AT_WRITE_EEPROM, 0);
			//xbee_configure(XBEE_AT_SW_RESET, 0);
			break;
		case XBEE_CMD_SET_OP_CONFIGURATION:
			// TODO
			break;
		default:
			log_format(LOG_ERROR, "unknown esp8266 command : %d", cmd_arg->cmd_id);
			break;
	}*/
}
