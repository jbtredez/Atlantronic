#define WEAK_ESP8266
#include "ESP8266.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/spi.h"
#include "kernel/log.h"
#include <string.h>

#define ESP8266_STACK_SIZE       		350
#define ESP8266_TX_BUFER_SIZE       	4096
#define ESP8266_SPI_DATA_SIZE       	32  //32 octets d'un Packet vers le SPI
#define ESP8266_SPI_TRANSACTION_SIZE    2 + ESP8266_SPI_DATA_SIZE  //2 octets de cmd (cmd+adresse) et 32 octets de msg
#define ESP8266_USB_HEADER_SIZE			4

static void esp8266_task(void* arg);
static unsigned char esp8266_tx_buffer_dma[ESP8266_SPI_TRANSACTION_SIZE];
static unsigned char esp8266_rx_buffer_dma[ESP8266_SPI_TRANSACTION_SIZE];
static uint32_t esp8266_configure(uint16_t at_cmd, uint32_t val);
static uint32_t esp8266_send_data_api(const unsigned char* msg, uint16_t size);

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
		return ERR_INIT_ESP8266;
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

			// limitation par paquets de 32 octets
			if( sizeMax > ESP8266_SPI_DATA_SIZE )
			{
				sizeMax = ESP8266_SPI_DATA_SIZE;
			}
			esp8266_send_data_api(esp8266_buffer + esp8266_buffer_begin, sizeMax);
			esp8266_buffer_size -= sizeMax;
			esp8266_buffer_begin = (esp8266_buffer_begin + sizeMax) % ESP8266_TX_BUFER_SIZE;

		}

		xSemaphoreGive(esp8266_mutex);

		vTaskDelay(20);

		if( esp8266_buffer_size == 0)
		{
			xSemaphoreTake(esp8266_write_sem, portMAX_DELAY);
		}
	}
}

static ESP8266Status esp8266_init()
{

	esp8266_configure(0,0);
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
	uint8_t Esp_msg[ESP8266_MSG_SIZE_MAX + 2 ] ;
	if(size == 0)
	{
		return;
	}
	if(size > ESP8266_MSG_SIZE_MAX)
	{
		size = ESP8266_MSG_SIZE_MAX;
	}
	// on se reserve le buffer circulaire pour les log si le xbee n'est pas pret
	//if( esp8266_status == ESP8266_STATUS_DISCONNECTED)
	//{
	//	return;
	//}

	//struct usb_header header = {type, size};

	///Header de traitement du logiciel de L'ESP
	Esp_msg[0] = ESP8266_CMD_DATA;
	Esp_msg[1] = size;
	memcpy(Esp_msg +2 , msg, size);
	xSemaphoreTake(esp8266_mutex, portMAX_DELAY);

	esp8266_write(Esp_msg, size );

	xSemaphoreGive(esp8266_mutex);

	xSemaphoreGive(esp8266_write_sem);
}

// TODO mettre en commun les fonctions protocole avec le fichier usb.c
void esp8266_add_log(unsigned char level, const char* func, uint16_t line, const char* msg)
{
	uint8_t Esp_msg[ESP8266_MSG_SIZE_MAX +2 ] ;
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

	uint8_t Espmsg_size =  2;
	memcpy(Esp_msg + Espmsg_size , &usb_header,ESP8266_USB_HEADER_SIZE);

	Espmsg_size += ESP8266_USB_HEADER_SIZE;
	memcpy(Esp_msg + Espmsg_size , log_header, len);
	if((Espmsg_size + len) > 256)
	{
			len = Espmsg_size - 256;
	}
	Espmsg_size += len;
	memcpy(Esp_msg + Espmsg_size, msg, msg_size);

	Espmsg_size += msg_size;

	Esp_msg[ Espmsg_size ] = '\n';
	Espmsg_size++;

	///Header de traitement du logiciel de L'ESP
	Esp_msg[0] = ESP8266_CMD_DATA;
	Esp_msg[1] = Espmsg_size - 2;

	xSemaphoreTake(esp8266_mutex, portMAX_DELAY);

	esp8266_write(Esp_msg,Espmsg_size);


	xSemaphoreGive(esp8266_mutex);

	xSemaphoreGive(esp8266_write_sem);
}

static uint32_t esp8266_send_data_api(const unsigned char* msg, uint16_t size)
{

	memset(esp8266_tx_buffer_dma,0,ESP8266_SPI_TRANSACTION_SIZE);
	memset(esp8266_rx_buffer_dma,0,ESP8266_SPI_TRANSACTION_SIZE);
	esp8266_tx_buffer_dma[0] = ESP8266_CMD_SPI_WRITE; //Adresse CMD
	esp8266_tx_buffer_dma[1] = ESP8266_CMD_SPI_ADDR_DATA;
	memcpy(&esp8266_tx_buffer_dma[2], msg, size); //32  octets de données
	spi_transaction(SPI_DEVICE_ESP8266, esp8266_tx_buffer_dma, esp8266_rx_buffer_dma, ESP8266_SPI_TRANSACTION_SIZE);

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
