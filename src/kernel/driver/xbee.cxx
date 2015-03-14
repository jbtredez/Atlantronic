#include "xbee.h"
#include "kernel/module.h"
#include "kernel/driver/usart.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include <string.h>

#define XBEE_STACK_SIZE       350
#define XBEE_USART         USART2_FULL_DUPLEX

static void xbee_task(void* arg);
static unsigned char xbee_tx_buffer_dma[256];
static unsigned char xbee_rx_buffer_dma[256];
static uint32_t xbee_configure(uint16_t at_cmd, uint32_t val);
static uint32_t xbee_send_data_api(const char* msg, uint16_t size, uint32_t addr_h, uint32_t addr_l);
static void xbee_cmd(void* arg);
static XbeeStatus xbee_init();

// TEST
void xbee_add_log(unsigned char level, const char* func, uint16_t line, const char* msg);

XbeeStatus xbee_status;

int xbee_module_init()
{
	portBASE_TYPE err = xTaskCreate(xbee_task, "xbee", XBEE_STACK_SIZE, NULL, PRIORITY_TASK_XBEE, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_XBEE;
	}

	xbee_status = XBEE_STATUS_DISCONNECTED;
	usb_add_cmd(USB_CMD_XBEE, &xbee_cmd);

	return 0;
}

module_init(xbee_module_init, INIT_XBEE);

void xbee_task(void* /*arg*/)
{
	usart_open(XBEE_USART, XBEE_OP_BAUDRATE);
	usart_set_write_dma_buffer(XBEE_USART, xbee_tx_buffer_dma);
	usart_set_read_dma_buffer(XBEE_USART, xbee_rx_buffer_dma);

	while(1)
	{
		if( xbee_status == XBEE_STATUS_DISCONNECTED)
		{
			xbee_status = xbee_init();
			if( xbee_status == XBEE_STATUS_DISCONNECTED)
			{
				vTaskDelay(ms_to_tick(100));
				continue;
			}
		}

		// TODO TEST
		xbee_add_log(LOG_INFO, __FUNCTION__, __LINE__, "toto");
		vTaskDelay(ms_to_tick(2000));
	}
}

static XbeeStatus xbee_init()
{
	uint32_t res = xbee_configure(XBEE_AT_NETWORK_ID, XBEE_NETWORK_ID);
	if( res )
	{
		return XBEE_STATUS_DISCONNECTED;
	}

	log(LOG_INFO, "xbee configured");
	return XBEE_STATUS_CONNECTED;
}

static uint32_t xbee_configure(uint16_t at_cmd, uint32_t val)
{
	uint16_t api_specific_size = 8;

	log_format(LOG_INFO, "xbee AT %c%c : %.4x", at_cmd>>8, at_cmd&0xff, (unsigned int)val);

	xbee_tx_buffer_dma[0] = 0x7e;
	xbee_tx_buffer_dma[1] = (api_specific_size >> 8) & 0xff;
	xbee_tx_buffer_dma[2] = api_specific_size & 0xff;
	xbee_tx_buffer_dma[3] = XBEE_CMD_AT;
	xbee_tx_buffer_dma[4] = 0x01; // id
	xbee_tx_buffer_dma[5] = ( at_cmd >> 8 ) & 0xff;
	xbee_tx_buffer_dma[6] = ( at_cmd & 0xff);
	xbee_tx_buffer_dma[7] = (val >> 24) & 0xff;
	xbee_tx_buffer_dma[8] = (val >> 16) & 0xff;
	xbee_tx_buffer_dma[9] = (val >> 8) & 0xff;
	xbee_tx_buffer_dma[10] = val & 0xff;

	// TODO gestion escaped character pour API mode 2 (API mode 1 pour le moment)
	uint8_t checksum = 0;
	for(int i = 3; i < api_specific_size + 3; i++)
	{
		checksum += xbee_tx_buffer_dma[i];
	}

	xbee_tx_buffer_dma[api_specific_size + 3] = 0xff - checksum;

	usart_set_read_dma_size(XBEE_USART, 9);
	usart_send_dma_buffer(XBEE_USART, api_specific_size + 4);

	int error = usart_wait_read(XBEE_USART, XBEE_TIMEOUT);
	if( error )
	{
		log_format(LOG_ERROR, "xbee - usart error %d", error);
		return error;
	}

	if( xbee_rx_buffer_dma[0] != 0x7e || xbee_rx_buffer_dma[1] != 0 ||
		xbee_rx_buffer_dma[2] != 5 || xbee_rx_buffer_dma[3] != 0x88 ||
		xbee_rx_buffer_dma[4] != xbee_tx_buffer_dma[4] ||
		xbee_rx_buffer_dma[5] != xbee_tx_buffer_dma[5] ||
		xbee_rx_buffer_dma[6] != xbee_tx_buffer_dma[6] )
	{
		// erreur de protocole
		log(LOG_ERROR, "xbee protocol error");
		return -1;

	}

	int res = -1;
	// status de la commande
	switch( xbee_rx_buffer_dma[7] )
	{
		case 0:
			// OK
			res = 0;
			break;
		default:
		case 1:
			log(LOG_ERROR, "xbee at error");
			res = 0;// TODO
			break;
		case 2:
			log(LOG_ERROR, "xbee invalid at command");
			break;
		case 3:
			log(LOG_ERROR, "xbee invalid at parameters");
			break;
	}

	return res;
}

// TODO TEST
void xbee_add_log(unsigned char level, const char* func, uint16_t line, const char* msg)
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

	static char buffer[2048];
	memcpy(buffer, &usb_header, sizeof(usb_header));
	memcpy(buffer + sizeof(usb_header), log_header, len);
	memcpy(buffer + sizeof(usb_header) + len, msg, msg_size);
	buffer[sizeof(usb_header) + len + msg_size] = '\n';

	xbee_send_data_api(buffer, sizeof(usb_header) + len + msg_size + 1, XBEE_ADDR_PC_H, XBEE_ADDR_PC_L);
}

static uint32_t xbee_send_data_api(const char* msg, uint16_t size, uint32_t addr_h, uint32_t addr_l)
{
	uint16_t api_specific_size = size + 14;

	xbee_tx_buffer_dma[0] = 0x7e;
	xbee_tx_buffer_dma[1] = (api_specific_size >> 8) & 0xff;
	xbee_tx_buffer_dma[2] = api_specific_size & 0xff;
	xbee_tx_buffer_dma[3] = XBEE_CMD_TX;    // type de trame
	xbee_tx_buffer_dma[4] = 1;              // id de la trame
	xbee_tx_buffer_dma[5] = addr_h >> 24;   // adresse destination 64 bits
	xbee_tx_buffer_dma[6] = addr_h >> 16;   // adresse destination 64 bits
	xbee_tx_buffer_dma[7] = addr_h >> 8;    // adresse destination 64 bits
	xbee_tx_buffer_dma[8] = addr_h;         // adresse destination 64 bits
	xbee_tx_buffer_dma[9] = addr_l >> 24;   // adresse destination 64 bits
	xbee_tx_buffer_dma[10] = addr_l >> 16;  // adresse destination 64 bits
	xbee_tx_buffer_dma[11] = addr_l >> 8;   // adresse destination 64 bits
	xbee_tx_buffer_dma[12] = addr_l;        // adresse destination 64 bits
	xbee_tx_buffer_dma[13] = 0xff;          // adresse destination 16 bit (0xfffe si pas connue)
	xbee_tx_buffer_dma[14] = 0xfe;          // adresse destination 16 bit (0xfffe si pas connue)
	xbee_tx_buffer_dma[15] = 0;             // broadcast radius
	xbee_tx_buffer_dma[16] = 0;             // options
	memcpy(&xbee_tx_buffer_dma[17], msg, size);

	uint8_t checksum = 0;
	for(int i = 3; i < size + 17; i++)
	{
		checksum += xbee_tx_buffer_dma[i];
	}

	xbee_tx_buffer_dma[size+17] = 0xff - checksum;
	usart_set_read_dma_size(XBEE_USART, 11);
	usart_send_dma_buffer(XBEE_USART, size + 18);

	int ret = usart_wait_read(XBEE_USART, 500);
	if( ret != 0)
	{
		log_format(LOG_ERROR, "xbee send data usart error %d", ret);
		return -1;
	}

	log_format(LOG_INFO, "tx status %x %x %x %x %x %x %x %x %x %x %x",
			xbee_rx_buffer_dma[0], xbee_rx_buffer_dma[1], xbee_rx_buffer_dma[2], xbee_rx_buffer_dma[3],
			xbee_rx_buffer_dma[4], xbee_rx_buffer_dma[5], xbee_rx_buffer_dma[6], xbee_rx_buffer_dma[7],
			xbee_rx_buffer_dma[8], xbee_rx_buffer_dma[9], xbee_rx_buffer_dma[10]);

	return 0;
}

static void xbee_cmd(void* arg)
{
	struct xbee_cmd_param* cmd_arg = (struct xbee_cmd_param*) arg;
	switch(cmd_arg->cmd_id)
	{
		case XBEE_CMD_SET_MANAGER_BAUDRATE:
			usart_set_frequency(XBEE_USART, (uint32_t)cmd_arg->param);
			break;
		case XBEE_CMD_SET_OP_BAUDRATE:
			xbee_configure(XBEE_AT_BAUDRATE, XBEE_OP_BAUDRATE);
			usart_set_frequency(XBEE_USART, XBEE_OP_BAUDRATE);
			vTaskDelay(ms_to_tick(25));
			xbee_configure(XBEE_AT_WRITE_EEPROM, 0);
			xbee_configure(XBEE_AT_SW_RESET, 0);
			break;
		case XBEE_CMD_SET_OP_CONFIGURATION:
			// TODO
			break;
		default:
			log_format(LOG_ERROR, "unknown xbee command : %d", cmd_arg->cmd_id);
			break;
	}
}
