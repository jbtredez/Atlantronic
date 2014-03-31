#include "xbee.h"
#include "kernel/module.h"
#include "kernel/driver/usart.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include <string.h>

#define XBEE_STACK_SIZE       350
#define XBEE_CMD_TX_64                     0x00      //!< envoi, adresse 64 bit
#define XBEE_CMD_TX_16                     0x01      //!< envoi, adresse 16 bit
#define XBEE_CMD_AT                        0x08
#define XBEE_CMD_AT_QUEUE_PARAM_VALUE      0x09
#define XBEE_CMD_REMOTE_AT                 0x17
#define XBEE_CMD_RX_MASK                   0x80
#define XBEE_CMD_MODEM_STATUS              0x8a

#define XBEE_AT_BAUDRATE                   (('B' << 8) + 'D')
#define XBEE_AT_WRITE_EEPROM               (('W' << 8) + 'R')
#define XBEE_AT_SW_RESET                   (('F' << 8) + 'R')
#define XBEE_AT_NETWORK_ID                 (('I' << 8) + 'D')
#define XBEE_AT_ADDR                       (('M' << 8) + 'Y')
#define XBEE_AT_DEST_L                     (('D' << 8) + 'L')
#define XBEE_AT_DEST_H                     (('D' << 8) + 'H')

#define XBEE_OP_BAUDRATE               250000
#define XBEE_TIMEOUT                       50
#define XBEE_NETWORK_ID                0x1818
#define XBEE_ADDR_DISCOVERY            0x1111
#define XBEE_ADDR_PC                   0x2222


enum XbeeStatus
{
	XBEE_STATUS_DISCONNECTED = 0,
	XBEE_STATUS_CONNECTED,
};

static void xbee_task(void* arg);
static unsigned char xbee_tx_buffer_dma[256];
static unsigned char xbee_rx_buffer_dma[256];
static uint32_t xbee_send_api(uint8_t cmd, const char* buffer, uint16_t size, uint16_t read_size, portTickType timeout);
static uint32_t xbee_configure(uint16_t at_cmd, uint32_t val);
//static uint32_t xbee_read_param(uint16_t at_cmd, uint32_t* val);
static void xbee_cmd(void* arg);
static XbeeStatus xbee_init();

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
	usart_open(USART3_FULL_DUPLEX, XBEE_OP_BAUDRATE);
	usart_set_write_dma_buffer(USART3_FULL_DUPLEX, xbee_tx_buffer_dma);
	usart_set_read_dma_buffer(USART3_FULL_DUPLEX, xbee_rx_buffer_dma);

	while(1)
	{
		if( xbee_status == XBEE_STATUS_DISCONNECTED)
		{
			xbee_status = xbee_init();
		}

		vTaskDelay(ms_to_tick(50));
	}
}

static XbeeStatus xbee_init()
{
	uint32_t res = xbee_configure(XBEE_AT_NETWORK_ID, XBEE_NETWORK_ID);
	if( res )
	{
		return XBEE_STATUS_DISCONNECTED;
	}

	res = xbee_configure(XBEE_AT_ADDR, XBEE_ADDR_DISCOVERY);
	if( res )
	{
		return XBEE_STATUS_DISCONNECTED;
	}

	res = xbee_configure(XBEE_AT_DEST_L, XBEE_ADDR_PC);
	if( res )
	{
		return XBEE_STATUS_DISCONNECTED;
	}
	// TODO voir si on passe sur d es adresse 64 bit ?
	res = xbee_configure(XBEE_AT_DEST_H, 0x00);
	if( res )
	{
		return XBEE_STATUS_DISCONNECTED;
	}

	log(LOG_INFO, "xbee configured");
	return XBEE_STATUS_CONNECTED;
}

static uint32_t xbee_send_api(uint8_t cmd, const char* buffer, uint16_t size, uint16_t read_size, portTickType timeout)
{
	uint16_t api_specific_size = size + 1;

	xbee_tx_buffer_dma[0] = 0x7e;
	xbee_tx_buffer_dma[1] = (api_specific_size >> 8) & 0xff;
	xbee_tx_buffer_dma[2] = api_specific_size & 0xff;
	xbee_tx_buffer_dma[3] = cmd;

	// TODO gestion escaped character pour API mode 2 (API mode 1 pour le moment)
	uint8_t checksum = cmd;
	for(int i = 0; i < size; i++)
	{
		xbee_tx_buffer_dma[4+i] = buffer[i];
		checksum += buffer[i];
	}

	xbee_tx_buffer_dma[size + 4] = 0xff - checksum;

	usart_set_read_dma_size(USART3_FULL_DUPLEX, read_size);
	usart_send_dma_buffer(USART3_FULL_DUPLEX, size + 5);

	return usart_wait_read(USART3_FULL_DUPLEX, timeout);
}

static uint32_t xbee_configure(uint16_t at_cmd, uint32_t val)
{
	char buffer[7];
	buffer[0] = 0x01; // id
	buffer[1] = ( at_cmd >> 8 ) & 0xff;
	buffer[2] = ( at_cmd & 0xff);
	buffer[3] = (val >> 24) & 0xff;
	buffer[4] = (val >> 16) & 0xff;
	buffer[5] = (val >> 8) & 0xff;
	buffer[6] = val & 0xff;

	return xbee_send_api(XBEE_CMD_AT, buffer, sizeof(buffer), 9, XBEE_TIMEOUT);
}
/*
static uint32_t xbee_read_param(uint16_t at_cmd, uint32_t* val)
{
	char buffer[3];
	buffer[0] = 0x01; // id
	buffer[1] = ( at_cmd >> 8 ) & 0xff;
	buffer[2] = ( at_cmd & 0xff);

	uint32_t res = xbee_send_api(XBEE_CMD_AT, buffer, sizeof(buffer), 13, XBEE_TIMEOUT);

	// TODO gestion erreur si timeout ou pb protocol
	if( ! res )
	{
		*val = (xbee_rx_buffer_dma[8] << 24) + (xbee_rx_buffer_dma[9] << 16) + (xbee_rx_buffer_dma[10] << 8) + xbee_rx_buffer_dma[11];
	}
	return res;
}
*/
static void xbee_cmd(void* arg)
{
	struct xbee_cmd_param* cmd_arg = (struct xbee_cmd_param*) arg;
	switch(cmd_arg->cmd_id)
	{
		case XBEE_CMD_SET_MANAGER_BAUDRATE:
			usart_set_frequency(USART3_FULL_DUPLEX, (uint32_t)cmd_arg->param);
			break;
		case XBEE_CMD_SET_OP_BAUDRATE:
			xbee_configure(XBEE_AT_BAUDRATE, XBEE_OP_BAUDRATE);
			usart_set_frequency(USART3_FULL_DUPLEX, XBEE_OP_BAUDRATE);
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
