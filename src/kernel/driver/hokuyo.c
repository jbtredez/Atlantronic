//! @file hokuyo.c
//! @brief Hokuyo module
//! @author Jean-Baptiste Trédez

#include "io/hokuyo.h"
#include "module.h"
#include "io/usart.h"
#include "io/rcc.h"

#define HOKUYO_STACK_SIZE       100


const char* hokuyo_scip2_cmd = "SCIP2.0\n";
const char* hokuyo_speed_cmd = "SS750000\n";
const char* hokuyo_laser_on_cmd = "BM\n";
const char* hokuyo_scan_all = "GS0044072500\n";

static uint8_t hokuyo_read_dma_buffer[1370];

static void hokuyo_task(void *arg);
static uint32_t hokuyo_init();
static uint32_t hokuyo_scip2();
static uint32_t hokuyo_set_speed();
static uint32_t hokuyo_laser_on();
static uint32_t hokuyo_scan();
static uint16_t hokuyo_decode16(const char data1, const char data2);

static int hokuyo_module_init(void)
{
	usart_open(USART3_FULL_DUPLEX, 19200);

	usart_set_read_dma_buffer(USART3_FULL_DUPLEX, hokuyo_read_dma_buffer);

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(hokuyo_task, "hokuyo", HOKUYO_STACK_SIZE, NULL, PRIORITY_TASK_HOKUYO, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_HOKUYO;
	}

	return 0;
}

module_init(hokuyo_module_init, INIT_HOKUYO);

static void hokuyo_task(void *arg)
{
	(void) arg;
	uint32_t err;

	do
	{
		err = hokuyo_init();
		if(err)
		{
			setLed(err);
		}
	}while(err);

	while(1)
	{
	
	}
}

static uint32_t hokuyo_init()
{
	uint32_t err;

	usart_set_frequency(USART3_FULL_DUPLEX, 19200);

	err = hokuyo_scip2();
	if(err == ERR_USART_TIMEOUT)
	{
		// pas de réponse à 19200, le hokuyo est peut être resté configuré
		// => on tente à la vitesse d'utilisation
	
		usart_set_frequency(USART3_FULL_DUPLEX, 750000);
		err = hokuyo_scip2();
	}

	if(err)
	{
		setLed(ERR_HOKUYO_DISCONNECTED);
		goto end;
	}

	err = hokuyo_set_speed();

	if(err)
	{
		goto end;
	}

	usart_set_frequency(USART3_FULL_DUPLEX, 750000);

	err = hokuyo_laser_on();

end:
	return err;
}

//! Vérifie que la commande envoyée est bien renvoyée par le hokuyo
//! @return 0 si c'est bon
//! @return ERR_HOKUYO_CHECK_CMD sinon
static uint32_t hokuyo_check_cmd(unsigned char* cmd, uint32_t size)
{
	uint32_t res = 0;
	uint32_t i = 0;

	for(; i < size; i++)
	{
		if(cmd[i] != hokuyo_read_dma_buffer[i])
		{
			res = ERR_HOKUYO_CHECK_CMD;
			goto end;
		}
	}

end:
	return res;
}

//! Envoi une commande et vérifie si le hokuyo fait bien un echo de la commande
//! @return 0 si ok
static uint32_t hokuyo_write_cmd(unsigned char* buf, uint32_t write_size, uint32_t read_size, portTickType timeout, uint32_t max_try)
{
	uint32_t err = 0;
	uint32_t i = 0;

	usart_set_write_dma_buffer(USART3_FULL_DUPLEX, buf);

	do
	{
		usart_set_read_dma_size(USART3_FULL_DUPLEX, read_size);
		usart_send_dma_buffer(USART3_FULL_DUPLEX, write_size);

		err = usart_wait_read(USART3_FULL_DUPLEX, timeout);
		i++;
	}while(err && i < max_try);

	if(err)
	{
		goto end;
	}

	err = hokuyo_check_cmd(buf, write_size);

end:
	return err;
}

static uint32_t hokuyo_check_sum(uint32_t start, uint32_t end)
{
	uint8_t sum = 0;
	uint32_t err = 0;

	for(; start < end; start++)
	{
		sum += hokuyo_read_dma_buffer[start];
	}
	
	sum &= 0x3F;
	sum += 0x30;
	
	if(sum != hokuyo_read_dma_buffer[end])
	{
		err = ERR_HOKUYO_CHECKSUM;
	}
	
	return err;
}

static uint32_t hokuyo_scip2()
{
	uint32_t err = 0;

	err = hokuyo_write_cmd((unsigned char*) hokuyo_scip2_cmd, 8, 13, ms_to_tick(50), 3);

	if(err)
	{
		goto end;
	}

	err = hokuyo_check_sum(8, 10);

	if(err)
	{
		goto end;
	}

	if( hokuyo_read_dma_buffer[8] != '0')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;	
	}

	if( hokuyo_read_dma_buffer[9] != '0' &&  hokuyo_read_dma_buffer[9] != 'E')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

end:
	return err;
}

static uint32_t hokuyo_set_speed()
{
	uint32_t err = 0;

	err = hokuyo_write_cmd((unsigned char*) hokuyo_speed_cmd, 9, 14, ms_to_tick(50), 3);

	if(err)
	{
		goto end;
	}

	err = hokuyo_check_sum(9, 11);

	if(err)
	{
		goto end;
	}

	if( hokuyo_read_dma_buffer[9] != '0')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

	switch(hokuyo_read_dma_buffer[10])
	{
		case '0':
		case '3':
			// OK
			break;
		case '1':
		case '2':
			err = ERR_HOKUYO_BAUD_RATE;
			goto end;
		default:
			err = ERR_HOKUYO_UNKNOWN_STATUS;
			goto end;
	}

end:
	return err;
}

static uint32_t hokuyo_laser_on()
{
	uint32_t err = 0;

	err = hokuyo_write_cmd((unsigned char*) hokuyo_laser_on_cmd, 3, 8, ms_to_tick(50), 3);

	if(err)
	{
		goto end;
	}

	err = hokuyo_check_sum(3, 5);

	if(err)
	{
		goto end;
	}

	if( hokuyo_read_dma_buffer[3] != 0)
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

	switch(hokuyo_read_dma_buffer[4])
	{
		case '0':
		case '2':
			// OK
			break;
		case '1':
			err = ERR_HOKUYO_LASER_MALFUNCTION;
			goto end;
		default:
			err = ERR_HOKUYO_UNKNOWN_STATUS;
			goto end;
	}

end:
	return err;
}

static uint32_t hokuyo_scan()
{
	uint32_t err = 0;

	err = hokuyo_write_cmd((unsigned char*) hokuyo_scan_all, 13, 1364, ms_to_tick(150), 3);

	if(err)
	{
		goto end;
	}

	err = hokuyo_check_sum(3, 5);

	if(err)
	{
		goto end;
	}
	
	if( hokuyo_read_dma_buffer[3] != 0)
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;	
	}

	switch(hokuyo_read_dma_buffer[4])
	{
		case '0':
		case '2':
			// OK
			break;
		case '1':
			err = ERR_HOKUYO_LASER_MALFUNCTION;
			goto end;
		default:
			err = ERR_HOKUYO_UNKNOWN_STATUS;
			goto end;
	}

end:
	return err;	
}

static uint16_t hokuyo_decode16(const char data1, const char data2)
{
	uint16_t val = data1 - 0x30;
	val <<= 6;
	val &= ~0x3f;
	val |= data2 - 0x30;

	return val;
}