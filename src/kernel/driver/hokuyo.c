//! @file hokuyo.c
//! @brief Hokuyo module
//! @author Atlantronic

#include "kernel/driver/hokuyo.h"
#include "kernel/error.h"
#include "kernel/driver/usart.h"
#include "kernel/rcc.h"
#include "kernel/hokuyo_tools.h"
#include <string.h>

//!< taille de la réponse maxi avec hokuyo_scan_all :
//!< 682 points => 1364 data
//!< 1364 data = 21 * 64 + 20 data
//!< donc 23 octets entête, + 21*(64+2) + (20+2) + 1 = 1432
#define HOKUYO_SCAN_BUFFER_SIZE   1432

const char* hokuyo_scip2_cmd = "SCIP2.0\n";
const char* hokuyo_speed_cmd = "SS750000\n";
const char* hokuyo_laser_on_cmd = "BM\n";
const char* hokuyo_scan_all = "GS0044072500\n";
#define HOKUYO_SPEED        750000

static uint8_t hokuyo_read_dma_buffer[HOKUYO_SCAN_BUFFER_SIZE];

static uint32_t hokuyo_scip2();
static uint32_t hokuyo_set_speed();
static uint32_t hokuyo_laser_on();

uint32_t hokuyo_init()
{
	uint32_t err = 0;

	usart_open(USART3_FULL_DUPLEX, 19200);
	usart_set_read_dma_buffer(USART3_FULL_DUPLEX, hokuyo_read_dma_buffer);

	// on vide tout ce qui traine sur la ligne de reception
	while(err == 0)
	{
		usart_set_read_dma_size(USART3_FULL_DUPLEX, HOKUYO_SCAN_BUFFER_SIZE);
		err = usart_wait_read(USART3_FULL_DUPLEX, ms_to_tick(100));
	}

	err = hokuyo_scip2();
	if(err == ERR_HOKUYO_DISCONNECTED)
	{
		// pas de réponse à 19200, le hokuyo est peut être resté configuré
		// => on tente à la vitesse d'utilisation
		usart_set_frequency(USART3_FULL_DUPLEX, HOKUYO_SPEED);

		err = hokuyo_scip2();
	}

	if(err)
	{
		goto end;
	}

	err = hokuyo_set_speed();

	if(err)
	{
		goto end;
	}

	usart_set_frequency(USART3_FULL_DUPLEX, HOKUYO_SPEED);

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

//! Envoi une commande, attend la reponse du hokuyo, vérifie si le hokuyo fait bien un echo de la commande et le checksum du status
//! @return 0 si ok
static uint32_t hokuyo_transaction(unsigned char* buf, uint32_t write_size, uint32_t read_size, portTickType timeout)
{
	uint32_t err = 0;

	usart_set_write_dma_buffer(USART3_FULL_DUPLEX, buf);

	usart_set_read_dma_size(USART3_FULL_DUPLEX, read_size);
	usart_send_dma_buffer(USART3_FULL_DUPLEX, write_size);
	err = usart_wait_read(USART3_FULL_DUPLEX, timeout);

	if(err)
	{
		switch(err)
		{
			case ERR_USART_TIMEOUT:
				err = ERR_HOKUYO_DISCONNECTED;
				break;
			case ERR_USART_READ_SR_FE:
				err = ERR_HOKUYO_USART_FE;
				break;
			case ERR_USART_READ_SR_NE:
				err = ERR_HOKUYO_USART_NE;
				break;
			case ERR_USART_READ_SR_ORE:
				err = ERR_HOKUYO_USART_ORE;
				break;
			default:
				err = ERR_HOKUYO_DISCONNECTED;
				break;
		}
		goto end;
	}

	err = hokuyo_check_cmd(buf, write_size);

	if(err)
	{
		goto end;
	}

	err = hokuyo_check_sum(write_size, write_size+2);

end:
	return err;
}

static uint32_t hokuyo_scip2()
{
	uint32_t err = 0;

	err = hokuyo_transaction((unsigned char*) hokuyo_scip2_cmd, 8, 13, ms_to_tick(100));

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

	err = hokuyo_transaction((unsigned char*) hokuyo_speed_cmd, 9, 14, ms_to_tick(100));

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

	err = hokuyo_transaction((unsigned char*) hokuyo_laser_on_cmd, 3, 8, ms_to_tick(100));

	if(err)
	{
		goto end;
	}

	if( hokuyo_read_dma_buffer[3] != '0')
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

void hokuyo_start_scan()
{
	usart_set_write_dma_buffer(USART3_FULL_DUPLEX, (unsigned char*)hokuyo_scan_all);
	usart_set_read_dma_size(USART3_FULL_DUPLEX, HOKUYO_SCAN_BUFFER_SIZE);
	usart_send_dma_buffer(USART3_FULL_DUPLEX, 13);
}

uint32_t hokuto_wait_decode_scan(uint16_t* distance, int size)
{
	uint32_t err = usart_wait_read(USART3_FULL_DUPLEX, ms_to_tick(150));

	if(err)
	{
		switch(err)
		{
			case ERR_USART_TIMEOUT:
				err = ERR_HOKUYO_DISCONNECTED;
				break;
			case ERR_USART_READ_SR_FE:
				err = ERR_HOKUYO_USART_FE;
				break;
			case ERR_USART_READ_SR_NE:
				err = ERR_HOKUYO_USART_NE;
				break;
			case ERR_USART_READ_SR_ORE:
				err = ERR_HOKUYO_USART_ORE;
				break;
			default:
				err = ERR_HOKUYO_DISCONNECTED;
				break;
		}
		goto end;
	}

	err = hokuyo_check_cmd((unsigned char*)hokuyo_scan_all, 13);

	if(err)
	{
		goto end;
	}

	err = hokuyo_check_sum(13, 15);

	if(err)
	{
		goto end;
	}
	
	if( hokuyo_read_dma_buffer[13] != '0')
	{
		err = ERR_HOKUYO_UNKNOWN_STATUS;
		goto end;
	}

	switch(hokuyo_read_dma_buffer[14])
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

	err = hokuyo_tools_decode_buffer(hokuyo_read_dma_buffer, HOKUYO_SCAN_BUFFER_SIZE, distance, size);

end:
	return err;
}
