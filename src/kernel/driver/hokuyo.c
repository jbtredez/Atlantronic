//! @file hokuyo.c
//! @brief Hokuyo module
//! @author Atlantronic

#include "kernel/driver/hokuyo.h"
#include "kernel/error.h"
#include "kernel/driver/usart.h"
#include "kernel/rcc.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/module.h"
#include "kernel/event.h"
#include <string.h>

#define ERR_HOKUYO_TIMEOUT              0x01
#define ERR_HOKUYO_USART_FE             0x02
#define ERR_HOKUYO_USART_NE             0x04
#define ERR_HOKUYO_USART_ORE            0x08
#define ERR_HOKUYO_CHECK_CMD            0x10

#define ERR_HOKUYO_UNKNOWN_STATUS       0x20
#define ERR_HOKUYO_CHECKSUM             0x40
#define ERR_HOKUYO_BAUD_RATE            0x80
#define ERR_HOKUYO_LASER_MALFUNCTION   0x100

//!< taille de la réponse maxi avec hokuyo_scan_all :
//!< 682 points => 1364 data
//!< 1364 data = 21 * 64 + 20 data
//!< donc 23 octets entête, + 21*(64+2) + (20+2) + 1 = 1432
#define HOKUYO_SCAN_BUFFER_SIZE       1432
#define HOKUYO_STACK_SIZE              400
#define HOKUYO_SPEED                750000

const char* hokuyo_scip2_cmd = "SCIP2.0\n";
const char* hokuyo_speed_cmd = "SS750000\n";
const char* hokuyo_laser_on_cmd = "BM\n";
const char* hokuyo_scan_all = "GS0044072500\n";

static uint8_t hokuyo_read_dma_buffer[HOKUYO_SCAN_BUFFER_SIZE];
struct hokuyo_scan hokuyo_scan;
xSemaphoreHandle hokuyo_scan_mutex;

static uint32_t hokuyo_scip2();
static uint32_t hokuyo_set_speed();
static uint32_t hokuyo_laser_on();
static void hokuyo_task();
static void hokuyo_fault_update(uint32_t err);
static uint32_t hokuyo_init();
static void hokuyo_start_scan();
static uint32_t hokuto_wait_decode_scan();
static uint16_t hokuyo_decode16(const unsigned char* data);
static int hokuyo_decode_scan();

int hokuyo_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(hokuyo_task, "hokuyo", HOKUYO_STACK_SIZE, NULL, PRIORITY_TASK_HOKUYO, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_HOKUYO;
	}

	hokuyo_scan_mutex = xSemaphoreCreateMutex();
	if(!hokuyo_scan_mutex)
	{
		return ERR_INIT_HOKUYO;
	}

	return 0;
}

module_init(hokuyo_module_init, INIT_HOKUYO);

static void hokuyo_fault_update(uint32_t err)
{
	if(err)
	{
		error(FAULT_HOKUYO_DISCONNECTED, ERROR_ACTIVE);
	}
	else
	{
		error(FAULT_HOKUYO_DISCONNECTED, ERROR_CLEAR);
	}

	if( err & (ERR_HOKUYO_USART_FE | ERR_HOKUYO_USART_NE | ERR_HOKUYO_USART_ORE | ERR_HOKUYO_CHECK_CMD | ERR_HOKUYO_CHECKSUM | ERR_HOKUYO_UNKNOWN_STATUS))
	{
		error(FAULT_HOKUYO_DATA_CORRUPTION, ERROR_ACTIVE);
	}
	else
	{
		error(FAULT_HOKUYO_DATA_CORRUPTION, ERROR_CLEAR);
	}
}

uint32_t hokuyo_init()
{
	uint32_t err = 0;

	log(LOG_INFO, "Initialisation du hokuyo");

	do
	{
		// tentative a la vitesse d'utilisation (hokuyo déjà configuré)
		usart_open(USART3_FULL_DUPLEX, HOKUYO_SPEED);
		usart_set_read_dma_buffer(USART3_FULL_DUPLEX, hokuyo_read_dma_buffer);

		// on vide tout ce qui traine sur la ligne de reception
		while(err != ERR_USART_TIMEOUT)
		{
			usart_set_read_dma_size(USART3_FULL_DUPLEX, HOKUYO_SCAN_BUFFER_SIZE);
			err = usart_wait_read(USART3_FULL_DUPLEX, ms_to_tick(100));
		}

		err = hokuyo_scip2();
		if(err & ERR_HOKUYO_TIMEOUT)
		{
			// pas de réponse à HOKUYO_SPEED, le hokuyo n'est peut être pas configuré
			// => on tente à la vitesse de base 19200
			usart_set_frequency(USART3_FULL_DUPLEX, 19200);

			err = hokuyo_scip2();

			if(err)
			{
				hokuyo_fault_update(err);
				continue;
			}

			log(LOG_INFO, "hokuyo - set speed");
			// mise a la bonne vitesse
			err = hokuyo_set_speed();

			if(err)
			{
				hokuyo_fault_update(err);
				continue;
			}

			usart_set_frequency(USART3_FULL_DUPLEX, HOKUYO_SPEED);
		}

		if(err)
		{
			hokuyo_fault_update(err);
			continue;
		}

		err = hokuyo_laser_on();
		hokuyo_fault_update(err);
	}
	while(err);

	log(LOG_INFO, "Hokuyo initialisé");

	return 0;
}

static void hokuyo_task()
{
	uint32_t err;
	portTickType last_scan_time;
	portTickType current_time;

	hokuyo_init();

	hokuyo_start_scan();
	// on gruge, le premier scan est plus long
	last_scan_time = systick_get_time() + ms_to_tick(100);

	while(1)
	{
		// on attend la fin du nouveau scan
		err = hokuto_wait_decode_scan();
		hokuyo_fault_update(err);
		if(err)
		{
			hokuyo_init();
			// on gruge, le premier scan est plus long
			last_scan_time = systick_get_time() + ms_to_tick(100);
		}
		else
		{
			// on a un scan toutes les 100ms, ce qui laisse 100ms pour faire le calcul sur l'ancien scan
			// pendant que le nouveau arrive. Si on depasse les 110ms (10% d'erreur), on met un log
			current_time = systick_get_time();
			if( current_time - last_scan_time > ms_to_tick(110) )
			{
				log_format(LOG_ERROR, "slow cycle : %lu us", (long unsigned int) tick_to_us(current_time - last_scan_time));
			}
			last_scan_time = current_time;
		}

		// on lance le prochain scan avant de faire les calculs sur le scan actuel
		hokuyo_start_scan();

		// si le dernier scan n'a pas echoue on fait les calculs
		if( ! err)
		{
			vTaskSetEvent(EVENT_LOCAL_HOKUYO_UPDATE);

			// on envoi les donnees par usb pour le debug
#if defined( __foo__ )
			usb_add(USB_HOKUYO_FOO, &hokuyo_scan, sizeof(hokuyo_scan));
#elif defined( __bar__ )
			usb_add(USB_HOKUYO_BAR, &hokuyo_scan, sizeof(hokuyo_scan));
#else
#error unknown card
#endif
		}
	}

	vTaskDelete(NULL);
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
		if(err & ERR_USART_TIMEOUT)
		{
			err |= ERR_HOKUYO_TIMEOUT;
		}
		if(err & ERR_USART_READ_SR_FE)
		{
			err |= ERR_HOKUYO_USART_FE;
		}
		if(err & ERR_USART_READ_SR_NE)
		{
			err |= ERR_HOKUYO_USART_NE;
		}
		if(err & ERR_USART_READ_SR_ORE)
		{
			err |= ERR_HOKUYO_USART_ORE;
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

uint32_t hokuto_wait_decode_scan()
{
	uint32_t err = usart_wait_read(USART3_FULL_DUPLEX, ms_to_tick(150));

	if(err)
	{
		if(err & ERR_USART_TIMEOUT)
		{
			err |= ERR_HOKUYO_TIMEOUT;
		}
		if(err & ERR_USART_READ_SR_FE)
		{
			err |= ERR_HOKUYO_USART_FE;
		}
		if(err & ERR_USART_READ_SR_NE)
		{
			err |= ERR_HOKUYO_USART_NE;
		}
		if(err & ERR_USART_READ_SR_ORE)
		{
			err |= ERR_HOKUYO_USART_ORE;
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

	xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
	err = hokuyo_decode_scan();
	xSemaphoreGive(hokuyo_scan_mutex);

end:
	return err;
}

uint16_t hokuyo_decode16(const unsigned char* data)
{
	uint16_t val = *data++ - 0x30;
	val <<= 6;
	val &= ~0x3f;
	val |= *data - 0x30;

	return val;
}

int hokuyo_decode_scan()
{
	const unsigned char* buffer = hokuyo_read_dma_buffer;
	uint16_t* distance = hokuyo_scan.distance;

	int j = 0;
	int i = 0;
	uint8_t sum = 0;
	int res = 0;

	// on passe l'entête
	buffer += 23;

	// traitement des pack de 64 data + sum + LF
	for( i = 21; i--; )
	{
		sum = 0;
		for( j = 32 ; j-- ; )
		{
			*distance = hokuyo_decode16(buffer);
			distance++;
			sum += *buffer;
			buffer++;
			sum += *buffer;
			buffer++;
		}

		sum &= 0x3F;
		sum += 0x30;

		if( sum != *buffer)
		{
			// par sécurité, on met le code d'erreur 10
			distance -= 32;
			for( j = 32 ; j-- ; )
			{
				*distance = 10;
				distance++;
			}
			res = ERR_HOKUYO_CHECKSUM;
		}
		buffer+=2;
	}

	// TODO checksum
	// traitement du reste
	for( i = 10 ; i-- ; buffer += 2)
	{
		*distance = hokuyo_decode16(buffer);
		distance++;
	}

	return res;
}