//! @file hokuyo.c
//! @brief Hokuyo module
//! @author Atlantronic

#include "kernel/driver/hokuyo.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/fault.h"
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

#define HOKUYO_STACK_SIZE              400
#define HOKUYO_SPEED                750000
#define HOKUYO_SPEED_PRE_CONFIGURED

const char* hokuyo_scip2_cmd = "SCIP2.0\n";
const char* hokuyo_speed_cmd = "SS750000\n";
const char* hokuyo_hs_cmd = "HS0\n";
//const char* hokuyo_hs_cmd = "HS1\n";
const char* hokuyo_laser_on_cmd = "BM\n";
const char* hokuyo_scan_all = "GS0044072500\n";

int Hokuyo::init(enum usart_id id, const char* name, int hokuyo_id, Location* location)
{
	m_usartId = id;
	scan.id = hokuyo_id;
	m_callback = NULL;
	m_lastError = 0;
	m_location = location;

	portBASE_TYPE err = xTaskCreate(taskWrapper, name, HOKUYO_STACK_SIZE, this, PRIORITY_TASK_HOKUYO, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_HOKUYO;
	}

	scan_mutex = xSemaphoreCreateMutex();
	if( ! scan_mutex )
	{
		return ERR_INIT_HOKUYO;
	}

	return 0;
}

void Hokuyo::setPosition(VectPlan pos, int sens)
{
	scan.pos_hokuyo = pos;
	scan.sens = sens;
}

void Hokuyo::registerCallback(HokuyoCallback callback, void* arg)
{
	m_callback = callback;
	m_callbackArg = arg;
}

void Hokuyo::faultUpdate(uint32_t err)
{
	if(err && ! m_lastError)
	{
		fault(FAULT_HOKUYO_DISCONNECTED, FAULT_ACTIVE);
		log_format(LOG_ERROR, "%s disconnected", pcTaskGetTaskName(NULL));
	}
	else if( !err && m_lastError)
	{
		log_format(LOG_ERROR, "%s connected", pcTaskGetTaskName(NULL));
		fault(FAULT_HOKUYO_DISCONNECTED, FAULT_CLEAR);
	}

	if( err & (ERR_HOKUYO_USART_FE | ERR_HOKUYO_USART_NE | ERR_HOKUYO_USART_ORE | ERR_HOKUYO_CHECK_CMD | ERR_HOKUYO_CHECKSUM | ERR_HOKUYO_UNKNOWN_STATUS))
	{
		log_format(LOG_ERROR, "%s data corruption %x", pcTaskGetTaskName(NULL), (unsigned int)err);
		fault(FAULT_HOKUYO_DATA_CORRUPTION, FAULT_ACTIVE);
	}
	else
	{
		fault(FAULT_HOKUYO_DATA_CORRUPTION, FAULT_CLEAR);
	}

	m_lastError = err;
}

uint32_t Hokuyo::initCom()
{
	uint32_t err = 0;

	log_format(LOG_INFO, "Initialisation %s ...", pcTaskGetTaskName(NULL));

	do
	{
		err = 0;
		// tentative a la vitesse d'utilisation (hokuyo déjà configuré)
		usart_open(m_usartId, HOKUYO_SPEED);
		usart_set_read_dma_buffer(m_usartId, m_readDmaBuffer);

		// on vide tout ce qui traine sur la ligne de reception
		while(err != ERR_USART_TIMEOUT)
		{
			usart_set_read_dma_size(m_usartId, HOKUYO_SCAN_BUFFER_SIZE);
			err = usart_wait_read(m_usartId, ms_to_tick(100));
		}

		err = scip2();
#ifndef HOKUYO_SPEED_PRE_CONFIGURED
		if(err & ERR_HOKUYO_TIMEOUT)
		{
			// pas de réponse à HOKUYO_SPEED, le hokuyo n'est peut être pas configuré
			// => on tente à la vitesse de base 19200
			usart_set_frequency(m_usartId, 19200);

			err = scip2();

			if(err)
			{
				goto retry;
			}

			log_format(LOG_INFO, "%s - set speed", pcTaskGetTaskName(NULL));
			// mise a la bonne vitesse
			err = setSpeed();

			if(err)
			{
				goto retry;
			}

			usart_set_frequency(m_usartId, HOKUYO_SPEED);
		}
#endif
		if(err)
		{
			goto retry;
		}

		err = laserOn();

		if(err)
		{
			goto retry;
		}

		err = hs();
retry:
		faultUpdate(err);
	}
	while(err);

	log_format(LOG_INFO, "%s initialisé", pcTaskGetTaskName(NULL));

	return 0;
}

void Hokuyo::taskWrapper(void* arg)
{
	Hokuyo* h = (Hokuyo*) arg;
	h->task();
}

void Hokuyo::task()
{
	uint32_t err;
	struct systime last_scan_time;
	struct systime current_time;
	initCom();

	startScan();
	// on gruge, le premier scan est plus long
	last_scan_time = systick_get_time();
	last_scan_time.ms += 100;

	while(1)
	{
		// on attend la fin du nouveau scan
		err = waitDecodeScan();
		scan.pos_robot = m_location->getPosition(); // TODO voir si meilleur moment
		faultUpdate(err);
		if(err)
		{
			initCom();
			// on gruge, le premier scan est plus long
			last_scan_time = systick_get_time();
			last_scan_time.ms += 100;
		}
		else
		{
			// on a un scan toutes les 100ms, ce qui laisse 100ms pour faire le calcul sur l'ancien scan
			// pendant que le nouveau arrive. Si on depasse les 110ms (10% d'erreur), on met un log
			current_time = systick_get_time();
			if( current_time.ms - last_scan_time.ms > 110 )
			{
				log_format(LOG_ERROR, "slow cycle : %lu ms", current_time.ms - last_scan_time.ms);
			}
			last_scan_time = current_time;
		}

		// on lance le prochain scan avant de faire les calculs sur le scan actuel
		startScan();

		// si le dernier scan n'a pas echoue on reveille la tache detection
		if( ! err)
		{
			scan.date = current_time;

			if(m_callback)
			{
				m_callback(m_callbackArg);
			}

			// on envoi les donnees par usb pour le debug
			usb_add(USB_HOKUYO, &scan, sizeof(scan));
		}
	}
}

//! Vérifie que la commande envoyée est bien renvoyée par le hokuyo
//! @return 0 si c'est bon
//! @return ERR_HOKUYO_CHECK_CMD sinon
uint32_t Hokuyo::checkCmd(unsigned char* cmd, uint32_t size)
{
	for(uint32_t i = 0; i < size; i++)
	{
		if(cmd[i] != m_readDmaBuffer[i])
		{
			return ERR_HOKUYO_CHECK_CMD;
		}
	}

	return 0;
}

uint32_t Hokuyo::checkSum(uint32_t start, uint32_t end)
{
	uint8_t sum = 0;
	uint32_t err = 0;

	for(; start < end; start++)
	{
		sum += m_readDmaBuffer[start];
	}

	sum &= 0x3F;
	sum += 0x30;

	if(sum != m_readDmaBuffer[end])
	{
		err = ERR_HOKUYO_CHECKSUM;
	}

	return err;
}

//! Envoi une commande, attend la reponse du hokuyo, vérifie si le hokuyo fait bien un echo de la commande et le checksum du status
//! @return 0 si ok
uint32_t Hokuyo::transaction(unsigned char* buf, uint32_t write_size, uint32_t read_size, portTickType timeout)
{
	usart_set_write_dma_buffer(m_usartId, buf);

	usart_set_read_dma_size(m_usartId, read_size);
	usart_send_dma_buffer(m_usartId, write_size);
	uint32_t err = usart_wait_read(m_usartId, timeout);

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
		return err;
	}

	err = checkCmd(buf, write_size);

	if(err)
	{
		return err;
	}

	return checkSum(write_size, write_size+2);
}

uint32_t Hokuyo::scip2()
{
	uint32_t err = transaction((unsigned char*) hokuyo_scip2_cmd, 8, 13, ms_to_tick(100));

	if(err)
	{
		return err;
	}

	if( m_readDmaBuffer[8] != '0')
	{
		return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	if( m_readDmaBuffer[9] != '0' &&  m_readDmaBuffer[9] != 'E')
	{
		return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	return 0;
}

uint32_t Hokuyo::setSpeed()
{
	uint32_t err = transaction((unsigned char*) hokuyo_speed_cmd, 9, 14, ms_to_tick(100));

	if(err)
	{
		return err;
	}

	if( m_readDmaBuffer[9] != '0')
	{
		return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	switch(m_readDmaBuffer[10])
	{
		case '0':
		case '3':
			// OK
			break;
		case '1':
		case '2':
			return ERR_HOKUYO_BAUD_RATE;
		default:
			return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	return 0;
}

uint32_t Hokuyo::hs()
{
	log_format(LOG_INFO, "%s - hs", pcTaskGetTaskName(NULL));

	uint32_t err = transaction((unsigned char*) hokuyo_hs_cmd, 4, 9, ms_to_tick(200));

	if(err)
	{
		return err;
	}

	if( m_readDmaBuffer[4] != '0')
	{
		return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	if( m_readDmaBuffer[5] != '0' && m_readDmaBuffer[5] != '2')
	{
		return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	return 0;
}

uint32_t Hokuyo::laserOn()
{
	log_format(LOG_INFO, "%s - laser on", pcTaskGetTaskName(NULL));

	uint32_t err = transaction((unsigned char*) hokuyo_laser_on_cmd, 3, 8, ms_to_tick(300));

	if(err)
	{
		return err;
	}

	if( m_readDmaBuffer[3] != '0')
	{
		log_format(LOG_ERROR, "%s - laser on - unknown status %c%c", pcTaskGetTaskName(NULL), m_readDmaBuffer[3], m_readDmaBuffer[4]);
		return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	switch(m_readDmaBuffer[4])
	{
		case '0':
		case '2':
			// OK
			break;
		case '1':
			return ERR_HOKUYO_LASER_MALFUNCTION;
		default:
			log_format(LOG_ERROR, "%s - laser on - unknown status %c%c", pcTaskGetTaskName(NULL), m_readDmaBuffer[3], m_readDmaBuffer[4]);
			return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	return 0;
}

void Hokuyo::startScan()
{
	usart_set_write_dma_buffer(m_usartId, (unsigned char*)hokuyo_scan_all);
	usart_set_read_dma_size(m_usartId, HOKUYO_SCAN_BUFFER_SIZE);
	usart_send_dma_buffer(m_usartId, 13);
}

uint32_t Hokuyo::waitDecodeScan()
{
	uint32_t err = usart_wait_read(m_usartId, ms_to_tick(150));

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
		return err;
	}

	err = checkCmd((unsigned char*)hokuyo_scan_all, 13);

	if(err)
	{
		return err;
	}

	err = checkSum(13, 15);

	if(err)
	{
		return err;
	}
	
	if( m_readDmaBuffer[13] != '0')
	{
		return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	switch(m_readDmaBuffer[14])
	{
		case '0':
		case '2':
			// OK
			break;
		case '1':
			return ERR_HOKUYO_LASER_MALFUNCTION;
		default:
			return ERR_HOKUYO_UNKNOWN_STATUS;
	}

	xSemaphoreTake(scan_mutex, portMAX_DELAY);
	err = decodeScan();
	xSemaphoreGive(scan_mutex);

	return err;
}

uint16_t Hokuyo::decode16(const unsigned char* data)
{
	uint16_t val = *data++ - 0x30;
	val <<= 6;
	val &= ~0x3f;
	val |= *data - 0x30;

	return val;
}

int Hokuyo::decodeScan()
{
	const unsigned char* buffer = m_readDmaBuffer;
	uint16_t* distance = scan.distance;

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
			*distance = decode16(buffer);
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
		*distance = decode16(buffer);
		distance++;
	}

	return res;
}
