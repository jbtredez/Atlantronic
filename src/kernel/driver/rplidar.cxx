#include "rplidar.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/fault.h"

#define RPLIDAR_STACK_SIZE              400
#define RPLIDAR_SPEED                115200

#define RPLIDAR_TX_START_FLAG          0xa5
#define RPLIDAR_RX_START_FLAG        0x5aa5

#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_RESET              0x40
#define RPLIDAR_CMD_GET_INFO           0x50
#define RPLIDAR_CMD_GET_HEALTH         0x52

#define RPLIDAR_SEND_MODE_SINGLE_RESPONSE     0
#define RPLIDAR_SEND_MODE_MULTIPLE_RESPONSE   1

#define RPLIDAR_DATA_TYPE_INFO         0x04
#define RPLIDAR_DATA_TYPE_HEALTH       0x06
#define RPLIDAR_DATA_TYPE_SCAN         0x81

#define RPLIDAR_STATUS_OK              0x01
#define RPLIDAR_STATUS_WARNING         0x02
#define RPLIDAR_STATUS_ERROR           0x03

//#define ERR_RPLIDAR_TIMEOUT              0x01
//#define ERR_RPLIDAR_USART_FE             0x02
//#define ERR_RPLIDAR_USART_NE             0x04
//#define ERR_RPLIDAR_USART_ORE            0x08
#define ERR_RPLIDAR_CHECK_HEADER             0x10

struct RplidarRxDescription
{
	uint16_t startFlag;
	uint32_t length : 30;
	uint32_t sendMode : 2;
	uint8_t dataType;
}__attribute__((packed));

struct RplidarRxHealthStatus
{
	RplidarRxDescription header;
	uint8_t status;
	uint16_t errorCode;
}__attribute__((packed));

struct RplidarRxMeasurement
{
	uint8_t startScan : 2;
	uint8_t quality : 6;
	uint16_t c : 1;
	uint16_t theta : 15;
	uint16_t distance;
}__attribute__((packed));

int Rplidar::init(enum usart_id id, const char* name, Location* location)
{
	m_usartId = id;
	m_location = location;

	portBASE_TYPE err = xTaskCreate(taskWrapper, name, RPLIDAR_STACK_SIZE, this, PRIORITY_TASK_RPLIDAR, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_HOKUYO;
	}

	return 0;
}

void Rplidar::setPosition(VectPlan pos, int sens)
{
	scan.pos_laser = pos;
	scan.sens = sens;
}

void Rplidar::registerCallback(LaserCallback callback, void* arg)
{
	m_callback = callback;
	m_callbackArg = arg;
}

void Rplidar::taskWrapper(void* arg)
{
	Rplidar* h = (Rplidar*) arg;
	h->task();
}

void Rplidar::task()
{
	while(1)
	{
		initCom();
		xSemaphoreTake(scan_mutex, portMAX_DELAY);
		getScan();
		xSemaphoreGive(scan_mutex);


		vTaskDelay(ms_to_tick(100));
	}
}

uint32_t Rplidar::initCom()
{
	uint32_t err = 0;

	log_format(LOG_INFO, "Initialisation %s ...", pcTaskGetTaskName(NULL));

	do
	{
		err = 0;
		// tentative a la vitesse d'utilisation (hokuyo déjà configuré)
		usart_open(m_usartId, RPLIDAR_SPEED);
		usart_set_read_dma_buffer(m_usartId, m_readDmaBuffer);
		usart_set_write_dma_buffer(m_usartId, m_writeDmaBuffer);

		m_writeDmaBuffer[0] = RPLIDAR_TX_START_FLAG;
		m_writeDmaBuffer[1] = RPLIDAR_CMD_STOP; // arret des scan si le rplidar est deja en marche
		usart_send_dma_buffer(m_usartId, 2);

		// on vide tout ce qui traine sur la ligne de reception
		while(err != ERR_USART_TIMEOUT)
		{
			usart_set_read_dma_size(m_usartId, RPLIDAR_READ_BUFFER_SIZE);
			err = usart_wait_read(m_usartId, ms_to_tick(100));
		}

		err = getHealth();

		faultUpdate(err);
	}
	while(err);

	log_format(LOG_INFO, "%s initialisé", pcTaskGetTaskName(NULL));

	return 0;
}

uint32_t Rplidar::getScan()
{
	RplidarRxMeasurement* mes = (RplidarRxMeasurement*)m_readDmaBuffer;

	log_format(LOG_INFO, "%s start scanning", pcTaskGetTaskName(NULL));

	m_writeDmaBuffer[0] = RPLIDAR_TX_START_FLAG;
	m_writeDmaBuffer[1] = RPLIDAR_CMD_SCAN;

	usart_set_read_dma_buffer(m_usartId, m_readDmaBuffer);
	usart_set_read_dma_size(m_usartId, sizeof(RplidarRxDescription));
	usart_send_dma_buffer(m_usartId, 2);
	uint32_t err = usart_wait_read(m_usartId, ms_to_tick(10));
	usart_set_read_dma_size(m_usartId, 100*sizeof(RplidarRxMeasurement));

	RplidarRxDescription* scanDesc = (RplidarRxDescription*)m_readDmaBuffer;
	if( err )
	{
		return err;
	}

	if( scanDesc->startFlag != RPLIDAR_RX_START_FLAG ||
		scanDesc->length != sizeof(RplidarRxMeasurement) ||
		scanDesc->sendMode != RPLIDAR_SEND_MODE_MULTIPLE_RESPONSE ||
		scanDesc->dataType != RPLIDAR_DATA_TYPE_SCAN)
	{
		return ERR_RPLIDAR_CHECK_HEADER;
	}

	log_format(LOG_INFO, "%s start scanning : ok", pcTaskGetTaskName(NULL));

	//systime t0 = systick_get_time();
	//int scanCount = -1;
	while( ! err )
	{
		err = usart_wait_read(m_usartId, ms_to_tick(1000));
		// on prend l'autre buffer, le temps de depiler les messages
		if( mes == (RplidarRxMeasurement*)m_readDmaBuffer )
		{
			usart_set_read_dma_buffer(m_usartId, m_readDmaBuffer2);
		}
		else
		{
			usart_set_read_dma_buffer(m_usartId, m_readDmaBuffer);
		}
		usart_set_read_dma_size(m_usartId, 100*sizeof(RplidarRxMeasurement));
		if( ! err )
		{
			for(int i = 0; i < 100; i++)
			{
				if( (mes[i].startScan != 1 && mes[i].startScan != 2) ||
					mes[i].c != 1)
				{
					// erreur de communication
					log(LOG_ERROR, "rplidar - com error c != 1");
					return ERR_RPLIDAR_CHECK_HEADER;
				}
				if( mes[i].startScan == 1)
				{
					/*
					// tests de perf
					if( scanCount == -1)
					{
						t0 = systick_get_time();
						log_format(LOG_INFO, "new scan");
						scanCount++;
					}
					else
					{
						scanCount++;
						systime t1 = systick_get_time();
						log_format(LOG_INFO, "new scan - mean delay %d - last %d pt (%d valid)", (int) (t1 - t0).ms/scanCount, scan.pointCount + scan.koPointCount, scan.pointCount );
					}*/

					if(m_callback)
					{
						m_callback(m_callbackArg);
					}

					usb_add(USB_RPLIDAR, &scan, sizeof(scan));
					scan.pointCount = 0;
					scan.koPointCount = 0;
					scan.pos_robot = m_location->getPosition();

				}
				if( mes[i].quality )
				{
					if( scan.pointCount < RPLIDAR_MAX_NUM_POINTS )
					{
						scan.theta[scan.pointCount] = mes[i].theta;
						scan.distance[scan.pointCount] = mes[i].distance;
						scan.pointCount++;
					}
					//log_format(LOG_INFO, "ss %d theta %d distance %d qualite %d", (int)mes[i].startScan, (int)(mes[i].theta/64.0f), (int)(mes[i].distance/4.0f), mes[i].quality);
				}
				else
				{
					scan.koPointCount++;
				}
			}
		}
		if( mes == (RplidarRxMeasurement*)m_readDmaBuffer )
		{
			mes = (RplidarRxMeasurement*)m_readDmaBuffer2;
		}
		else
		{
			mes = (RplidarRxMeasurement*)m_readDmaBuffer;
		}
	}

	return 0;
}

uint32_t Rplidar::getHealth()
{
	m_writeDmaBuffer[0] = RPLIDAR_TX_START_FLAG;
	m_writeDmaBuffer[1] = RPLIDAR_CMD_GET_HEALTH;

	usart_set_read_dma_buffer(m_usartId, m_readDmaBuffer);
	usart_set_read_dma_size(m_usartId, sizeof(RplidarRxHealthStatus));
	usart_send_dma_buffer(m_usartId, 2);
	uint32_t err = usart_wait_read(m_usartId, ms_to_tick(2000));

	RplidarRxHealthStatus* healthStatus = (RplidarRxHealthStatus*)m_readDmaBuffer;
	if( err )
	{
		return err;
	}

	if( healthStatus->header.startFlag != RPLIDAR_RX_START_FLAG ||
		healthStatus->header.length != sizeof(RplidarRxHealthStatus) - sizeof(RplidarRxDescription) ||
		healthStatus->header.sendMode != RPLIDAR_SEND_MODE_SINGLE_RESPONSE ||
		healthStatus->header.dataType != RPLIDAR_DATA_TYPE_HEALTH)
	{
		return ERR_RPLIDAR_CHECK_HEADER;
	}
	/*const char* status[]
	{
		"good",
		"warning",
		"error"
	}*/
	log_format(LOG_INFO, "healthStatus : status %d errorCode %d", (int)healthStatus->status, (int)healthStatus->errorCode);

	return 0;
}

void Rplidar::faultUpdate(uint32_t err)
{
	if(err && ! m_lastError)
	{
		fault(FAULT_RPLIDAR_DISCONNECTED, FAULT_ACTIVE);
		log_format(LOG_ERROR, "%s disconnected", pcTaskGetTaskName(NULL));
	}
	else if( !err && m_lastError)
	{
		log_format(LOG_ERROR, "%s connected", pcTaskGetTaskName(NULL));
		fault(FAULT_RPLIDAR_DISCONNECTED, FAULT_CLEAR);
	}

	if( err & (ERR_USART_READ_SR_FE | ERR_USART_READ_SR_NE | ERR_USART_READ_SR_ORE ))
	{
		log_format(LOG_ERROR, "%s data corruption %x", pcTaskGetTaskName(NULL), (unsigned int)err);
		fault(FAULT_RPLIDAR_DATA_CORRUPTION, FAULT_ACTIVE);
	}
	else
	{
		fault(FAULT_RPLIDAR_DATA_CORRUPTION, FAULT_CLEAR);
	}

	m_lastError = err;
}

