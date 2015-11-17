#ifndef HOKUYO_H
#define HOKUYO_H

//! @file hokuyo.h
//! @brief Hokuyo module
//! @author Atlantronic


#include <stdint.h>
#include "kernel/math/VectPlan.h"
#ifndef LINUX
#include "kernel/module.h"
#include "kernel/driver/usart.h"
#include "kernel/FreeRTOS.h"
#include "kernel/semphr.h"
#endif
#include "kernel/systick.h"
#include "kernel/location/location.h"

#define HOKUYO_NUM_POINTS            682
//!< taille de la réponse maxi avec hokuyo_scan_all :
//!< 682 points => 1364 data
//!< 1364 data = 21 * 64 + 20 data
//!< donc 23 octets entête, + 21*(64+2) + (20+2) + 1 = 1432
#define HOKUYO_SCAN_BUFFER_SIZE       1432

#define HOKUYO_DTHETA         	                                        (M_PI / 512.0f)
#define HOKUYO_START_ANGLE               ((- 135 * M_PI / 180.0f) + 44 * HOKUYO_DTHETA)      //!< 135 degrés + 44 HOKUYO_DTHETA
#define HOKUYO_MAX_RANGE                                                           4000
#define HOKUYO_POINT_TO_POINT_DT                                         (0.1f/1024.0f)

struct hokuyo_scan
{
	int id;
	VectPlan pos_robot; //!< position absolue du robot au moment du scan
	VectPlan pos_hokuyo; //!< position du hokuyo dans le repère robot
	signed char sens; //!< sens du hokuyo (1 = vers le haut, -1 = vers le bas)
	uint16_t distance[HOKUYO_NUM_POINTS]; //!< distances des angles 44 à 725 du hokuyo
	systime date;
	float theta_min;
	float theta_max;
	int16_t min_object_size;
	int16_t min_distance;
} __attribute__((packed));

#ifndef LINUX

typedef void (*HokuyoCallback)(void* arg);

class Hokuyo
{
	public:
		__OPTIMIZE_SIZE__ int init(enum usart_id id, const char* name, int hokuyo_id, Location* location);
		void setPosition(VectPlan pos, int sens);

		//!< enregistrement d'une callback
		void registerCallback(HokuyoCallback callback, void* arg);

		xSemaphoreHandle scan_mutex;
		struct hokuyo_scan scan;

	protected:
		static void taskWrapper(void* arg);
		void task();
		uint32_t waitDecodeScan();
		__OPTIMIZE_SIZE__ uint32_t initCom();
		uint32_t scip2();
		uint32_t transaction(unsigned char* buf, uint32_t write_size, uint32_t read_size, portTickType timeout);
		uint32_t checkCmd(unsigned char* cmd, uint32_t size);
		uint32_t checkSum(uint32_t start, uint32_t end);
		uint32_t setSpeed();
		uint32_t hs();
		uint32_t laserOn();
		int decodeScan();
		void faultUpdate(uint32_t err);
		void startScan();

		static uint16_t decode16(const unsigned char* data);

		HokuyoCallback m_callback;
		void* m_callbackArg;
		uint32_t m_lastError;
		enum usart_id m_usartId;
		// variable alignee pour le dma
		uint8_t m_readDmaBuffer[HOKUYO_SCAN_BUFFER_SIZE] __attribute__ ((aligned (16)));
		Location* m_location;
};

#endif

#endif
