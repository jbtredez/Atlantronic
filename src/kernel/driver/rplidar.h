#ifndef RPLIDAR_H
#define RPLIDAR_H

#include "kernel/driver/usart.h"
#include "kernel/location/location.h"
#include "kernel/semphr.h"

#define RPLIDAR_READ_BUFFER_SIZE         2048
#define RPLIDAR_WRITE_BUFFER_SIZE           2
#define RPLIDAR_MAX_NUM_POINTS            768

typedef void (*LaserCallback)(void* arg);

struct rplidar_scan
{
	VectPlan pos_robot; //!< position absolue du robot au moment du scan
	VectPlan pos_laser; //!< position du laser dans le repère robot
	signed char sens; //!< sens du laser (1 = vers le haut, -1 = vers le bas)
	uint16_t theta[RPLIDAR_MAX_NUM_POINTS]; //!< angles en 1/64 deg
	uint16_t distance[RPLIDAR_MAX_NUM_POINTS]; //!< distances en 1/4 mm
	int pointCount;
	int koPointCount;
	systime date;
//	float theta_min;
//	float theta_max;
//	int16_t min_object_size;
	int16_t min_distance;
} __attribute__((packed));

#ifndef LINUX

class Rplidar
{
	public:
		int init(enum usart_id id, const char* name, Location* location);
		void setPosition(VectPlan pos, int sens);

		//!< enregistrement d'une callback
		void registerCallback(LaserCallback callback, void* arg);

		xSemaphoreHandle scan_mutex;
		struct rplidar_scan scan;

	protected:
		static void taskWrapper(void* arg);
		void task();
		uint32_t initCom();
		uint32_t getHealth();
		uint32_t getScan();
		void faultUpdate(uint32_t err);

		LaserCallback m_callback;
		void* m_callbackArg;
		enum usart_id m_usartId;
		Location* m_location;
		uint32_t m_lastError;

		// variable alignee pour le dma
		uint8_t m_readDmaBuffer[RPLIDAR_READ_BUFFER_SIZE] __attribute__ ((aligned (16)));
		uint8_t m_readDmaBuffer2[RPLIDAR_READ_BUFFER_SIZE] __attribute__ ((aligned (16)));
		uint8_t m_writeDmaBuffer[RPLIDAR_WRITE_BUFFER_SIZE] __attribute__ ((aligned (16)));
};

#endif

#endif
