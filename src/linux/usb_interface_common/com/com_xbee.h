#ifndef COM_XBEE_H
#define COM_XBEE_H

#include "com.h"
#include <pthread.h>
#include "rs.h"
#include "kernel/driver/xbee.h"
#include "linux/usb_interface_common/ring_buffer.h"

class ComXbee : public Com
{
	public:
		ComXbee(const char* fileName);
		~ComXbee();

		bool init(const char* fileName);
		int open();
		int close();
		int write(const void* buf, int size);
		int read(void* buf, int size);

	protected:
		uint32_t configure(uint16_t at_cmd, uint32_t val);
		void decodeApiFrame(int frameSize);
		void decodeApiDataFrame(int frameSize);

		unsigned char txBuffer[2048];
		unsigned char rxBuffer[2048];
		RingBuffer m_rxData;
		Rs m_rs;
};

#endif
