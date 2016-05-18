#ifndef COM_UDP_H
#define COM_UDP_H

#include "com.h"
#include "linux/tools/ring_buffer.h"
class ServerUdp ;
class ComUdp : public Com
{
	public:
		ComUdp(ServerUdp *Serveur,const char * ip);
		~ComUdp();
		int write(const void* buf, int size);
		int read(void* buf, int size);
		void save(unsigned char * msg,int size );
		int open();
		virtual int close() {return 1;};
	private :
		ServerUdp * m_pServeur;
		unsigned char txBuffer[2048];
		unsigned char rxBuffer[2048];
		RingBuffer m_rxData;
};

#endif
