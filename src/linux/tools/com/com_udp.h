#ifndef COM_UDP_H
#define COM_UDP_H

#include "com.h"

class ComUdp : public Com
{
	public:
		ComUdp(const char* ip);
		~ComUdp();

		int open();
		int close();
		int write(const void* buf, int size);
		int read(void* buf, int size);

	protected:
		int fd;
		char* ip;
};

#endif
