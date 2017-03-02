#ifndef COM_TCP_H
#define COM_TCP_H

#include "com.h"

class ComTcp : public Com
{
	public:
		ComTcp(const char* ip);
		~ComTcp();

		int open();
		int close();
		int write(const void* buf, int size);
		int read(void* buf, int size);

	protected:
		int fd;
		char* ip;
};

#endif
