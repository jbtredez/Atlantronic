#ifndef COM_USB_H
#define COM_USB_H

#include "com.h"

class ComUsb : public Com
{
	public:
		ComUsb(const char* file_read, const char* file_write);
		~ComUsb();

		int open();
		int close();
		int write(const void* buf, int size);
		int read(void* buf, int size);

	protected:
		int fd_read;
		int fd_write;
		char* file_read;
		char* file_write;
};

#endif
