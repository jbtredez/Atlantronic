#ifndef RS_H
#define RS_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

class Rs
{
	public:
		Rs();

		bool configure(const char* fileName, int baudrate, int nData, int vTimeMs);
		int open();
		ssize_t read(void *buffer, int length);
		ssize_t write(const void* buffer, int length);
		int close();

	protected:
		char m_fileName[256];
		int m_vTimeMs;
		tcflag_t m_cflags;
		int m_fd;
};

#endif
