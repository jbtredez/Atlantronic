#ifndef COM_H
#define COM_H

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>

class Com
{
	public:
		void init(const char* file_read, const char* file_write);
		void destroy();
		int close();
		int open();
		void open_block();
		int read(int min_buffer_size);
		int read_header(uint16_t* type, uint16_t* size);
		void copy(void* data, int offset, int size);
		void skip(int count);
		int write(const void* buf, int size);

		int fd_read;
		int fd_write;
		char* file_read;
		char* file_write;
		unsigned char buffer[65536];
		int buffer_end;
		int buffer_begin;
		int buffer_size;
};

#endif
