#ifndef COM_H
#define COM_H

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include "kernel/driver/usb.h"

class Com
{
	public:
		// TODO faire 2 classes comUsb et ComTcp
		void init(const char* file_read, const char* file_write, const char* ip);

		void destroy();
		int close();
		int open();
		void open_block();
		int read(int min_buffer_size);
		int read_header(struct usb_header* header);
		void copy(void* data, int offset, int size);
		void skip(int count);
		int write(const void* buf, int size);

		int fd_read;
		int fd_write;
		char* file_read;
		char* file_write;
		char* ip;
		unsigned char buffer[65536];
		int buffer_end;
		int buffer_begin;
		int buffer_size;
		bool opened;
};

#endif
