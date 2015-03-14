#ifndef COM_H
#define COM_H

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <pthread.h>
#include "kernel/driver/usb.h"
#include "linux/tools/cli.h"

class Com
{
	public:
		Com();

		virtual int open();
		virtual int close() = 0;
		virtual int write(const void* buf, int size) = 0;

		void open_block();
		int read(int min_buffer_size);
		int read_header(struct usb_header* header);
		void copy(void* data, int offset, int size);
		void skip(int count);

		unsigned char buffer[65536];
		int buffer_end;
		int buffer_begin;
		int buffer_size;
		bool opened;
	protected:
		virtual int read(void* buf, int size) = 0;
};

#endif
